% collectSystemResponse.m
% Script para coletar dados de resposta do sistema a um sinal de degrau.
% Os dados coletados serão usados para identificação do sistema no System Identification Toolbox.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- INICIANDO COLETA DE DADOS DE RESPOSTA DO SISTEMA ---');

% Carrega as definições de pinos (strings) e calibração do potenciômetro
try
    load('arduinoPins.mat', 'potPin', 'motorPin1', 'motorPin2'); % Carrega pinos do setupPins.m
    load('potentiometerCalibration.mat', 'minVoltage', 'maxVoltage', 'angleRangeDegrees', 'voltsPerDegree'); % Carrega calibração do novo arquivo
    if ~exist('potPin', 'var') || ~exist('minVoltage', 'var')
        error('Dependências não carregadas. Execute setupPins.m e calibratePotentiometer.m primeiro.');
    end
    disp('Configurações e calibração carregadas com sucesso!');
    % --- NOVOS LOGS: Verificar pinos e calibração carregados ---
    fprintf('  Pino do Potenciômetro (potPin): %s\n', potPin);
    fprintf('  Pino do Motor 1 (motorPin1): %s\n', motorPin1);
    fprintf('  Pino do Motor 2 (motorPin2): %s\n', motorPin2);
    fprintf('  Calibração: minVoltage=%.4fV, maxVoltage=%.4fV, angleRangeDegrees=%.2f, voltsPerDegree=%.4fV/deg\n', ...
            minVoltage, maxVoltage, angleRangeDegrees, voltsPerDegree);
catch ME
    disp(['ERRO ao carregar definições: ', ME.message]);
    disp('Por favor, execute setupPins.m e calibratePotentiometer.m antes de prosseguir.');
    return;
end

% --- RECRIAR O OBJETO ARDUINO NESTE SCRIPT ---
a = []; % Inicializa 'a' como vazio
try
    % Substitua 'COMx' pela porta serial da sua ESP32
    % Certifique-se que o nome do módulo ESP32 está correto!
    a = arduino("COM9", "ESP32-WROOM-DevKitV1"); % OU "ESP32 Dev Module" - Verifique o nome exato!
    disp('Conexão com ESP32 estabelecida para este script!');
    % --- NOVO LOG: Verificar se o objeto Arduino é válido ---
    if isvalid(a)
        disp('  Objeto Arduino criado e válido.');
    else
        disp('  AVISO: Objeto Arduino não é válido após a criação.');
        clear a; return;
    end
catch ME
    disp(['ERRO ao conectar à ESP32: ', ME.message]);
    disp('Verifique se a placa está conectada, os drivers instalados e a porta COM correta.');
    return; % Sai do script se não conseguir conectar
end

% Configura os pinos do motor como PWM
configurePin(a, motorPin1, "PWM");
configurePin(a, motorPin2, "PWM");
disp('Pinos do motor configurados como PWM.');

% --- PARÂMETROS DO CONTROLE E DA COLETA DE DADOS ---
Ts = 0.05; % Tempo de amostragem em segundos (50 ms)
stabilizeTime = 10; % Tempo para estabilizar na posição inicial (segundos)
maxPWMDutyCycle = 0.5; % <--- AUMENTADO: Aumenta a potência máxima do motor para 50%

% Ganhos PID temporários para a coleta de dados (somente Kp, Ki e Kd são 0 para simplificar)
tempKp = 0.02; % <--- AUMENTADO: Ganho Proporcional para resposta mais forte
tempKi = 0;     % Ganho Integral (zero para não acumular erro)
tempKd = 0;     % Ganho Derivativo (zero para não reagir a variações rápidas)

% Sequência de ângulos alvo para os degraus
targetAngles = [45, 90, 0]; % Move para 45, depois 90, e volta para 0.

% Pre-alocação de arrays para armazenar os dados
durationPerStep = 10; % Duração de cada degrau em segundos
totalSamples = (stabilizeTime + length(targetAngles) * durationPerStep) / Ts;
outputData = zeros(totalSamples, 1); % Posição do potenciômetro
inputData = zeros(totalSamples, 1);  % Sinal de controle aplicado (PWM)
timeData = zeros(totalSamples, 1);   % Tempo

currentSampleIdx = 1; % Índice global para os dados

disp('Iniciando a sequência de degraus para coleta de dados...');
disp('Certifique-se de que o motor pode se mover livremente.');

% --- Estabilização na posição inicial (0 graus) ---
initialTarget = 0;
fprintf('Movendo para a posição inicial (%.2f graus)...\n', initialTarget);
startTime = tic; % Inicia o cronômetro para a estabilização

while toc(startTime) < stabilizeTime
    currentVoltage = readVoltage(a, potPin);
    currentPositionDegrees = (currentVoltage - minVoltage) / voltsPerDegree;
    % Limita a posição para o range calibrado para evitar valores absurdos
    currentPositionDegrees = max(0, min(angleRangeDegrees, currentPositionDegrees));

    error = initialTarget - currentPositionDegrees;
    
    % Calcula o sinal de controle (apenas P)
    controlSignal = tempKp * error;

    % Aplica o sinal de controle ao motor (considerando a direção corrigida)
    if controlSignal > 0 % Precisa aumentar o ângulo (sentido horário com D25)
        pwmDutyCycle = min([controlSignal, maxPWMDutyCycle]);
        writePWMDutyCycle(a, motorPin1, pwmDutyCycle); % Ativa D25
        writePWMDutyCycle(a, motorPin2, 0);
    elseif controlSignal < 0 % Precisa diminuir o ângulo (sentido anti-horário com D26)
        pwmDutyCycle = min([abs(controlSignal), maxPWMDutyCycle]);
        writePWMDutyCycle(a, motorPin1, 0);
        writePWMDutyCycle(a, motorPin2, pwmDutyCycle); % Ativa D26
    else % Se o erro for zero, parar o motor
        writePWMDutyCycle(a, motorPin1, 0);
        writePWMDutyCycle(a, motorPin2, 0);
    end

    % Armazena os dados
    outputData(currentSampleIdx) = currentPositionDegrees;
    inputData(currentSampleIdx) = pwmDutyCycle;
    timeData(currentSampleIdx) = toc(startTime);

    if mod(currentSampleIdx-1, 10) == 0 % Imprime a cada 10 amostras
        fprintf('  Estabilização #%d: Pos=%.2f°, Erro=%.2f°, CtrlSig=%.3f, PWM_App=%.3f\n', ...
                currentSampleIdx, currentPositionDegrees, error, controlSignal, pwmDutyCycle);
    end
    
    currentSampleIdx = currentSampleIdx + 1; % Incrementa o índice global
    pause(Ts);
end
writePWMDutyCycle(a, motorPin1, 0); % Parar o motor
writePWMDutyCycle(a, motorPin2, 0);
fprintf('Posição inicial (%.2f graus) atingida e estabilizada.\n', initialTarget);
pause(1); % Pequena pausa antes de iniciar os degraus

% --- Aplicação dos degraus ---
for i = 1:length(targetAngles)
    targetAngle = targetAngles(i);
    fprintf('Aplicando degrau para: %.2f graus\n', targetAngle);
    startTime = tic; % Inicia o cronômetro para este degrau

    while toc(startTime) < durationPerStep
        currentVoltage = readVoltage(a, potPin);
        currentPositionDegrees = (currentVoltage - minVoltage) / voltsPerDegree;
        currentPositionDegrees = max(0, min(angleRangeDegrees, currentPositionDegrees));

        error = targetAngle - currentPositionDegrees;
        
        % Calcula o sinal de controle (apenas P)
        controlSignal = tempKp * error;

        % Aplica o sinal de controle ao motor (considerando a direção corrigida)
        if controlSignal > 0 % Precisa aumentar o ângulo (sentido horário com D25)
            pwmDutyCycle = min([controlSignal, maxPWMDutyCycle]);
            writePWMDutyCycle(a, motorPin1, pwmDutyCycle); % Ativa D25
            writePWMDutyCycle(a, motorPin2, 0);
        elseif controlSignal < 0 % Precisa diminuir o ângulo (sentido anti-horário com D26)
            pwmDutyCycle = min([abs(controlSignal), maxPWMDutyCycle]);
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, pwmDutyCycle); % Ativa D26
        else % Se o erro for zero, parar o motor
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, 0);
        end
        
        % Armazena os dados
        outputData(currentSampleIdx) = currentPositionDegrees;
        inputData(currentSampleIdx) = pwmDutyCycle;
        timeData(currentSampleIdx) = timeData(currentSampleIdx-1) + Ts; % Tempo acumulado
        
        if mod(currentSampleIdx - (length(targetAngles)*(i-1)*durationPerStep/Ts), 10) == 1 % Imprime a cada 10 amostras do degrau atual
            fprintf('  Degrau %d (#%d): SP=%.2f°, Pos=%.2f°, Erro=%.2f°, CtrlSig=%.3f, PWM_App=%.3f\n', ...
                    i, currentSampleIdx - ((i-1)*durationPerStep/Ts), targetAngle, currentPositionDegrees, error, controlSignal, pwmDutyCycle);
        end
        
        currentSampleIdx = currentSampleIdx + 1; % Incrementa o índice global
        pause(Ts);
    end
    writePWMDutyCycle(a, motorPin1, 0); % Parar o motor no final de cada degrau
    writePWMDutyCycle(a, motorPin2, 0);
    fprintf('Degrau para %.2f graus finalizado.\n', targetAngle);
    pause(0.5); % Pequena pausa entre os degraus
end

disp('Coleta de dados concluída. Salvando...');

% Limpar dados extras se o totalSamples foi superestimado
outputData(currentSampleIdx:end) = [];
inputData(currentSampleIdx:end) = [];
timeData(currentSampleIdx:end) = [];

% Criar objeto iddata para o System Identification Toolbox
% input: sinal de controle aplicado
% output: posição do potenciômetro
data = iddata(outputData, inputData, Ts);

% Salvar os dados para uso posterior
save('systemResponseData.mat', 'data');
disp('Dados salvos em systemResponseData.mat');

% Plotar os dados para visualização rápida
figure;
subplot(2,1,1);
plot(data.SamplingInstants, data.OutputData);
title('Resposta do Sistema (Posição do Potenciômetro)');
xlabel('Tempo (s)');
ylabel('Posição (graus)');
grid on;

subplot(2,1,2);
plot(data.SamplingInstants, data.InputData);
title('Sinal de Controle Aplicado (PWM)');
xlabel('Tempo (s)');
ylabel('PWM Duty Cycle');
grid on;

% Fechar conexão Arduino
clear a;
disp('Conexão com ESP32 fechada para este script.');