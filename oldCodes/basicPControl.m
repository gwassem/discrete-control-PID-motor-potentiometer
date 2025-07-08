% basicPControl.m
% Script para implementar um controle de posição de malha fechada com um
% controlador Proporcional (P) básico.
% Os dados coletados serão usados para comparar com o desempenho do PID tunado.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- INICIANDO CONTROLE PROPORCIONAL BÁSICO ---');

% 1. Carregar Definições de Pinos
% Tenta carregar as variáveis de pino salvas por 'setupPins.m'.
try
    load('arduinoPins.mat', 'potPin', 'motorPin1', 'motorPin2');
    if ~exist('potPin', 'var') || ~exist('motorPin1', 'var') || ~exist('motorPin2', 'var')
        error('Pinos (potPin, motorPin1, motorPin2) não carregados. Execute setupPins.m primeiro.');
    end
    fprintf('Pino do Potenciômetro (potPin): %s\n', potPin);
    fprintf('Pino do Motor IN1 (motorPin1): %s\n', motorPin1);
    fprintf('Pino do Motor IN2 (motorPin2): %s\n', motorPin2);
catch ME
    disp(['ERRO ao carregar definições de pinos: ', ME.message]);
    disp('Por favor, execute setupPins.m antes de prosseguir.');
    return; % Sai do script se não conseguir carregar os pinos
end

% 2. Carregar Parâmetros de Calibração do Potenciômetro
% Tenta carregar os parâmetros de calibração salvos por 'calibratePotentiometer.m'.
try
    load('potentiometerCalibration.mat', 'minVoltage', 'maxVoltage', 'angleRangeDegrees', 'voltsPerDegree');
    if ~exist('minVoltage', 'var') || ~exist('voltsPerDegree', 'var')
        error('Parâmetros de calibração não carregados. Execute calibratePotentiometer.m primeiro.');
    end
    fprintf('Tensão Mínima (0 graus): %.4fV\n', minVoltage);
    fprintf('Volts por Grau: %.4fV/deg\n', voltsPerDegree);
catch ME
    disp(['ERRO ao carregar parâmetros de calibração: ', ME.message]);
    disp('Por favor, execute calibratePotentiometer.m antes de prosseguir.');
    return; % Sai do script se não conseguir carregar a calibração
end

% 3. Recriar o Objeto Arduino
% É necessário recriar o objeto 'arduino' em cada script que interage com a placa.
a = []; % Inicializa 'a' como vazio
try
    % Substitua 'COM9' pela porta serial da sua ESP32, se for diferente.
    a = arduino("COM9", "ESP32-WROOM-DevKitV1");
    disp('Conexão com ESP32 estabelecida para este script!');
    % Configurar os pinos do motor como PWM novamente, para garantir.
    configurePin(a, motorPin1, "PWM");
    configurePin(a, motorPin2, "PWM");
    disp('Pinos do motor configurados como PWM.');
catch ME
    disp(['ERRO ao conectar à ESP32 ou configurar pinos: ', ME.message]);
    disp('Verifique se a placa está conectada e a porta COM correta.');
    return; % Sai do script se não conseguir conectar
end

% --- Parâmetros do Controle P Básico ---
% Ganho Proporcional (Kp): Este valor precisará ser ajustado por tentativa e erro.
% Um Kp muito pequeno resultará em resposta lenta e grande erro em regime permanente.
% Um Kp muito grande pode causar oscilações ou instabilidade.
Kp = 0.005; % Valor inicial. Comece com um valor pequeno e aumente gradualmente.

% --- Limites de Potência (Duty Cycle) do Motor ---
% PWM_MAX: Limite superior para o duty cycle (60% da potência máxima).
PWM_MAX = 0.60;
% PWM_MIN_ACTIVE: Limite inferior para o duty cycle que garante que o motor se mova.
% Se a potência for menor que isso (e diferente de zero), o motor pode não girar.
% Escolha entre 0.10 (10%) ou 0.15 (15%) com base nos seus testes físicos.
PWM_MIN_ACTIVE = 0.15; % Usando 15% como mínimo. Ajuste conforme necessário.

% Setpoints (posições desejadas em graus)
% Vamos testar dois setpoints para ver a resposta a degraus de referência.
setPoints = [180, 360]; % Certifique-se de que esses valores estejam dentro da faixa calibrada do seu potenciômetro.
                      
% Tolerância para considerar que o motor atingiu o setpoint.
tolerance = 5; % Graus. Se o erro for menor que este valor, consideramos "atingido".

% Parâmetros da Coleta de Dados para Análise
durationPerSetpoint = 15; % Tempo em segundos para manter cada setpoint e coletar dados.
                           % Deve ser longo o suficiente para o sistema se estabilizar.
sampleRate = 0.02;         % Intervalo de amostragem em segundos (50 Hz).
numSamplesPerSetpoint = round(durationPerSetpoint / sampleRate);

% Arrays para armazenar os dados coletados
timeData = zeros(1, length(setPoints) * numSamplesPerSetpoint);
positionData = zeros(1, length(setPoints) * numSamplesPerSetpoint);
setpointData = zeros(1, length(setPoints) * numSamplesPerSetpoint);
% MUDEI O NOME DA VARIÁVEL PARA CLAREZA, E ELA ARMAZENARÁ O VALOR REAL APLICADO
actualPWMApplied = zeros(1, length(setPoints) * numSamplesPerSetpoint); 

currentSampleIndex = 1; % Índice global para preencher os arrays de dados.

% Loop através dos SetPoints
for spIdx = 1:length(setPoints)
    targetSetpoint = setPoints(spIdx);
    fprintf('\n--- Movendo para o SetPoint: %.2f Graus ---\n', targetSetpoint);

    tic; % Resetar o cronômetro
    
    for i = 1:numSamplesPerSetpoint
        currentTime = toc;
        if currentTime < (i-1) * sampleRate
            pause(((i-1) * sampleRate) - currentTime);
            currentTime = toc;
        end

        currentVoltage = readVoltage(a, potPin);
        currentAngle = (currentVoltage - minVoltage) / voltsPerDegree;

        % --- Cálculo do Erro e Ação de Controle P ---
        error = targetSetpoint - currentAngle;
        controlSignal = Kp * error; % Sinal bruto do controlador

        % --- Lógica de Saturação e Limites de Potência ---
        pwmValue = 0; % Inicializa o valor PWM a ser enviado
        appliedDirectionalPWM = 0; % Variável para armazenar o PWM com a direção (para plotagem)

        if controlSignal > 0 % O motor precisa girar em uma direção
            pwmValue = min(controlSignal, PWM_MAX);
            if pwmValue > 0 && pwmValue < PWM_MIN_ACTIVE
                pwmValue = PWM_MIN_ACTIVE;
            end
            
            writePWMDutyCycle(a, motorPin1, pwmValue);
            writePWMDutyCycle(a, motorPin2, 0);
            appliedDirectionalPWM = pwmValue; % Armazena o valor positivo
            
        elseif controlSignal < 0 % O motor precisa girar na direção oposta
            absControlSignal = abs(controlSignal);
            pwmValue = min(absControlSignal, PWM_MAX);
            if pwmValue > 0 && pwmValue < PWM_MIN_ACTIVE
                pwmValue = PWM_MIN_ACTIVE;
            end
            
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, pwmValue);
            appliedDirectionalPWM = -pwmValue; % Armazena o valor negativo para a direção oposta
            
        else % Erro é zero ou muito próximo de zero
            pwmValue = 0; % Desliga o motor
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, 0);
            appliedDirectionalPWM = 0;
        end
        
        % --- Armazenamento dos Dados ---
        timeData(currentSampleIndex) = currentTime + (spIdx - 1) * durationPerSetpoint;
        positionData(currentSampleIndex) = currentAngle;
        setpointData(currentSampleIndex) = targetSetpoint;
        % AGORA SALVA O VALOR REALMENTE APLICADO AO MOTOR
        actualPWMApplied(currentSampleIndex) = appliedDirectionalPWM; 

        currentSampleIndex = currentSampleIndex + 1;

        % Feedback no console (opcional)
        if mod(i, 50) == 0 || i == 1 || i == numSamplesPerSetpoint
            fprintf('  Tempo: %.2fs, SP: %.2f deg, Pos: %.2f deg, Erro: %.2f deg, PWM Aplicado: %.4f\n', ...
                    timeData(currentSampleIndex-1), targetSetpoint, currentAngle, error, appliedDirectionalPWM);
        end

        elapsedTime = toc - currentTime;
        if elapsedTime < sampleRate
            pause(sampleRate - elapsedTime);
        end
    end
    
    writePWMDutyCycle(a, motorPin1, 0);
    writePWMDutyCycle(a, motorPin2, 0);
    disp('Motor parado antes do próximo setpoint ou final.');
end

% ... (código de finalização e limpeza) ...

% 5. Visualização dos Dados Coletados
figure;
subplot(2,1,1);
plot(timeData, setpointData, 'k--', 'LineWidth', 1); hold on;
plot(timeData, positionData, 'r-', 'LineWidth', 1.5);
title('Resposta de Posição com Controle P Básico (Limites Aplicados)');
xlabel('Tempo (s)');
ylabel('Posição (Graus)');
legend('SetPoint', 'Posição Medida');
grid on;
hold off;

subplot(2,1,2);
% AGORA PLOTAMOS O VALOR REALMENTE APLICADO
plot(timeData, actualPWMApplied, 'b-', 'LineWidth', 1);
yline(PWM_MAX, 'r--', 'Max PWM');
yline(-PWM_MAX, 'r--', 'Min PWM');
yline(PWM_MIN_ACTIVE, 'g:', 'Min Active PWM');
yline(-PWM_MIN_ACTIVE, 'g:', 'Min Active PWM');
title('Sinal de Controle (PWM Realmente Aplicado)');
xlabel('Tempo (s)');
ylabel('Duty Cycle Aplicado (-1 a 1)');
grid on;

% 6. Salvar os Dados para Comparação Futura
try
    % ATUALIZANDO O NOME DA VARIÁVEL SALVA
    save('basicPControlResponse.mat', 'timeData', 'positionData', 'setpointData', 'actualPWMApplied', 'Kp', 'PWM_MAX', 'PWM_MIN_ACTIVE');
    disp('Dados de resposta do Controle P Básico salvos em basicPControlResponse.mat!');
catch ME
    disp(['ERRO ao salvar dados do Controle P Básico: ', ME.message]);
end

% ... (restante do código) ...