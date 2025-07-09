% collectPWMStepResponse.m
% Script para coletar a curva de resposta do motor a três degraus de PWM complexos.
% Os dados coletados (tempo, posição, duty cycle de entrada) serão usados
% no System Identification Toolbox para gerar um modelo do sistema.
clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- INICIANDO COLETA DE RESPOSTA AO DEGRAU DE PWM PARA MODELAGEM ---');
% 1. Carregar Definições de Pinos
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
    return;
end
% 2. Carregar Parâmetros de Calibração do Potenciômetro
try
    load('potentiometerCalibration.mat', 'minVoltage', 'maxVoltage', 'angleRangeDegrees', 'voltsPerDegree');
    if ~exist('minVoltage', 'var') || ~exist('voltsPerDegree', 'var') || ~exist('angleRangeDegrees', 'var')
        error('Parâmetros de calibração não carregados. Execute calibratePotentiometer.m primeiro.');
    end
    fprintf('Tensão Mínima (0 graus): %.4fV\n', minVoltage);
    fprintf('Volts por Grau: %.4fV/deg\n', voltsPerDegree);
    fprintf('Amplitude Angular Calibrada: %.2f graus\n', angleRangeDegrees);
catch ME
    disp(['ERRO ao carregar parâmetros de calibração: ', ME.message]);
    disp('Por favor, execute calibratePotentiometer.m antes de prosseguir.');
    return;
end
% Calcular os limites de 5% e o ponto médio para partida segura
lowerLimitAngle = angleRangeDegrees * 0.05;
upperLimitAngle = angleRangeDegrees * 0.95;
targetStartingAngle = angleRangeDegrees * 0.50; % Ponto médio do range
angleTolerance = 2; % Tolerância de 2 graus para o ponto de partida
fprintf('Limite Inferior: %.2f graus | Limite Superior: %.2f graus\n', lowerLimitAngle, upperLimitAngle);
fprintf('Ângulo alvo para partida (aprox. centro): %.2f graus\n', targetStartingAngle);
% 3. Recriar o Objeto Arduino
a = [];
try
    % Substitua 'COM9' pela porta serial da sua ESP32, se for diferente.
    a = arduino("COM9", "ESP32-WROOM-DevKitV1");
    disp('Conexão com ESP32 estabelecida para este script!');
    configurePin(a, motorPin1, "PWM");
    configurePin(a, motorPin2, "PWM");
    disp('Pinos do motor configurados como PWM.');
catch ME
    disp(['ERRO ao conectar à ESP32 ou configurar pinos: ', ME.message]);
    disp('Verifique se a placa está conectada e a porta COM correta.');
    return;
end
% --- Início do bloco try-catch-finally para execução principal e limpeza garantida ---
try
    % 4. PREPARAÇÃO: Mover motor para posição intermediária (anti-horário, partindo do limite superior)
    disp(' ');
    disp('--- PREPARANDO MOTOR: Movendo para posição inicial segura (anti-horário) ---');
    prepDutyCycle = 0.3; % PWM para mover o motor para a posição inicial
    prepTimeout = 15; % Timeout de segurança para a preparação em segundos
    % Desliga o motor para garantir que não haja movimento inicial inesperado
    writePWMDutyCycle(a, motorPin1, 0);
    writePWMDutyCycle(a, motorPin2, 0);
    pause(0.5); % Pequena pausa para garantir que o comando foi processado
    currentVoltage = readVoltage(a, potPin);
    currentAngle = (currentVoltage - minVoltage) / voltsPerDegree;
    fprintf('Posição inicial do motor: %.2f graus\n', currentAngle);
    if abs(currentAngle - targetStartingAngle) > angleTolerance
        disp(['Movendo motor para aprox. %.2f graus...', targetStartingAngle]);
        % Assume motorPin2 gira no sentido anti-horário (do limite superior para o centro)
        writePWMDutyCycle(a, motorPin2, prepDutyCycle);
        writePWMDutyCycle(a, motorPin1, 0);
        ticPrep = tic;
        while abs(currentAngle - targetStartingAngle) > angleTolerance
            currentVoltage = readVoltage(a, potPin);
            currentAngle = (currentVoltage - minVoltage) / voltsPerDegree;
            fprintf('Posição atual: %.2f graus\n', currentAngle);
            pause(0.05); % Pausa curta para não sobrecarregar a comunicação
            if toc(ticPrep) > prepTimeout
                disp('AVISO: Timeout na preparação do motor. Posição não atingida.');
                break;
            end
        end
        disp('Preparação concluída ou timeout atingido. Motor parado.');
        writePWMDutyCycle(a, motorPin1, 0); % Para o motor após a preparação
        writePWMDutyCycle(a, motorPin2, 0);
        pause(0.5); % Pequena pausa
    else
        disp('O motor já está em uma posição segura de partida (%.2f graus).', currentAngle);
    end
    disp(' ');
    disp('--- INICIANDO COLETA DE DADOS ---');
    disp('Pressione Ctrl+C no console do MATLAB para parar a qualquer momento.');
    % 5. Parâmetros da Coleta de Dados
    durationPerStep = 5; % Tempo em segundos para cada degrau de PWM.
    sampleRate = 0.02;    % Intervalo de amostragem em segundos (20 Hz).
    numSamplesPerStep = round(durationPerStep / sampleRate);
    % Arrays para armazenar os dados coletados: tempo, posição e duty cycle de entrada.
    % Agora para 3 degraus.
    timeData = zeros(1, 3 * numSamplesPerStep);
    positionData = zeros(1, 3 * numSamplesPerStep); 
    inputDutyCycle = zeros(1, 3 * numSamplesPerStep); % Armazena o duty cycle aplicado. 
    % Variáveis para offset de tempo e índice
    totalElapsed = 0;
    dataIndex = 1;
    % --- Definição dos Degraus de PWM ---
    % Cada linha é [dutyCycle_motorPin1, dutyCycle_motorPin2, dutyCycle_simulado_para_plot]
    pwmSteps = [
        0.4, 0, -0.4;  % Degrau 1: 40% PWM no motorPin2 (anti-horário, vindo do superior)
        0, 0.6, 0.6;   % Degrau 2: 60% PWM no motorPin1 (horário)
        0.3, 0, -0.3;  % Degrau 3: 30% PWM no motorPin2 (anti-horário, com PWM diferente)
        0, 0, 0        % NOVO DEGRAU: Desliga o motor ao final da sequência
    ];

    % Recalcule o tamanho dos arrays de dados para incluir o degrau de parada
    numTotalSteps = size(pwmSteps, 1);
    timeData = zeros(1, numTotalSteps * numSamplesPerStep);
    positionData = zeros(1, numTotalSteps * numSamplesPerStep);
    inputDutyCycle = zeros(1, numTotalSteps * numSamplesPerStep);
    
    for stepNum = 1:numTotalSteps
        currentPWM1 = pwmSteps(stepNum, 1);
        currentPWM2 = pwmSteps(stepNum, 2);
        currentSimulatedDutyCycle = pwmSteps(stepNum, 3);
        disp(['--- Aplicando Degrau ', num2str(stepNum), ': PWM1=', num2str(currentPWM1), ', PWM2=', num2str(currentPWM2), ' ---']);
        writePWMDutyCycle(a, motorPin1, currentPWM1);
        writePWMDutyCycle(a, motorPin2, currentPWM2);

        % Pausa extra no degrau de parada para garantir o comando
        if stepNum == numTotalSteps
            disp('Degrau Final: Garantindo o desligamento do motor...');
            pause(1); % Adiciona uma pausa de 1 segundo para o comando de desligamento ser processado
        end

        tic; % Inicia o cronômetro para este degrau
        for i = 1:numSamplesPerStep
            % Garante que a amostragem seja no tempo correto
            expectedTime = (i-1) * sampleRate;
            currentTimeFromStepStart = toc;
            if currentTimeFromStepStart < expectedTime
                pause(expectedTime - currentTimeFromStepStart);
            end
            
            % Lê dados e armazena
            currentVoltage = readVoltage(a, potPin);
            currentAngle = (currentVoltage - minVoltage) / voltsPerDegree;
            
            timeData(dataIndex) = totalElapsed + toc; % Tempo total acumulado
            positionData(dataIndex) = currentAngle;
            inputDutyCycle(dataIndex) = currentSimulatedDutyCycle;
            
            dataIndex = dataIndex + 1;
        end
        totalElapsed = timeData(dataIndex-1); % Atualiza o tempo total acumulado
    end
    disp(' ');
    disp('--- COLETA DE DADOS CONCLUÍDA ---');
catch ME % Bloco 'catch' para capturar erros ou interrupções
    if strcmp(ME.identifier, 'MATLAB:KeyboardInterrupt')
        % Tenta desligar o motor e fechar a conexão imediatamente
        if isvalid(a)
            disp('Tentando desligar o motor e liberar o objeto Arduino imediatamente...');
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, 0);
            delete(a); % Usa delete(a) em vez de clear a para forçar o destrutor
            disp('Tentativa de limpeza imediata concluída.');
        end
    else
        disp(' ');
        disp(['!!! ERRO INESPERADO DURANTE A OPERAÇÃO: ', ME.message, ' !!!']);
    end
finally % Bloco 'finally': SEMPRE executa, não importa o que aconteça no 'try' ou 'catch'
    % Garante que a limpeza aconteça, mesmo que já tenha sido tentada no catch
    if isvalid(a) % Verifica se o objeto 'a' (Arduino) ainda é válido
        disp(' ');
        disp('--- EXECUTANDO ROTINA DE LIMPEZA FINAL: Parando o motor e fechando conexão ---');
        writePWMDutyCycle(a, motorPin1, 0); % Desliga o motor.
        writePWMDutyCycle(a, motorPin2, 0); % Desliga o motor.
        delete(a); % Usa delete(a) para garantir que o objeto seja destruído
        disp('Conexão com ESP32 fechada.');
    elseif exist('a', 'var') && ~isvalid(a)
        % Mensagem adicional se o objeto já não for válido (ex: já foi deletado no catch)
        disp('Objeto Arduino já foi invalidado. Nenhuma ação de limpeza adicional necessária.');
    end
end
% --- Fim do bloco try-catch-finally ---
% 6. Visualização dos Dados Coletados
% Plota as curvas de entrada (duty cycle) e saída (posição) para inspeção visual.
figure;
subplot(2,1,1); % Cria uma figura com dois subplots, um acima do outro.
plot(timeData, inputDutyCycle, 'b-'); % Plota o duty cycle aplicado ao longo do tempo.
title('Sinal de Entrada (Duty Cycle Aplicado)');
xlabel('Tempo (s)');
ylabel('Duty Cycle (0-1 ou -1 a 1)');
grid on;
subplot(2,1,2);
plot(timeData, positionData, 'r-'); % Plota a posição angular lida do potenciômetro.
title('Resposta de Posição do Motor (Graus)');
xlabel('Tempo (s)');
ylabel('Posição (Graus)');
grid on;
% 7. Salvar os Dados para o System Identification Toolbox
% Salva as variáveis em um arquivo .mat para serem carregadas no Toolbox.
try
    save('multiStepResponseData.mat', 'timeData', 'positionData', 'inputDutyCycle');
    disp('Dados de resposta a múltiplos degraus salvos em multiStepResponseData.mat!');
catch ME
    disp(['ERRO ao salvar dados de resposta ao degrau: ', ME.message]);
end
disp(' ');
disp('Próximo passo: Abra o System Identification Toolbox (comando "ident" no MATLAB)');
disp('e importe os dados de multiStepResponseData.mat para criar um modelo do sistema.');