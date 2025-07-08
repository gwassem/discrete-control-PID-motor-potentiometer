% rotateMotorToLimit.m
% Script para girar o motor em uma direção até um limite ou interrupção manual,
% com opção de escolha da direção de rotação.
clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- INICIANDO ROTAÇÃO DO MOTOR ATÉ O LIMITE ---');

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
catch ME1
    disp(['ERRO ao carregar parâmetros de calibração: ', ME.message]);
    disp('Por favor, execute calibratePotentiometer.m antes de prosseguir.');
    return;
end

% Calcular os limites de 5%
% Assumimos que 0 graus é o minVoltage e angleRangeDegrees é o maxVoltage do range.
lowerLimitAngle = angleRangeDegrees * 0.05; % 5% do range total
upperLimitAngle = angleRangeDegrees * 0.95; % 95% do range total

fprintf('Limite Inferior para Parada: %.2f graus\n', lowerLimitAngle);
fprintf('Limite Superior para Parada: %.2f graus\n', upperLimitAngle);

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

% --- MODIFICAÇÃO: Verificação Inicial da Posição do Motor (apenas alerta) ---
disp(' ');
disp('--- VERIFICANDO POSIÇÃO ATUAL DO MOTOR ---');
try
    currentVoltage = readVoltage(a, potPin);
    currentAngle = (currentVoltage - minVoltage) / voltsPerDegree;
    fprintf('Posição atual detectada: Tensão %.4fV | Ângulo %.2f graus\n', currentVoltage, currentAngle);

    if currentAngle <= lowerLimitAngle
        disp('*** AVISO: O motor já está muito próximo ou abaixo do limite inferior (%.2f graus). ***', lowerLimitAngle);
        disp('*** Prossiga com cautela. ***');
    elseif currentAngle >= upperLimitAngle
        disp('*** AVISO: O motor já está muito próximo ou acima do limite superior (%.2f graus). ***', upperLimitAngle);
        disp('*** Prossiga com cautela. ***');
    else
        disp('Posição de partida segura detectada.');
    end
catch ME
    disp(['ERRO ao ler a posição inicial do potenciômetro: ', ME.message]);
    disp('Não foi possível verificar a posição inicial do motor. Prossiga com cautela.');
    % Não termina o script, apenas avisa sobre a falha na leitura inicial.
end
% --- FIM DA MODIFICAÇÃO ---

% 4. Parâmetros de Rotação e Seleção de Direção
dutyCycle = 0.5; % Duty cycle para rotação (50%). Ajuste conforme a necessidade.
sampleRate = 0.1; % Intervalo de amostragem em segundos.

disp(' ');
disp('--- Selecione a Direção de Rotação ---');
% Pergunta agora se quer girar no sentido horário ou anti-horário
promptString = sprintf('Digite 1 para girar no SENTIDO HORÁRIO ou 2 para girar no SENTIDO ANTI-HORÁRIO: ');
directionChoice = input(promptString);

selectedMotorPin = '';
otherMotorPin = '';

% Ajuste a lógica aqui para mapear 1 (horário) e 2 (anti-horário)
% para os seus pinos físicos.
% EXEMPLO: ASSUMINDO que motorPin1 -> Sentido Horário e motorPin2 -> Sentido Anti-Horário
% SE ESSA ASSUNÇÃO ESTIVER ERRADA PARA O SEU HARDWARE, VOCÊ DEVE INVERTER
% 'motorPin1' e 'motorPin2' DENTRO DOS BLOCOS 'if' E 'elseif' ABAIXO.
if directionChoice == 1 % Usuário escolhe sentido horário
    selectedMotorPin = motorPin1; % Define o pino que corresponde ao sentido horário
    otherMotorPin = motorPin2;
    disp('Motor girará no sentido horário.');
elseif directionChoice == 2 % Usuário escolhe sentido anti-horário
    selectedMotorPin = motorPin2; % Define o pino que corresponde ao sentido anti-horário
    otherMotorPin = motorPin1;
    disp('Motor girará no sentido anti-horário.');
else
    disp('Opção inválida. Nenhuma rotação será iniciada.');
    % Garante que a conexão com o Arduino seja fechada se não prosseguir
    clear a;
    return;
end

disp(' ');
disp('--- Iniciando Rotação do Motor ---');
disp('Pressione Ctrl+C no console do MATLAB para parar a qualquer momento.');

% Aplica o PWM para girar na direção escolhida
writePWMDutyCycle(a, selectedMotorPin, dutyCycle);
writePWMDutyCycle(a, otherMotorPin, 0); % Garante que o outro pino está em 0

% Loop principal de rotação e leitura
try
    while true
        currentVoltage = readVoltage(a, potPin);
        currentAngle = (currentVoltage - minVoltage) / voltsPerDegree;

        fprintf('Tensão: %.4fV | Ângulo: %.2f graus\n', currentVoltage, currentAngle);

        % Verifica se atingiu um dos limites de rotação
        if currentAngle <= lowerLimitAngle || currentAngle >= upperLimitAngle
            disp('Limite de rotação atingido! Parando o motor.');
            break; % Sai do loop
        end

        pause(sampleRate); % Aguarda o tempo de amostragem
    end
catch ME
    % Captura interrupção (Ctrl+C) ou outros erros
    if strcmp(ME.identifier, 'MATLAB:KeyboardInterrupt')
        disp('Rotação interrompida pelo usuário (Ctrl+C).');
    else
        disp(['Erro inesperado durante a rotação: ', ME.message]);
    end
end

% 5. Finalização e Limpeza
disp(' ');
disp('Parando o motor...');
writePWMDutyCycle(a, motorPin1, 0); % Desliga o motor.
writePWMDutyCycle(a, motorPin2, 0); % Desliga o motor.
clear a; % Fecha a conexão com a ESP32.
disp('Conexão com ESP32 fechada.');
disp('--- ROTAÇÃO CONCLUÍDA ---');