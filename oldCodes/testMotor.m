% moveMotorSimple.m
% Script para mover o motor em uma direção e velocidade específicas.

clc; clear all; close all; % Limpa console, workspace e fecha figuras
disp('--- CONTROLE SIMPLES DE DIREÇÃO E VELOCIDADE DO MOTOR ---');

% --- 1. Carregar Definições de Pinos do Motor ---
try
    load('arduinoPins.mat', 'motorPin1', 'motorPin2');
    if ~exist('motorPin1', 'var')
        error('Definições de pinos do motor não carregadas. Execute setupPins.m primeiro.');
    end
    fprintf('Pino do Motor 1 (motorPin1): %s\n', motorPin1);
    fprintf('Pteino do Motor 2 (motorPin2): %s\n', motorPin2);
catch ME
    disp(['ERRO ao carregar definições de pinos: ', ME.message]);
    disp('Por favor, execute setupPins.m antes de prosseguir.');
    return;
end


% --- RECRIAR O OBJETO ARDUINO NESTE SCRIPT ---
a = []; % Inicializa 'a' como vazio
try
    % Substitua 'COMx' pela porta serial da sua ESP32
    a = arduino("COM9", "ESP32-WROOM-DevKitV1"); % Adapte "COMx" e o nome da placa se necessário.
    disp('Conexão com ESP32 estabelecida para este script!');
    % Para calibração, não precisamos configurar PWM, apenas ler ADC.

catch ME
    disp(['ERRO ao reconectar à ESP32: ', ME.message]);
    disp('Verifique se a placa está conectada e a porta COM correta.');
    return; % Sai do script se não conseguir conectar
end
% --- Loop Principal de Controle ---
try
    while true
        disp(' '); % Linha em branco para melhor leitura
        directionInput = input('Digite a direção (H para Horário, A para Anti-horário, S para Parar e Sair): ', 's');
        directionInput = upper(directionInput); % Converte para maiúscula

        if directionInput == 'S'
            disp('Parando motor e encerrando script.');
            break; % Sai do loop
        end

        pwmValue = input('Digite a intensidade do PWM (0 a 1): ');
        if pwmValue < 0 || pwmValue > 1
            warning('Valor PWM inválido. Deve ser entre 0 e 1. Usando 0.5.');
            pwmValue = 0.5;
        end

        % Zera ambos os pinos PWM antes de aplicar o novo valor para evitar curto
        writePWMDutyCycle(a, motorPin1, 0);
        writePWMDutyCycle(a, motorPin2, 0);
        pause(0.1); % Pequena pausa para garantir a desativação

        if directionInput == 'H'
            writePWMDutyCycle(a, motorPin1, pwmValue);
            writePWMDutyCycle(a, motorPin2, 0);
            fprintf('Movendo no sentido HORÁRIO com PWM %.2f\n', pwmValue);
        elseif directionInput == 'A'
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, pwmValue);
            fprintf('Movendo no sentido ANTI-HORÁRIO com PWM %.2f\n', pwmValue);
        else
            disp('Entrada inválida. Digite H, A ou S.');
            writePWMDutyCycle(a, motorPin1, 0); % Garante que o motor pare em caso de entrada inválida
            writePWMDutyCycle(a, motorPin2, 0);
        end
    end

% --- Finalização (Executado ao interromper o loop ou digitar 'S') ---
catch ME % Captura a exceção se Ctrl+C for pressionado
    if (strcmp(ME.identifier, 'MATLAB:array:SizeLimitExceeded') || strcmp(ME.identifier, 'MATLAB:BREAK'))
        disp(' '); % Linha em branco
        disp('Script interrompido pelo usuário (Ctrl+C).');
    else
        disp(['Erro inesperado: ', ME.message]);
    end
end

% Parar o motor antes de fechar a conexão
if exist('a', 'var') && ~isempty(a) && isvalid(a)
    writePWMDutyCycle(a, motorPin1, 0);
    writePWMDutyCycle(a, motorPin2, 0);
    disp('Motor parado.');
    
    % Fechar a conexão com o Arduino
    clear a;
    disp('Conexão com ESP32 fechada.');
end

disp('--- FIM DO SCRIPT ---');