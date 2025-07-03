% calibratePotentiometer.m
% Script para calibrar o potenciômetro, mapeando tensão em graus.

clc; clear all; close all;
disp('--- CALIBRAÇÃO DO POTENCIÔMETRO ---');

% 1. Carregar Definições de Pinos
try
    load('arduinoPins.mat', 'potPin');
    if ~exist('potPin', 'var')
        error('Pino do potenciômetro (potPin) não carregado. Execute setupPins.m primeiro.');
    end
    fprintf('Pino do Potenciômetro (potPin): %s\n', potPin);
catch ME
    disp(['ERRO ao carregar definições de pinos: ', ME.message]);
    disp('Por favor, execute setupPins.m antes de prosseguir.');
    return;
end

% --- RECRIAR O OBJETO ARDUINO NESTE SCRIPT ---
a = []; % Inicializa 'a' como vazio
try
    % Substitua 'COMx' pela porta serial da sua ESP32
    % Certifique-se que o nome do módulo ESP32 está correto!
    a = arduino("COM9", "ESP32-WROOM-DevKitV1"); 
    disp('Conexão com ESP32 estabelecida para este script!');
    if isvalid(a)
        disp('  Objeto Arduino criado e válido.');
    else
        disp('  AVISO: Objeto Arduino não é válido após a criação.');
        clear a; return;
    end
catch ME
    disp(['ERRO ao conectar à ESP32: ', ME.message]);
    disp('Verifique se a placa está conectada e a porta COM correta.');
    return;
end

% 2. Calibrar o Ponto Mínimo (0 graus)
disp(' ');
disp('Passo 1: Gire o potenciômetro/motor para a posição de ANGULO MÍNIMO (ex: 0 graus).');
input('Pressione Enter quando estiver pronto para registrar a tensão mínima...');
minVoltage = readVoltage(a, potPin);
fprintf('Tensão mínima registrada: %.4fV\n', minVoltage);

% 3. Calibrar o Ponto Máximo (3600 graus para 10 voltas)
disp(' ');
disp('Passo 2: Gire o potenciômetro/motor para a posição de ANGULO MÁXIMO (ex: 3600 graus).');
input('Pressione Enter quando estiver pronto para registrar a tensão máxima...');
maxVoltage = readVoltage(a, potPin);
fprintf('Tensão máxima registrada: %.4fV\n', maxVoltage);

% 4. Definir o range angular total que o motor percorre
% Para um potenciômetro de 10 voltas, 3600.00 é o correto.
angleRangeDegrees = 3600.00; 

% 5. Calcular a relação Volts por Grau
voltsPerDegree = (maxVoltage - minVoltage) / angleRangeDegrees;
fprintf('Calculado: voltsPerDegree = %.4fV/deg\n', voltsPerDegree);

% 6. Salvar parâmetros de calibração
try
    % AGORA SALVA NO ARQUIVO CORRETO: potentiometerCalibration.mat
    save('potentiometerCalibration.mat', 'minVoltage', 'maxVoltage', 'angleRangeDegrees', 'voltsPerDegree');
    disp('Parâmetros de calibração salvos em potentiometerCalibration.mat!');
catch ME
    disp(['ERRO ao salvar parâmetros de calibração: ', ME.message]);
end

% 7. Testar a conversão (opcional)
disp(' ');
testVoltage = input('Digite uma tensão para testar a conversão em graus (ex: 1.5): ');
if ~isempty(testVoltage) && isnumeric(testVoltage)
    testDegrees = (testVoltage - minVoltage) / voltsPerDegree;
    fprintf('%.4fV corresponde a %.2f graus.\n', testVoltage, testDegrees);
end

% 8. Fechar conexão Arduino
clear a;
disp('Conexão com ESP32 fechada.');