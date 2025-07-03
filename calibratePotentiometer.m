% calibratePotentiometer.m
% Script para calibrar o potenciômetro (tensão para graus).

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- CALIBRAÇÃO DO POTENCIÔMETRO ---');

% Carrega APENAS as definições de pinos (strings)
try
    load('arduinoPins.mat', 'potPin'); % Apenas 'potPin' necessário aqui
    if ~exist('potPin', 'var')
        error('Definições de pinos não carregadas. Execute setupPins.m primeiro.');
    end
    disp('Definições de pinos carregadas com sucesso!');
catch ME
    disp(['ERRO ao carregar definições de pinos: ', ME.message]);
    disp('Por favor, execute setupPins.m antes de prosseguir.');
    return;
end

% --- RECRIAR O OBJETO ARDUINO NESTE SCRIPT ---
a = []; % Inicializa 'a' como vazio
try
    % Substitua 'COMx' pela porta serial da sua ESP32
    a = arduino("COMx", "ESP32 Dev Module"); % Adapte "COMx" e o nome da placa se necessário.
    disp('Conexão com ESP32 estabelecida para este script!');
    % Para calibração, não precisamos configurar PWM, apenas ler ADC.

catch ME
    disp(['ERRO ao reconectar à ESP32: ', ME.message]);
    disp('Verifique se a placa está conectada e a porta COM correta.');
    return; % Sai do script se não conseguir conectar
end

% Interação para encontrar os limites de tensão
disp('** Siga as instruções para girar o motor manualmente **');
disp('Gire o motor (e o potenciômetro) para a **posição angular MÍNIMA** que ele pode atingir.');
disp('Pressione Enter para registrar a leitura.');
pause; % Espera o usuário pressionar Enter
minVoltage = readVoltage(a, potPin);
disp(['Tensão na posição MÍNIMA: ', num2str(minVoltage), ' V']);

disp('Gire o motor (e o potenciômetro) para a **posição angular MÁXIMA** que ele pode atingir.');
disp('Certifique-se de que o motor está em seu limite físico de rotação (se houver) ou no limite da faixa que você deseja controlar.');
disp('Pressione Enter para registrar a leitura.');
pause; % Espera o usuário pressionar Enter
maxVoltage = readVoltage(a, potPin);
disp(['Tensão na posição MÁXIMA: ', num2str(maxVoltage), ' V']);

% --- DEFINIR A FAIXA ANGULAR REAL DO SEU SISTEMA ---
% Este é o valor mais importante a ser ajustado manualmente com base na sua montagem.
% Se o seu motor, com o acoplamento do potenciômetro de 10 voltas,
% pode girar, por exemplo, 5 voltas completas (5 * 360 = 1800 graus), use 1800.
% Se ele tem um limite físico que é equivalente a 7.5 voltas do pot,
% então seria 7.5 * 360 = 2700 graus.
angleRangeDegrees = 1800; % <--- AJUSTE ESTE VALOR para a faixa REAL em graus do seu motor/pot!

% Calcule a escala para converter Volts em Graus
voltsPerDegree = (maxVoltage - minVoltage) / angleRangeDegrees;

% Validação simples para evitar divisão por zero ou dados inconsistentes
if abs(maxVoltage - minVoltage) < 1e-3 % Se a variação de tensão for insignificante
    error('Variação de tensão muito pequena durante a calibração. Verifique o acoplamento do potenciômetro e a fiação.');
end
if abs(voltsPerDegree) < 1e-6
    error('Erro no cálculo voltsPerDegree. Pode haver problema com a faixa de tensão ou ângulo.');
end


disp(['Faixa angular total calibrada: ', num2str(angleRangeDegrees), ' graus']);
disp(['Volts por Grau: ', num2str(voltsPerDegree), ' V/grau']);

% Salva os parâmetros de calibração em um arquivo .mat
save('potentiometerCalibration.mat', 'minVoltage', 'maxVoltage', 'angleRangeDegrees', 'voltsPerDegree');
disp('Parâmetros de calibração salvos em potentiometerCalibration.mat.');

disp('--- CALIBRAÇÃO CONCLUÍDA ---');

% --- NO FINAL DO SCRIPT, SEMPRE LIMPE O OBJETO 'a' ---
clear a;
disp('Conexão com a ESP32 fechada para este script.');