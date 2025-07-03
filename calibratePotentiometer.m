% calibratePotentiometer.m
% Script para calibrar o potenciômetro (tensão para graus).

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- CALIBRAÇÃO DO POTENCIÔMETRO ---');

% Carrega as configurações da ESP32 do script setupPins.m
try
    load('arduinoSetup.mat', 'a', 'potPin');
    if ~exist('a', 'var') || ~isvalid(a)
        error('Objeto Arduino "a" não carregado ou inválido. Execute setupPins.m primeiro.');
    end
    disp('Configuração da ESP32 carregada com sucesso!');
catch ME
    disp(['ERRO ao carregar configuração: ', ME.message]);
    disp('Por favor, execute setupPins.m antes de prosseguir.');
    return;
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

angleRangeDegrees = 3600; % <--- AJUSTE ESTE VALOR para a faixa REAL em graus do seu motor/pot!

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

% Função anônima para converter tensão para graus (para referência)
voltageToDegrees = @(voltage) (voltage - minVoltage) / voltsPerDegree;

disp('--- CALIBRAÇÃO CONCLUÍDA ---');