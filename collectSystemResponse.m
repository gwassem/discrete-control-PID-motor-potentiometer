% collectSystemResponse.m
% Script para coletar dados de resposta do sistema (motor e potenciômetro).
% Os dados serão usados para a System Identification Toolbox.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- COLETANDO DADOS DE RESPOSTA DO SISTEMA ---');

% Carrega as configurações da ESP32 e calibração do potenciômetro
try
    load('arduinoSetup.mat', 'a', 'potPin', 'motorPin1', 'motorPin2');
    load('potentiometerCalibration.mat', 'minVoltage', 'maxVoltage', 'angleRangeDegrees', 'voltsPerDegree');
    if ~exist('a', 'var') || ~isvalid(a)
        error('Objeto Arduino "a" não carregado ou inválido. Execute setupPins.m primeiro.');
    end
    disp('Configurações carregadas com sucesso!');
catch ME
    disp(['ERRO ao carregar configurações: ', ME.message]);
    disp('Por favor, execute setupPins.m e calibratePotentiometer.m antes de prosseguir.');
    return;
end

% Função anônima para converter tensão para graus
voltageToDegrees = @(voltage) (voltage - minVoltage) / voltsPerDegree;

% --- Parâmetros da Coleta ---
Ts = 0.05; % Período de amostragem (segundos)
numSamples = 1000; % Número total de amostras a coletar (ajuste conforme necessário)

% Arrays para armazenar os dados
inputPWM = zeros(numSamples, 1);       % Sinal de controle (PWM duty cycle)
outputPosition = zeros(numSamples, 1); % Posição lida (em graus)
timeVector = zeros(numSamples, 1);     % Vetor de tempo

disp('Iniciando a coleta... O motor irá girar.');
disp('Pressione Ctrl+C na Command Window para parar a qualquer momento.');
tic; % Inicia temporizador

% --- Geração do Sinal de Entrada (Rampa e Degrau) ---
% Vamos usar uma sequência que inclui uma rampa seguida de um degrau.
% Isso ajuda a excitar a dinâmica do sistema para a identificação.

pwmLevel1 = 0.3; % Primeiro nível de PWM (ex: 30%)
pwmLevel2 = 0.7; % Segundo nível de PWM (ex: 70%)
rampDuration = 200; % Número de amostras para a rampa (ex: 200 amostras * 0.05s = 10s de rampa)
stepDuration = 400; % Número de amostras para o degrau (ex: 400 amostras * 0.05s = 20s de degrau)

% Garante que o motor comece parado
writePWMDutyCycle(a, motorPin1, 0);
writePWMDutyCycle(a, motorPin2, 0);
pause(1); % Pequena pausa

for k = 1:numSamples
    currentPWM = 0; % Sinal PWM atual

    if k <= rampDuration
        % Rampa crescente
        currentPWM = (k / rampDuration) * pwmLevel1;
    elseif k <= rampDuration + stepDuration
        % Degrau para frente
        currentPWM = pwmLevel2;
    elseif k <= rampDuration + stepDuration + rampDuration
        % Rampa decrescente para zero (ou para outro lado, se quiser testar)
        currentPWM = pwmLevel2 - ((k - (rampDuration + stepDuration)) / rampDuration) * pwmLevel2;
    else
        % Degrau zero (parar)
        currentPWM = 0;
    end

    % Aplicar o sinal PWM em um sentido para a coleta
    writePWMDutyCycle(a, motorPin1, currentPWM);
    writePWMDutyCycle(a, motorPin2, 0); % O outro pino fica em zero para este sentido

    % Ler a posição do potenciômetro
    currentVoltage = readVoltage(a, potPin);
    currentPosition = voltageToDegrees(currentVoltage);

    % Armazenar dados
    inputPWM(k) = currentPWM;
    outputPosition(k) = currentPosition;
    timeVector(k) = toc;

    % Exibir progresso
    if mod(k, 50) == 0
        fprintf('Amostra %d/%d: PWM = %.2f, Posição = %.2f graus\n', k, numSamples, currentPWM, currentPosition);
    end

    % Pausar para manter o período de amostragem
    elapsedTime = toc - timeVector(k);
    if elapsedTime < Ts
        pause(Ts - elapsedTime);
    end
end

% Parar o motor no final da coleta
writePWMDutyCycle(a, motorPin1, 0);
writePWMDutyCycle(a, motorPin2, 0);

disp('Coleta de dados finalizada. Plotando e salvando...');

% Plotar os dados coletados para visualização
figure;
subplot(2,1,1);
plot(timeVector, inputPWM);
title('Sinal de Entrada (PWM)');
xlabel('Tempo (s)');
ylabel('PWM Duty Cycle (0-1)');
grid on;

subplot(2,1,2);
plot(timeVector, outputPosition);
title('Posição do Motor (Resposta do Sistema)');
xlabel('Tempo (s)');
ylabel('Posição (graus)');
grid on;

% Salvar os dados para a System Identification Toolbox
% É recomendado salvar os dados em um objeto iddata
data = iddata(outputPosition, inputPWM, Ts);
save('systemResponseData.mat', 'data', 'timeVector', 'inputPWM', 'outputPosition');
disp('Dados de resposta do sistema salvos em systemResponseData.mat.');

disp('--- COLETANDO DADOS DE RESPOSTA CONCLUÍDA ---');
disp('Agora, você pode usar a System Identification Toolbox do MATLAB:');
disp('1. Digite "ident" na Command Window do MATLAB.');
disp('2. Na janela da Toolbox, vá em "Import data" -> "Time domain data".');
disp('3. Selecione "From workspace" e importe a variável "data".');
disp('4. Use as ferramentas da Toolbox para estimar um modelo do seu sistema.');