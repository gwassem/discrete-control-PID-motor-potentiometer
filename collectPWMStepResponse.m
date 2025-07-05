% collectPWMStepResponse.m
% Script para coletar a curva de resposta do motor a um degrau de PWM.
% Os dados coletados (tempo, posição, duty cycle de entrada) serão usados
% no System Identification Toolbox para gerar um modelo do sistema.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- INICIANDO COLETA DE RESPOSTA AO DEGRAU DE PWM ---');

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

% 4. Parâmetros da Coleta de Dados
% Definição do tempo total de coleta para cada degrau e a taxa de amostragem.
durationPerStep = 10; % Tempo em segundos para cada degrau de PWM ser aplicado.
                      % Este tempo deve ser suficiente para o motor atingir o regime permanente.
sampleRate = 0.02;    % Intervalo de amostragem em segundos (equivalente a 20 Hz).
                      % Um valor menor significa mais amostras e potencialmente mais precisão.
numSamplesPerStep = round(durationPerStep / sampleRate); % Número de amostras por degrau.

% Arrays para armazenar os dados coletados: tempo, posição e duty cycle de entrada.
% Vamos coletar dados para dois degraus.
timeData = zeros(1, 2 * numSamplesPerStep);
positionData = zeros(1, 2 * numSamplesPerStep);
inputDutyCycle = zeros(1, 2 * numSamplesPerStep); % Armazena o duty cycle aplicado.

% 5. Coleta de Dados - Primeiro Degrau de PWM
disp(' ');
disp('--- Aplicando Primeiro Degrau de PWM ---');
% Define o duty cycle para o primeiro degrau. Ajuste este valor conforme necessário.
% Um duty cycle positivo pode girar em uma direção.
dutyCycle1 = 0.4; % Exemplo: 40% de duty cycle. Valor entre 0 e 1.

fprintf('Aplicando %s com Duty Cycle de %.2f por %d segundos.\n', motorPin1, dutyCycle1, durationPerStep);
writePWMDutyCycle(a, motorPin1, dutyCycle1); % Aplica o PWM no motorPin1.
writePWMDutyCycle(a, motorPin2, 0);          % Garante que o outro pino está em 0 para definir a direção.

% Inicia o cronômetro para medir o tempo decorrido.
tic;
for i = 1:numSamplesPerStep
    currentTime = toc; % Tempo atual desde o início do cronômetro.

    % Garante que a amostra seja coletada no tempo correto, pausando se necessário.
    if currentTime < (i-1) * sampleRate
        pause(((i-1) * sampleRate) - currentTime);
        currentTime = toc; % Atualiza o tempo após a pausa.
    end

    currentVoltage = readVoltage(a, potPin); % Lê a tensão do potenciômetro.
    % Converte a tensão lida para graus usando os parâmetros de calibração.
    currentAngle = (currentVoltage - minVoltage) / voltsPerDegree;

    % Armazena os dados no array correspondente.
    timeData(i) = currentTime;
    positionData(i) = currentAngle;
    inputDutyCycle(i) = dutyCycle1; % A entrada é o duty cycle aplicado.

    % Se o processamento demorou mais que o sampleRate, não precisa pausar.
    % Caso contrário, pausa para manter o sampleRate.
    elapsedTime = toc - currentTime;
    if elapsedTime < sampleRate
        pause(sampleRate - elapsedTime);
    end
end

% 6. Coleta de Dados - Segundo Degrau de PWM (Opcional, para excitar em outra direção ou nível)
disp(' ');
disp('--- Aplicando Segundo Degrau de PWM (Inverso) ---');
% Define o duty cycle para o segundo degrau. Aqui, um valor negativo simbólico
% para representar a direção oposta, embora o PWM seja sempre positivo na saída.
dutyCycle2 = 0.4; % Mesmo valor, mas agora para a outra direção.

fprintf('Aplicando %s com Duty Cycle de %.2f por %d segundos.\n', motorPin2, dutyCycle2, durationPerStep);
writePWMDutyCycle(a, motorPin1, 0);          % Garante que o primeiro pino está em 0.
writePWMDutyCycle(a, motorPin2, dutyCycle2); % Aplica o PWM no motorPin2 (direção oposta).

% O offset é usado para continuar armazenando os dados nos arrays após o primeiro degrau.
offset = numSamplesPerStep;
tic; % Reinicia o cronômetro para medir o tempo do segundo degrau.
for i = 1:numSamplesPerStep
    currentTime = toc; % Tempo atual desde o início deste degrau.

    % Garante o sampleRate.
    if currentTime < (i-1) * sampleRate
        pause(((i-1) * sampleRate) - currentTime);
        currentTime = toc;
    end

    currentVoltage = readVoltage(a, potPin);
    currentAngle = (currentVoltage - minVoltage) / voltsPerDegree;

    % Armazena os dados. O tempo é acumulado para ter uma linha do tempo contínua.
    timeData(offset + i) = currentTime + timeData(offset);
    positionData(offset + i) = currentAngle;
    inputDutyCycle(offset + i) = -dutyCycle2; % Usa um valor negativo para indicar a direção inversa na plotagem.

    elapsedTime = toc - currentTime;
    if elapsedTime < sampleRate
        pause(sampleRate - elapsedTime);
    end
end

% 7. Finalização e Limpeza
disp(' ');
disp('Parando o motor...');
writePWMDutyCycle(a, motorPin1, 0); % Desliga o motor.
writePWMDutyCycle(a, motorPin2, 0); % Desliga o motor.
clear a; % Fecha a conexão com a ESP32.
disp('Conexão com ESP32 fechada.');
disp('--- COLETA DE DADOS CONCLUÍDA ---');

% 8. Visualização dos Dados Coletados
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

% 9. Salvar os Dados para o System Identification Toolbox
% Salva as variáveis cruciais em um arquivo .mat para serem carregadas no Toolbox.
try
    save('stepResponseData.mat', 'timeData', 'positionData', 'inputDutyCycle');
    disp('Dados de resposta ao degrau salvos em stepResponseData.mat!');
catch ME
    disp(['ERRO ao salvar dados de resposta ao degrau: ', ME.message]);
end

disp(' ');
disp('Próximo passo: Abra o System Identification Toolbox (comando "ident" no MATLAB)');
disp('e importe os dados de stepResponseData.mat para criar um modelo do sistema.');