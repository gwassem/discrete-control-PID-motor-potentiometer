% collectSystemResponse.m
% Script para coletar dados de resposta do sistema (motor e potenciômetro).
% Os dados serão usados para a System Identification Toolbox.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- COLETANDO DADOS DE RESPOSTA DO SISTEMA ---');

% Carrega as definições de pinos (strings) e os parâmetros de calibração do potenciômetro
try
    load('arduinoPins.mat', 'potPin', 'motorPin1', 'motorPin2');
    load('potentiometerCalibration.mat', 'minVoltage', 'maxVoltage', 'angleRangeDegrees', 'voltsPerDegree');
    if ~exist('potPin', 'var') || ~exist('minVoltage', 'var')
        error('Dependências não carregadas. Execute setupPins.m e calibratePotentiometer.m primeiro.');
    end
    disp('Configurações e calibração carregadas com sucesso!');
catch ME
    disp(['ERRO ao carregar configurações ou calibração: ', ME.message]);
    disp('Por favor, execute setupPins.m e calibratePotentiometer.m antes de prosseguir.');
    return;
end

% --- RECRIAR O OBJETO ARDUINO NESTE SCRIPT ---
a = []; % Inicializa 'a' como vazio
try
    % Substitua 'COMx' pela porta serial da sua ESP32
    a = arduino("COMx", "ESP32 Dev Module");
    disp('Conexão com ESP32 estabelecida para este script!');

    % Configurar os pinos do motor como PWM (necessário aqui para controlar o motor)
    configurePin(a, motorPin1, "PWM");
    configurePin(a, motorPin2, "PWM");
    disp('Pinos do motor configurados como PWM.');

catch ME
    disp(['ERRO ao reconectar ou configurar a ESP32: ', ME.message]);
    disp('Verifique se a placa está conectada e a porta COM correta.');
    return; % Sai do script se não conseguir conectar
end

% Função anônima para converter tensão para graus
voltageToDegrees = @(voltage) (voltage - minVoltage) / voltsPerDegree;

% --- Parâmetros da Coleta ---
Ts = 0.05; % Período de amostragem (segundos). Ajuste conforme a dinâmica do seu motor!
numSamples = 1000; % Número total de amostras a coletar (ajuste conforme necessário)
                   % Ex: 1000 amostras * 0.05s/amostra = 50 segundos de dados.

% Arrays para armazenar os dados
inputPWM = zeros(numSamples, 1);       % Sinal de controle (PWM duty cycle)
outputPosition = zeros(numSamples, 1); % Posição lida (em graus)
timeVector = zeros(numSamples, 1);     % Vetor de tempo

disp('Iniciando a coleta de dados de resposta do sistema... O motor irá girar.');
disp('Pressione Ctrl+C na Command Window para parar a qualquer momento.');
tic; % Inicia temporizador

% --- Geração do Sinal de Entrada (Exemplo: Rampa, Degrau para frente, Degrau para trás) ---
% Este sinal é projetado para "excitar" a dinâmica do motor em diferentes regimes.
% Você pode ajustar os níveis de PWM e durações.
pwmLevelStart = 0.2; % Nível inicial de PWM (evitar 0 para sistemas com atrito)
pwmLevelMid = 0.5;   % Nível intermediário
pwmLevelEnd = 0.8;   % Nível mais alto

segment1Duration = 200; % PWM constante no nívelStart
segment2Duration = 300; % Rampa de pwmLevelStart para pwmLevelMid
segment3Duration = 300; % PWM constante no nívelMid
segment4Duration = 200; % Rampa de pwmLevelMid para pwmLevelEnd
segment5Duration = 300; % PWM constante no nívelEnd
segment6Duration = 200; % Rampa para 0
totalDurationInSamples = segment1Duration + segment2Duration + segment3Duration + segment4Duration + segment5Duration + segment6Duration;

if numSamples < totalDurationInSamples
    warning('numSamples é menor que a duração total dos segmentos. Ajustando numSamples.');
    numSamples = totalDurationInSamples;
end

% Garante que o motor comece parado
writePWMDutyCycle(a, motorPin1, 0);
writePWMDutyCycle(a, motorPin2, 0);
pause(1); % Pequena pausa para garantir que o motor esteja parado e estabilizado

for k = 1:numSamples
    currentPWM = 0; % Sinal PWM atual

    if k <= segment1Duration
        currentPWM = pwmLevelStart; % Degrau inicial
    elseif k <= segment1Duration + segment2Duration
        % Rampa crescente
        rampProgress = (k - segment1Duration) / segment2Duration;
        currentPWM = pwmLevelStart + rampProgress * (pwmLevelMid - pwmLevelStart);
    elseif k <= segment1Duration + segment2Duration + segment3Duration
        currentPWM = pwmLevelMid; % Degrau intermediário
    elseif k <= segment1Duration + segment2Duration + segment3Duration + segment4Duration
        % Rampa crescente
        rampProgress = (k - (segment1Duration + segment2Duration + segment3Duration)) / segment4Duration;
        currentPWM = pwmLevelMid + rampProgress * (pwmLevelEnd - pwmLevelMid);
    elseif k <= segment1Duration + segment2Duration + segment3Duration + segment4Duration + segment5Duration
        currentPWM = pwmLevelEnd; % Degrau alto
    elseif k <= segment1Duration + segment2Duration + segment3Duration + segment4Duration + segment5Duration + segment6Duration
        % Rampa decrescente de volta a zero
        rampProgress = (k - (segment1Duration + segment2Duration + segment3Duration + segment4Duration + segment5Duration)) / segment6Duration;
        currentPWM = pwmLevelEnd * (1 - rampProgress);
    else
        currentPWM = 0; % Sinal zero após os segmentos definidos
    end

    % Garantir que o PWM esteja dentro dos limites [0, 1]
    currentPWM = max(0, min(currentPWM, 1));

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

    % Exibir progresso (opcional)
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
disp('4. Use as ferramentas da Toolbox para estimar um modelo do seu sistema (ex: função de transferência).');
disp('   Considere um modelo de 1ª ou 2ª ordem com possível atraso de tempo (dead time).');

% --- NO FINAL DO SCRIPT, SEMPRE LIMPE O OBJETO 'a' ---
clear a;
disp('Conexão com a ESP32 fechada para este script.');