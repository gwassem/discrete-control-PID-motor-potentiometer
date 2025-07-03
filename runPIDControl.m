% runPIDControl.m
% Script para executar o controle PID de posição do motor continuamente.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- INICIANDO CONTROLE PID DE POSIÇÃO ---');

% Carrega as definições de pinos (strings) e calibração do potenciômetro
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
    a = arduino("COM9", "ESP32-WROOM-DevKitV1");
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

% --- Parâmetros do Controlador PID ---
% ESTES SÃO OS VALORES CRÍTICOS QUE VOCÊ AJUSTARÁ COM O PID TUNER.
% Use os valores obtidos após a identificação do sistema e ajuste no PID Tuner.
Kp = 0.5;   % Ganho Proporcional (AJUSTE AQUI!)
Ki = 0.01;  % Ganho Integral (AJUSTE AQUI!)
Kd = 0.05;  % Ganho Derivativo (AJUSTE AQUI!)

% Parâmetros do loop de controle
Ts = 0.05; % Período de amostragem (segundos). Deve ser o mesmo usado na coleta de dados!

% Variáveis do PID
integrativeError = 0;
lastError = 0;

% Obter a posição inicial para o cálculo do primeiro erro derivativo
initialVoltage = readVoltage(a, potPin);
initialPositionDegrees = voltageToDegrees(initialVoltage);

% Armazenamento de dados para análise posterior (opcional)
% Para um loop contínuo, prealocar pode não ser tão eficiente
% Por enquanto, vamos manter uma pré-alocação generosa para análise.
maxDataPoints = 6000; % 6000 amostras * 0.05s/amostra = 300 segundos (5 minutos)
positionData = zeros(maxDataPoints, 1);
setpointData = zeros(maxDataPoints, 1);
controlSignalData = zeros(maxDataPoints, 1);
timeData = zeros(maxDataPoints, 1);
k = 0; % Contador de iterações

disp('--- INICIANDO CONTROLE PID ---');

% --- SOLICITAÇÃO DO ÂNGULO DESEJADO AO USUÁRIO ---
while true
    prompt = sprintf('Digite o ângulo desejado em graus (0 a %.0f), "r" para novo setpoint, ou "q" para sair: ', angleRangeDegrees);
    userInput = input(prompt, 's');

    if strcmpi(userInput, 'q')
        disp('Comando para sair recebido.');
        % Parar o motor antes de sair
        writePWMDutyCycle(a, motorPin1, 0);
        writePWMDutyCycle(a, motorPin2, 0);
        break; % Sai do loop de entrada de setpoint
    elseif strcmpi(userInput, 'r')
        % Continua no loop para pedir um novo setpoint
        disp('Aguardando novo setpoint...');
        continue;
    end

    newTargetPositionDegrees = str2double(userInput);

    if ~isnan(newTargetPositionDegrees) && newTargetPositionDegrees >= 0 && newTargetPositionDegrees <= angleRangeDegrees
        targetPositionDegrees = newTargetPositionDegrees; % Atualiza o setpoint
        fprintf('Ângulo desejado definido para: %.2f graus. O controle continuará.\n', targetPositionDegrees);
        % Resetar o erro integral ao mudar o setpoint ajuda a evitar overshoot
        integrativeError = 0;
        lastError = 0;
        break; % Sai do loop de entrada de setpoint para começar/continuar o controle
    else
        disp('Entrada inválida. Por favor, digite um número entre 0 e o valor máximo da faixa calibrada, "r", ou "q".');
    end
end

% Se o usuário digitou 'q' na primeira vez, o script já saiu.
% Se chegou aqui, um setpoint foi definido e o controle pode começar.

% Loop de Controle Infinito
disp('Iniciando o loop de controle contínuo... Pressione Ctrl+C na Command Window para parar.');
tic; % Inicia temporizador para o loop
while true % Loop infinito
    k = k + 1; % Incrementa o contador de iterações

    % 1. Medir a Posição Atual
    currentVoltage = readVoltage(a, potPin);
    currentPositionDegrees = voltageToDegrees(currentVoltage);

    % 2. Calcular o Erro
    error = targetPositionDegrees - currentPositionDegrees;

    % 3. Calcular Termos PID
    proportionalTerm = Kp * error;

    integrativeError = integrativeError + error * Ts; % Soma do erro * Ts (aproximação da integral discreta)
    integralTerm = Ki * integrativeError;
    
    % Anti-windup (opcional, mas recomendado para o termo integral)
    % Limita o erro integral para evitar que ele cresça indefinidamente quando o motor satura
    integralTermMax = 0.5; % Ajuste conforme a magnitude do seu sinal de controle
    integralTerm = max(-integralTermMax, min(integralTerm, integralTermMax));


    derivativeTerm = Kd * (error - lastError) / Ts; % Variação do erro / Ts (aproximação da derivada discreta)

    % 4. Calcular Sinal de Controle (Saída do PID)
    controlSignal = proportionalTerm + integralTerm + derivativeTerm;

    % 5. Mapear Sinal de Controle para PWM e Atuar no Motor
    % Limitando o sinal de controle para a faixa aceitável de PWM (0 a 1).
    % Este é o sinal que será enviado para a ponte H.
    if controlSignal > 0 % Girar em um sentido (ex: horário)
        pwmDutyCycle = min(abs(controlSignal), 1); % Limita o valor absoluto a 1 (100% PWM)
        writePWMDutyCycle(a, motorPin1, pwmDutyCycle); % PWM em um pino
        writePWMDutyCycle(a, motorPin2, 0);             % Outro pino em 0
    elseif controlSignal < 0 % Girar no sentido contrário (anti-horário)
        pwmDutyCycle = min(abs(controlSignal), 1); % Limita o valor absoluto a 1
        writePWMDutyCycle(a, motorPin1, 0);             % Um pino em 0
        writePWMDutyCycle(a, motorPin2, pwmDutyCycle); % PWM no outro pino
    else % Parar o motor se o erro for zero ou muito pequeno
        writePWMDutyCycle(a, motorPin1, 0);
        writePWMDutyCycle(a, motorPin2, 0);
    end

    % 6. Atualizar Variáveis para a Próxima Iteração
    lastError = error; % O erro atual se torna o "erro anterior" para a próxima iteração

    % 7. Armazenar Dados (opcional, para plotagem)
    % Crescimento dinâmico ou aviso se o buffer estiver cheio
    if k <= maxDataPoints
        positionData(k) = currentPositionDegrees;
        setpointData(k) = targetPositionDegrees;
        controlSignalData(k) = controlSignal;
        timeData(k) = toc;
    elseif mod(k, maxDataPoints/10) == 0 % Avisa de vez em quando se o buffer estiver cheio
        disp('Aviso: Buffer de dados cheio. Os dados mais antigos estão sendo sobrescritos (implementação de buffer circular não aplicada).');
        % Para um buffer circular, você implementaria algo como:
        % dataIndex = mod(k-1, maxDataPoints) + 1;
        % positionData(dataIndex) = currentPositionDegrees; etc.
    end

    % Exibir status
    fprintf('Iteração %d: Tempo=%.2fs, Pos=%.2f°, Setpoint=%.2f°, Erro=%.2f°, Sinal=%.2f\n', ...
            k, timeData(k), currentPositionDegrees, targetPositionDegrees, error, controlSignal);

    % Pausar para o Próximo Período de Amostragem
    elapsedTime = toc - timeData(k);
    if elapsedTime < Ts
        pause(Ts - elapsedTime);
    end

    % --- Ponto para mudar setpoint ou sair ---
    % Por simplicidade, para mudar o setpoint ou parar, você terá que pressionar Ctrl+C

end

% O código abaixo só será executado se o loop 'while true' for interrompido (ex: Ctrl+C)
disp('--- CONTROLE PID FINALIZADO (Interrompido) ---');

% Parar o motor
writePWMDutyCycle(a, motorPin1, 0);
writePWMDutyCycle(a, motorPin2, 0);

% --- Plotar Resultados do PID ---
% Ajusta os vetores de dados para o tamanho real das iterações executadas (k)
figure;
subplot(2,1,1);
plot(timeData(1:k), positionData(1:k), 'b', 'LineWidth', 1.5);
hold on;
plot(timeData(1:k), setpointData(1:k), 'r--', 'LineWidth', 1.5);
hold off;
title('Controle de Posição do Motor');
xlabel('Tempo (s)');
ylabel('Posição (graus)');
legend('Posição Atual', 'SetPoint', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(timeData(1:k), controlSignalData(1:k), 'k', 'LineWidth', 1.5);
title('Sinal de Controle (Saída do PID)');
xlabel('Tempo (s)');
ylabel('Sinal de Controle (Valor Bruto PID)');
grid on;

% --- Fechar a Conexão com o Arduino ---
clear a; % Libera o objeto e fecha a conexão com a ESP32
disp('Conexão com a ESP32 fechada para este script.');