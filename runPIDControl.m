% runPIDControl.m
% Script para executar o controle PID de posição do motor.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- INICIANDO CONTROLE PID DE POSIÇÃO ---');

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

% --- Parâmetros do Controlador PID ---
% ESTES SÃO OS VALORES CRÍTICOS QUE VOCÊ AJUSTARÁ COM O PID TUNER.
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
lastPositionDegrees = initialPositionDegrees;

% Armazenamento de dados para análise posterior (opcional)
maxRunTime = 60; % Tempo máximo de execução do controle em segundos
numIterations = ceil(maxRunTime / Ts); % Calcula o número de iterações

positionData = zeros(numIterations, 1);
setpointData = zeros(numIterations, 1);
controlSignalData = zeros(numIterations, 1);
timeData = zeros(numIterations, 1);

disp('--- INICIANDO CONTROLE PID ---');

% --- SOLICITAÇÃO DO ÂNGULO DESEJADO AO USUÁRIO ---
while true
    prompt = sprintf('Digite o ângulo desejado em graus (0 a %.0f, ou "q" para sair): ', angleRangeDegrees);
    userInput = input(prompt, 's');

    if strcmpi(userInput, 'q')
        disp('Saindo do programa.');
        % Parar o motor antes de sair
        writePWMDutyCycle(a, motorPin1, 0);
        writePWMDutyCycle(a, motorPin2, 0);
        clear a; % Libera o objeto e fecha a conexão com a ESP32
        return; % Sai da função/script
    end

    targetPositionDegrees = str2double(userInput);

    if ~isnan(targetPositionDegrees) && targetPositionDegrees >= 0 && targetPositionDegrees <= angleRangeDegrees
        fprintf('Ângulo desejado definido para: %.2f graus\n', targetPositionDegrees);
        break; % Sai do loop de entrada se o valor for válido
    else
        disp('Entrada inválida. Por favor, digite um número entre 0 e o valor máximo da faixa calibrada, ou "q".');
    end
end

% Loop de Controle
disp('Iniciando o loop de controle...');
tic; % Inicia temporizador para o loop
for k = 1:numIterations
    % 1. Medir a Posição Atual
    currentVoltage = readVoltage(a, potPin);
    currentPositionDegrees = voltageToDegrees(currentVoltage);

    % 2. Calcular o Erro
    error = targetPositionDegrees - currentPositionDegrees;

    % 3. Calcular Termos PID
    proportionalTerm = Kp * error;

    integrativeError = integrativeError + error * Ts;
    integralTerm = Ki * integrativeError;

    derivativeTerm = Kd * (error - lastError) / Ts;

    % 4. Calcular Sinal de Controle (Saída do PID)
    controlSignal = proportionalTerm + integralTerm + derivativeTerm;

    % 5. Mapear Sinal de Controle para PWM e Atuar no Motor
    % Limitando o sinal de controle para a faixa aceitável de PWM (0 a 1).
    if controlSignal > 0 % Girar em um sentido (ex: horário)
        pwmDutyCycle = min(abs(controlSignal), 1); % Limita a 1 (100% PWM)
        writePWMDutyCycle(a, motorPin1, pwmDutyCycle); % PWM em um pino
        writePWMDutyCycle(a, motorPin2, 0);             % Outro pino em 0
    elseif controlSignal < 0 % Girar no sentido contrário (anti-horário)
        pwmDutyCycle = min(abs(controlSignal), 1); % Limita a 1
        writePWMDutyCycle(a, motorPin1, 0);             % Um pino em 0
        writePWMDutyCycle(a, motorPin2, pwmDutyCycle); % PWM no outro pino
    else % Parar o motor se o erro for zero ou muito pequeno
        writePWMDutyCycle(a, motorPin1, 0);
        writePWMDutyCycle(a, motorPin2, 0);
    end

    % 6. Atualizar Variáveis para a Próxima Iteração
    lastError = error;
    % lastPositionDegrees = currentPositionDegrees; % Não usado diretamente para derivativo agora, mas pode ser útil para debug

    % 7. Armazenar Dados (para plotagem e análise)
    positionData(k) = currentPositionDegrees;
    setpointData(k) = targetPositionDegrees;
    controlSignalData(k) = controlSignal;
    timeData(k) = toc;

    % Exibir status (opcional, pode ser lento se Ts for muito pequeno)
    fprintf('Iteração %d: Tempo=%.2fs, Pos=%.2f°, Erro=%.2f°, Sinal=%.2f\n', ...
            k, timeData(k), currentPositionDegrees, error, controlSignal);

    % 8. Pausar para o Próximo Período de Amostragem
    elapsedTime = toc - timeData(k);
    if elapsedTime < Ts
        pause(Ts - elapsedTime);
    end

    % Condição de parada adicional: se o erro for pequeno e o sistema estiver estabilizado
    % Ajuste a tolerância (1.0 grau) e o sinal de controle (0.1) conforme necessário.
    if abs(error) < 1.0 && abs(controlSignal) < 0.1 && k > 20 % k > 20 para dar tempo de estabilizar
        disp('Posição atingida e estabilizada (dentro da tolerância).');
        break; % Sai do loop de controle
    end
end

% Parar o motor ao final do loop
writePWMDutyCycle(a, motorPin1, 0);
writePWMDutyCycle(a, motorPin2, 0);

disp('--- CONTROLE PID FINALIZADO ---');

% --- Plotar Resultados do PID ---
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
disp('Conexão com a ESP32 fechada.');