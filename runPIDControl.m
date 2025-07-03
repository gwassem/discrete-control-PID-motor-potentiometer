% runPIDControl.m
% Script essencial para implementar o controle PID de posição de um motor DC.

clc; clear all; close all; % Limpa console, workspace e fecha figuras
disp('--- INICIANDO CONTROLE PID DE POSIÇÃO DO MOTOR ---');

% --- 1. Carregar Configurações e Calibração ---
try
    load('arduinoPins.mat', 'potPin', 'motorPin1', 'motorPin2');
    load('potentiometerCalibration.mat', 'minVoltage', 'maxVoltage', 'angleRangeDegrees', 'voltsPerDegree');
    if ~exist('potPin', 'var') || ~exist('minVoltage', 'var')
        error('Dependências não carregadas. Execute setupPins.m e calibratePotentiometer.m primeiro.');
    end
    disp('Configurações e calibração carregadas.');
catch ME
    disp(['ERRO ao carregar dependências: ', ME.message]);
    disp('Por favor, execute setupPins.m e calibratePotentiometer.m antes de prosseguir.');
    return;
end

% --- 2. Conectar ao Arduino ---
a = []; % Inicializa 'a' como vazio
try
    % PERGUNTA AO USUÁRIO A PORTA COM
    comPort = input('Digite a porta COM da sua ESP32 (ex: ''COM3''): ', 's');
    boardName = "ESP32-WROOM-32-DevKitV1"; % OU "ESP32 Dev Module" - Mantenha este valor fixo aqui.
    
    a = arduino(comPort, boardName);
    disp(['Conexão com ESP32 em ', comPort, ' estabelecida!']);

    configurePin(a, motorPin1, "PWM");
    configurePin(a, motorPin2, "PWM");
    disp('Pinos do motor configurados.');

catch ME
    disp(['ERRO ao conectar ou configurar a ESP32: ', ME.message]);
    disp('Verifique se a placa está conectada e a porta COM correta.');
    return;
end

% --- 3. Definir Ganhos PID ---
% ESTES VALORES DEVEM SER AJUSTADOS APÓS A IDENTIFICAÇÃO DO SISTEMA E USO DO PID TUNER!
Kp = 0.5;   % Ganho Proporcional
Ki = 0.01;  % Ganho Integral
Kd = 0.05;  % Ganho Derivativo
Ts = 0.05;  % Período de amostragem (segundos)

disp('Ganhos PID (Kp, Ki, Kd) e Ts definidos. Ajuste-os para o melhor desempenho!');

% --- 4. Definir Setpoint Inicial ---
targetPositionDegrees = input(sprintf('Digite o SetPoint em graus (0 a %.0f): ', angleRangeDegrees));
if targetPositionDegrees < 0 || targetPositionDegrees > angleRangeDegrees
    warning(sprintf('SetPoint fora da faixa válida (0 a %.0f). Usando 0 graus.', angleRangeDegrees));
    targetPositionDegrees = 0;
end
disp(['SetPoint inicial definido para: ', num2str(targetPositionDegrees), ' graus.']);

% --- 5. Inicializar Variáveis do PID e Histórico para Gráfico ---
integrativeError = 0; % Variável para o termo integral
lastError = 0;        % Variável para o termo derivativo
controlSignal = 0;    % Sinal de controle inicial

% VETORES PARA ARMAZENAR HISTÓRICO PARA O GRÁFICO
positionHistory = [];
timeHistory = [];

% --- 6. Loop Principal do Controle PID ---
disp('Iniciando o loop de controle PID. Pressione Ctrl+C na janela de comando para parar.');
disp(' '); % Linha em branco para melhor leitura

globalStartTime = tic; % Inicia o cronômetro para o tempo total de execução
try
    while true % Loop contínuo até ser interrompido manualmente (Ctrl+C)
        currentLoopTime = toc(globalStartTime); % Tempo decorrido desde o início

        % 1. Medir a Posição Atual
        currentVoltage = readVoltage(a, potPin);
        currentPositionDegrees = (currentVoltage - minVoltage) / voltsPerDegree;

        % 2. Calcular o Erro
        error = targetPositionDegrees - currentPositionDegrees;

        % 3. Calcular Termos PID
        proportionalTerm = Kp * error;

        integrativeError = integrativeError + error * Ts;
        integralTerm = Ki * integrativeError;
        % Anti-windup simples para o termo integral
        integralTermMax = 0.5; 
        integralTerm = max(-integralTermMax, min(integralTerm, integralTermMax));

        derivativeTerm = Kd * (error - lastError) / Ts;

        % 4. Calcular Sinal de Controle
        controlSignal = proportionalTerm + integralTerm + derivativeTerm;

        % 5. Mapear Sinal de Controle para PWM e Atuar no Motor
        if controlSignal > 0 % Sentido 1
            pwmDutyCycle = min(abs(controlSignal), 1);
            writePWMDutyCycle(a, motorPin1, pwmDutyCycle);
            writePWMDutyCycle(a, motorPin2, 0);
        elseif controlSignal < 0 % Sentido 2
            pwmDutyCycle = min(abs(controlSignal), 1);
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, pwmDutyCycle);
        else % Parar
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, 0);
        end

        % 6. Atualizar Variáveis para a Próxima Iteração e Armazenar Histórico
        lastError = error;
        positionHistory = [positionHistory; currentPositionDegrees]; % Armazena a posição
        timeHistory = [timeHistory; currentLoopTime]; % Armazena o tempo

        % Exibir posição atual (opcional, remova para otimização)
        fprintf('Tempo: %.2fs | SetPoint: %.2f° | Posição Atual: %.2f° | Erro: %.2f°\n', ...
                currentLoopTime, targetPositionDegrees, currentPositionDegrees, error);

        % Pausa para manter o período de amostragem
        pause(Ts);
    end

% --- 7. Finalização (Executado ao interromper o loop com Ctrl+C) ---
catch ME % Captura a exceção quando Ctrl+C é pressionado
    if (strcmp(ME.identifier, 'MATLAB:array:SizeLimitExceeded') || strcmp(ME.identifier, 'MATLAB:BREAK'))
        disp(' '); % Linha em branco
        disp('Controle PID interrompido pelo usuário (Ctrl+C).');
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

% --- 8. Geração do Gráfico da Posição ---
if ~isempty(positionHistory)
    figure;
    plot(timeHistory, positionHistory, 'b-', 'LineWidth', 1.5);
    hold on;
    % Adiciona linha do SetPoint
    plot(timeHistory, targetPositionDegrees * ones(size(timeHistory)), 'r--', 'LineWidth', 1);
    hold off;
    title('Histórico de Posição do Motor com Controle PID');
    xlabel('Tempo (s)');
    ylabel('Posição (graus)');
    legend('Posição Atual', 'SetPoint Desejado', 'Location', 'best');
    grid on;
    disp('Gráfico da posição gerado.');
else
    disp('Nenhum dado de posição foi coletado para gerar o gráfico.');
end

disp('--- FIM DO SCRIPT DE CONTROLE PID ---');v