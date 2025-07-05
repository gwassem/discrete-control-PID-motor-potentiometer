% fullPIDControl.m
% Script para implementar um controle de posição de malha fechada com
% controlador PID, utilizando os ganhos obtidos do PID Tuner.
% Inclui um anti-windup robusto para o termo integral.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- INICIANDO CONTROLE PID COMPLETO ---');

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
% Tenta carregar os parâmetros de calibração salvas por 'calibratePotentiometer.m'.
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

% --- PARÂMETROS DO CONTROLADOR PID ---
% ESTES VALORES SERÃO POPULADOS COM OS RESULTADOS DO PID TUNER
% Exemplo de valores (APENAS PARA TESTE INICIAL, VOCÊS VÃO MUDAR ISSO!)
Kp = 0.01;   % Ganho Proporcional
Ki = 0.0001; % Ganho Integral
Kd = 0.001;  % Ganho Derivativo

fprintf('\nPARÂMETROS PID DEFINIDOS:\n  Kp = %.4f\n  Ki = %.4f\n  Kd = %.4f\n', Kp, Ki, Kd);

% --- Limites de Potência (Duty Cycle) do Motor ---
PWM_MAX = 0.60;         % Limite superior para o duty cycle (60% da potência máxima).
PWM_MIN_ACTIVE = 0.15;  % Limite inferior para o duty cycle que garante que o motor se mova.

% Setpoints (posições desejadas em graus)
setPoints = [900, 1800]; % Exemplo: 900 graus (2.5 voltas), depois 1800 graus (5 voltas).
                      % Certifique-se de que esses valores estejam dentro da faixa calibrada do seu potenciômetro.
                      
% Tolerância para considerar que o motor atingiu o setpoint (para feedback visual).
tolerance = 5; % Graus.

% Parâmetros da Coleta de Dados para Análise
durationPerSetpoint = 15; % Tempo em segundos para manter cada setpoint e coletar dados.
                           % Deve ser longo o suficiente para o sistema se estabilizar.
sampleRate = 0.02;         % Intervalo de amostragem em segundos (50 Hz).
numSamplesPerSetpoint = round(durationPerSetpoint / sampleRate);

% Variáveis para o cálculo do PID (estado interno do controlador)
integralError = 0;   % Acumulador do erro integral
lastError = 0;       % Último erro para cálculo do erro derivativo

% Arrays para armazenar os dados coletados
timeData = zeros(1, length(setPoints) * numSamplesPerSetpoint);
positionData = zeros(1, length(setPoints) * numSamplesPerSetpoint);
setpointData = zeros(1, length(setPoints) * numSamplesPerSetpoint);
actualPWMApplied = zeros(1, length(setPoints) * numSamplesPerSetpoint); % Armazena o PWM real aplicado

currentSampleIndex = 1; % Índice global para preencher os arrays de dados.

% Loop através dos SetPoints
for spIdx = 1:length(setPoints)
    targetSetpoint = setPoints(spIdx);
    fprintf('\n--- Movendo para o SetPoint: %.2f Graus ---\n', targetSetpoint);

    % Resetar variáveis do PID para cada novo setpoint
    % (especialmente importante para o integral em mudanças abruptas de SP)
    integralError = 0;
    lastError = 0;     

    tic; % Resetar o cronômetro para cada setpoint
    
    % Loop de controle e coleta de dados para o setpoint atual
    for i = 1:numSamplesPerSetpoint
        currentTime = toc; % Tempo atual desde o início do setpoint.

        % Garante que a amostra seja coletada no tempo correto, pausando se necessário.
        if currentTime < (i-1) * sampleRate
            pause(((i-1) * sampleRate) - currentTime);
            currentTime = toc;
        end

        currentVoltage = readVoltage(a, potPin); % Lê a tensão do potenciômetro.
        currentAngle = (currentVoltage - minVoltage) / voltsPerDegree; % Converte para graus.

        % --- Cálculo do Erro ---
        error = targetSetpoint - currentAngle;
        
        % --- Cálculo dos Termos PID ---
        % Termo Proporcional
        proportionalTerm = Kp * error;

        % Termo Derivativo
        derivativeTerm = Kd * ((error - lastError) / sampleRate); % Discretização da derivada

        % --- Anti-Windup para o Termo Integral ---
        % Calcula um candidato para o novo erro integral, caso ele fosse acumulado
        integralError_candidate = integralError + (error * sampleRate); % Acumula o erro

        % Calcula um sinal de controle hipotético (demanda do controlador)
        % com o erro integral candidato. Isso é o que o controlador gostaria de gerar.
        hypothetical_controlSignal = proportionalTerm + (Ki * integralError_candidate) + derivativeTerm;

        % Lógica de Anti-Windup:
        % O erro integral só é atualizado para 'integralError_candidate' se:
        % 1. A demanda do controlador (hypothetical_controlSignal) não ultrapassa PWM_MAX
        % OU
        % 2. Se a demanda ultrapassa PWM_MAX, mas o erro mudou de sinal (indicando que o sistema
        % está saindo da saturação ou superando a saturação inicial).
        % Isso evita que o integral continue acumulando quando o atuador já está no limite e o erro
        % continua empurrando na mesma direção da saturação.
        
        % Condição para NÃO ATUALIZAR integralError (manter o valor anterior):
        % Se hypothetical_controlSignal está acima de PWM_MAX E o erro ainda é positivo, OU
        % Se hypothetical_controlSignal está abaixo de -PWM_MAX E o erro ainda é negativo.
        if (hypothetical_controlSignal > PWM_MAX && error > 0) || ...
           (hypothetical_controlSignal < -PWM_MAX && error < 0)
            % Não faz nada. integralError mantém seu valor anterior, evitando wind-up.
        else
            % Caso contrário (sinal dentro dos limites ou erro mudando de direção),
            % o integral acumula normalmente.
            integralError = integralError_candidate;
        end

        % Sinal de Controle PID Final (com o termo integral já ajustado pelo anti-windup)
        controlSignal = proportionalTerm + (Ki * integralError) + derivativeTerm;

        % Atualiza o último erro para a próxima iteração
        lastError = error;

        % --- Lógica de Saturação e Limites de Potência (PWM_MAX, PWM_MIN_ACTIVE) ---
        pwmValue = 0; % Valor PWM a ser enviado (entre 0 e 1)
        appliedDirectionalPWM = 0; % Valor PWM com sinal para plotagem (-1 a 1)

        if controlSignal > 0 % Motor deve girar em uma direção
            pwmValue = min(controlSignal, PWM_MAX); % Aplica o limite superior
            % Aplica o limite inferior ativo (se pwmValue não for zero e for muito baixo)
            if pwmValue > 0 && pwmValue < PWM_MIN_ACTIVE
                pwmValue = PWM_MIN_ACTIVE;
            end
            
            writePWMDutyCycle(a, motorPin1, pwmValue);
            writePWMDutyCycle(a, motorPin2, 0);
            appliedDirectionalPWM = pwmValue; % Valor real aplicado, positivo
            
        elseif controlSignal < 0 % Motor deve girar na direção oposta
            absControlSignal = abs(controlSignal); % Usa o valor absoluto
            pwmValue = min(absControlSignal, PWM_MAX); % Aplica o limite superior
            % Aplica o limite inferior ativo
            if pwmValue > 0 && pwmValue < PWM_MIN_ACTIVE
                pwmValue = PWM_MIN_ACTIVE;
            end
            
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, pwmValue);
            appliedDirectionalPWM = -pwmValue; % Valor real aplicado, negativo
            
        else % Erro é zero ou muito próximo de zero
            pwmValue = 0; % Desliga o motor
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, 0);
            appliedDirectionalPWM = 0;
        end
        
        % --- Armazenamento dos Dados ---
        timeData(currentSampleIndex) = currentTime + (spIdx - 1) * durationPerSetpoint; % Tempo acumulado
        positionData(currentSampleIndex) = currentAngle;
        setpointData(currentSampleIndex) = targetSetpoint;
        actualPWMApplied(currentSampleIndex) = appliedDirectionalPWM; % PWM real enviado ao motor

        currentSampleIndex = currentSampleIndex + 1;

        % Feedback no console (opcional, para acompanhar o processo)
        if mod(i, 50) == 0 || i == 1 || i == numSamplesPerSetpoint % Imprime a cada 50 amostras ou no início/fim
            fprintf('  T: %.2fs, SP: %.2f deg, Pos: %.2f deg, Erro: %.2f deg, PWM: %.4f\n', ...
                    timeData(currentSampleIndex-1), targetSetpoint, currentAngle, error, appliedDirectionalPWM);
        end

        % Adicionar um pequeno atraso para manter a taxa de amostragem
        elapsedTime = toc - currentTime;
        if elapsedTime < sampleRate
            pause(sampleRate - elapsedTime);
        end
    end
    
    % Parar o motor ao final de cada setpoint
    writePWMDutyCycle(a, motorPin1, 0);
    writePWMDutyCycle(a, motorPin2, 0);
    disp('Motor parado antes do próximo setpoint ou final.');
end

% 4. Finalização e Limpeza
disp(' ');
disp('--- Controle PID Completo Finalizado ---');
clear a; % Fecha a conexão com a ESP32.
disp('Conexão com ESP32 fechada.');

% 5. Visualização dos Dados Coletados
figure;
subplot(2,1,1);
plot(timeData, setpointData, 'k--', 'LineWidth', 1); hold on;
plot(timeData, positionData, 'r-', 'LineWidth', 1.5);
title('Resposta de Posição com Controle PID (Limits e Anti-Windup Aplicados)');
xlabel('Tempo (s)');
ylabel('Posição (Graus)');
legend('SetPoint', 'Posição Medida');
grid on;
hold off;

subplot(2,1,2);
plot(timeData, actualPWMApplied, 'b-', 'LineWidth', 1);
yline(PWM_MAX, 'r--', 'Max PWM');
yline(-PWM_MAX, 'r--', 'Min PWM');
yline(PWM_MIN_ACTIVE, 'g:', 'Min Active PWM');
yline(-PWM_MIN_ACTIVE, 'g:', 'Min Active PWM');
title('Sinal de Controle (PWM Realmente Aplicado)');
xlabel('Tempo (s)');
ylabel('Duty Cycle Aplicado (-1 a 1)');
grid on;

% 6. Salvar os Dados para Comparação Futura
try
    save('fullPIDControlResponse.mat', 'timeData', 'positionData', 'setpointData', 'actualPWMApplied', 'Kp', 'Ki', 'Kd', 'PWM_MAX', 'PWM_MIN_ACTIVE');
    disp('Dados de resposta do Controle PID salvos em fullPIDControlResponse.mat!');
catch ME
    disp(['ERRO ao salvar dados de resposta do Controle PID: ', ME.message]);
end

disp(' ');
disp('Agora, após ajustar os ganhos PID no PID Tuner, edite este script (fullPIDControl.m)');
disp('e insira os valores otimizados de Kp, Ki e Kd nas variáveis correspondentes.');
disp('Então, execute-o para ver a performance do seu controlador PID ajustado!');