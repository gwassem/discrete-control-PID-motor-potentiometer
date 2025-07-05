% basicPControl.m
% ... (código anterior, incluindo definições de PWM_MAX e PWM_MIN_ACTIVE) ...

% Arrays para armazenar os dados coletados
timeData = zeros(1, length(setPoints) * numSamplesPerSetpoint);
positionData = zeros(1, length(setPoints) * numSamplesPerSetpoint);
setpointData = zeros(1, length(setPoints) * numSamplesPerSetpoint);
% MUDEI O NOME DA VARIÁVEL PARA CLAREZA, E ELA ARMAZENARÁ O VALOR REAL APLICADO
actualPWMApplied = zeros(1, length(setPoints) * numSamplesPerSetpoint); 

currentSampleIndex = 1; % Índice global para preencher os arrays de dados.

% Loop através dos SetPoints
for spIdx = 1:length(setPoints)
    targetSetpoint = setPoints(spIdx);
    fprintf('\n--- Movendo para o SetPoint: %.2f Graus ---\n', targetSetpoint);

    tic; % Resetar o cronômetro
    
    for i = 1:numSamplesPerSetpoint
        currentTime = toc;
        if currentTime < (i-1) * sampleRate
            pause(((i-1) * sampleRate) - currentTime);
            currentTime = toc;
        end

        currentVoltage = readVoltage(a, potPin);
        currentAngle = (currentVoltage - minVoltage) / voltsPerDegree;

        % --- Cálculo do Erro e Ação de Controle P ---
        error = targetSetpoint - currentAngle;
        controlSignal = Kp * error; % Sinal bruto do controlador

        % --- Lógica de Saturação e Limites de Potência ---
        pwmValue = 0; % Inicializa o valor PWM a ser enviado
        appliedDirectionalPWM = 0; % Variável para armazenar o PWM com a direção (para plotagem)

        if controlSignal > 0 % O motor precisa girar em uma direção
            pwmValue = min(controlSignal, PWM_MAX);
            if pwmValue > 0 && pwmValue < PWM_MIN_ACTIVE
                pwmValue = PWM_MIN_ACTIVE;
            end
            
            writePWMDutyCycle(a, motorPin1, pwmValue);
            writePWMDutyCycle(a, motorPin2, 0);
            appliedDirectionalPWM = pwmValue; % Armazena o valor positivo
            
        elseif controlSignal < 0 % O motor precisa girar na direção oposta
            absControlSignal = abs(controlSignal);
            pwmValue = min(absControlSignal, PWM_MAX);
            if pwmValue > 0 && pwmValue < PWM_MIN_ACTIVE
                pwmValue = PWM_MIN_ACTIVE;
            end
            
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, pwmValue);
            appliedDirectionalPWM = -pwmValue; % Armazena o valor negativo para a direção oposta
            
        else % Erro é zero ou muito próximo de zero
            pwmValue = 0; % Desliga o motor
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, 0);
            appliedDirectionalPWM = 0;
        end
        
        % --- Armazenamento dos Dados ---
        timeData(currentSampleIndex) = currentTime + (spIdx - 1) * durationPerSetpoint;
        positionData(currentSampleIndex) = currentAngle;
        setpointData(currentSampleIndex) = targetSetpoint;
        % AGORA SALVA O VALOR REALMENTE APLICADO AO MOTOR
        actualPWMApplied(currentSampleIndex) = appliedDirectionalPWM; 

        currentSampleIndex = currentSampleIndex + 1;

        % Feedback no console (opcional)
        if mod(i, 50) == 0 || i == 1 || i == numSamplesPerSetpoint
            fprintf('  Tempo: %.2fs, SP: %.2f deg, Pos: %.2f deg, Erro: %.2f deg, PWM Aplicado: %.4f\n', ...
                    timeData(currentSampleIndex-1), targetSetpoint, currentAngle, error, appliedDirectionalPWM);
        end

        elapsedTime = toc - currentTime;
        if elapsedTime < sampleRate
            pause(sampleRate - elapsedTime);
        end
    end
    
    writePWMDutyCycle(a, motorPin1, 0);
    writePWMDutyCycle(a, motorPin2, 0);
    disp('Motor parado antes do próximo setpoint ou final.');
end

% ... (código de finalização e limpeza) ...

% 5. Visualização dos Dados Coletados
figure;
subplot(2,1,1);
plot(timeData, setpointData, 'k--', 'LineWidth', 1); hold on;
plot(timeData, positionData, 'r-', 'LineWidth', 1.5);
title('Resposta de Posição com Controle P Básico (Limites Aplicados)');
xlabel('Tempo (s)');
ylabel('Posição (Graus)');
legend('SetPoint', 'Posição Medida');
grid on;
hold off;

subplot(2,1,2);
% AGORA PLOTAMOS O VALOR REALMENTE APLICADO
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
    % ATUALIZANDO O NOME DA VARIÁVEL SALVA
    save('basicPControlResponse.mat', 'timeData', 'positionData', 'setpointData', 'actualPWMApplied', 'Kp', 'PWM_MAX', 'PWM_MIN_ACTIVE');
    disp('Dados de resposta do Controle P Básico salvos em basicPControlResponse.mat!');
catch ME
    disp(['ERRO ao salvar dados do Controle P Básico: ', ME.message]);
end

% ... (restante do código) ...