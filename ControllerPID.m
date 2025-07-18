% simplePositionControllerPID.m
% Script para controle de posição do motor usando um controlador PID.
% Pede a posição angular desejada ao usuário e tenta alcançá-la.
clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- INICIANDO CONTROLADOR PID DE POSIÇÃO ---');

% --- 1. CARREGAMENTO DE CONFIGURAÇÃO E CALIBRAÇÃO ---
% Este script AGORA CARREGA OS VALORES de arquivos .mat.
% Certifique-se de que 'setupPins.m' e 'calibratePotentiometer.m' foram executados
% e salvaram os arquivos 'arduinoPins.mat' e 'potentiometerCalibration.mat'.

% 1.1. Carregar Definições de Pinos
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
    return;
end

% 1.2. Carregar Parâmetros de Calibração do Potenciômetro
try
    load('potentiometerCalibration.mat', 'minVoltage', 'maxVoltage', 'angleRangeDegrees', 'voltsPerDegree');
    if ~exist('minVoltage', 'var') || ~exist('voltsPerDegree', 'var') || ~exist('angleRangeDegrees', 'var')
        error('Parâmetros de calibração não carregados. Execute calibratePotentiometer.m primeiro.');
    end
    fprintf('Tensão Mínima (0 graus): %.4fV\n', minVoltage);
    fprintf('Volts por Grau: %.4fV/deg\n', voltsPerDegree);
    fprintf('Amplitude Angular Calibrada: %.2f graus\n', angleRangeDegrees); % Deveria ser 3600
catch ME1
    disp(['ERRO ao carregar parâmetros de calibração: ', ME1.message]);
    disp('Por favor, execute calibratePotentiometer.m antes de prosseguir.');
    return;
end

% --- Proteção de Fins de Curso (5% de 0 a 3600 graus) ---
% Calcule os ângulos que representam 5% do range calibrado
angleLimitLower = angleRangeDegrees * 0.05; % 5% do limite inferior (do total de 3600)
angleLimitUpper = angleRangeDegrees * 0.95; % 95% do limite superior (do total de 3600)
fprintf('Limite de Operação Seguro: %.2f graus a %.2f graus (5%% dos extremos)\n', angleLimitLower, angleLimitUpper);

% --- 2. CONEXÃO COM ARDUINO/ESP32 ---
a = []; % Inicializa o objeto Arduino como vazio
try
    % Substitua 'COM9' pela porta serial da sua ESP32, se for diferente.
    % Use 'ESP32-WROOM-32' ou 'ESP32-WROOM-DevKitV1' dependendo do seu modelo.
    a = arduino("COM9", "ESP32-WROOM-DevKitV1");
    disp('Conexão com ESP32 estabelecida!');
    configurePin(a, motorPin1, "PWM");
    configurePin(a, motorPin2, "PWM");
    disp('Pinos do motor configurados como PWM.');
catch ME
    disp(['ERRO ao conectar à ESP32 ou configurar pinos: ', ME.message]);
    disp('Verifique se a placa está conectada e a porta COM correta.');
    return; % Sai do script se a conexão falhar
end

% --- 3. PARÂMETROS DO CONTROLADOR PID ---
% Ajuste experimental: Comece com Kp, Ki do PID Tuner (sinal invertido)
% Se houver overshoot ou oscilação excessiva, comece reduzindo Ki.
% Se a resposta for lenta, aumente Kp (e talvez um pouco Ki).
Kp = 0.0027;      % GANHO PROPORCIONAL
Ki = 0.00004393;     % GANHO INTEGRAL (Tente reduzir este se houver muito overshoot ou windup)
Kd = 0;             % GANHO DERIVATIVO

pwmMax = 0.8;    % Duty Cycle Máximo (100% PWM)
pwmMinAtivacao = 0.15; % PWM mínimo para superar atrito.
erroZonaMorta = 5; % [graus] Margem de erro em que o motor é considerado "no alvo".

% --- 4. PEDIR A POSIÇÃO DESEJADA AO USUÁRIO ---
disp(' ');
disp('--- AJUSTE DE POSIÇÃO ---');
posicaoDesejada = input(sprintf('Digite o ângulo desejado (entre %.2f e %.2f graus): ', 0, angleRangeDegrees));

% Validação da entrada e ajuste para dentro dos limites de operação seguros
if posicaoDesejada < angleLimitLower
    warning('Ângulo desejado menor que o limite seguro (%.2f graus). Ajustando para %.2f graus.', angleLimitLower, angleLimitLower);
    posicaoDesejada = angleLimitLower;
elseif posicaoDesejada > angleLimitUpper
    warning('Ângulo desejado maior que o limite seguro (%.2f graus). Ajustando para %.2f graus.', angleLimitUpper, angleLimitUpper);
    posicaoDesejada = angleLimitUpper;
end
fprintf('Ângulo alvo definido para: %.2f graus\n', posicaoDesejada);

% --- 5. INICIALIZAÇÃO PARA REGISTRO DE DADOS E PID ---
% Vetores para armazenar dados para o gráfico
tempoRegistro = [];
posicaoAtualRegistro = [];
posicaoDesejadaRegistro = []; % Para plotar o setpoint
pwmAplicadoRegistro = []; % Armazena o PWM aplicado em cada instante

% Variáveis para o cálculo PID
integralErro = 0;
erroAnterior = 0;

% --- 6. LOOP DE CONTROLE PRINCIPAL ---
disp(' ');
disp('--- INICIANDO CONTROLE ---');
disp('Pressione Ctrl+C no console do MATLAB para PARAR o controle a qualquer momento.');
tempoInicioControle = tic; % Inicia o cronômetro para o tempo de execução do controle
duracaoMaxControle = 120; % [segundos] Tempo máximo para o loop de controle
samplePeriod = 0.02; % [segundos] Período de amostragem do loop de controle

try
    while toc(tempoInicioControle) < duracaoMaxControle
        % A. Ler a posição atual do potenciômetro
        currentVoltage = readVoltage(a, potPin);
        posicaoAtual = (currentVoltage - minVoltage) / voltsPerDegree;

        % B. Verificar limites de fim de curso antes de calcular o erro
        % SE o motor atingir um dos limites de 5% dos fins de curso, DESLIGAR IMEDIATAMENTE.
        if (posicaoAtual < angleLimitLower && posicaoDesejada < posicaoAtual) || ... % Se está abaixo do limite INFERIOR e o alvo está na mesma direção (tentando ir mais baixo)
           (posicaoAtual > angleLimitUpper && posicaoDesejada > posicaoAtual)        % Se está acima do limite SUPERIOR e o alvo está na mesma direção (tentando ir mais alto)
            disp('!!! FIM DE CURSO ATINGIDO OU EXCEDIDO! PARANDO MOTOR IMEDIATAMENTE !!!');
            fprintf('Set: %.2f | Atual: %.2f | Erro: %.2f | P: %.2f | I: %.2f | D: %.2f | PID Bruto: %.2f | PWM Aplicado: %.2f\n', ...
                posicaoDesejada, posicaoAtual, erro, termoP, Ki * integralErro, termoD, rawPidOutput, pwmToApply);
            writePWMDutyCycle(a, motorPin1, 0);
            writePWMDutyCycle(a, motorPin2, 0);
            % Registra os últimos pontos antes de parar
            tempoRegistro = [tempoRegistro; toc(tempoInicioControle)];
            posicaoAtualRegistro = [posicaoAtualRegistro; posicaoAtual];
            posicaoDesejadaRegistro = [posicaoDesejadaRegistro; posicaoDesejada];
            pwmAplicadoRegistro = [pwmAplicadoRegistro; 0]; % PWM 0 ao parar
            break; % Sai do loop de controle
        end
        
        % C. Calcular o Erro
        erro = posicaoDesejada - posicaoAtual;
        
        % Inicializa pwmToApply para esta iteração
        pwmToApply = 0; 
        rawPidOutput = 0; % Variável para armazenar a saída PID ANTES da saturação
        
        % G. Controlar a Direção do Motor com os Pinos PWM
        % Nova lógica: Se o erro estiver dentro da zona morta, considera o target atingido e para o script.
        if abs(erro) < erroZonaMorta 
            disp(' ');
            disp('!!! TARGET ATINGIDO DENTRO DA ZONA MORTA! PARANDO O SCRIPT !!!');
            writePWMDutyCycle(a, motorPin1, 0); % Garante que o motor pare
            writePWMDutyCycle(a, motorPin2, 0);
            % Registra os últimos pontos antes de parar
            tempoRegistro = [tempoRegistro; toc(tempoInicioControle)];
            posicaoAtualRegistro = [posicaoAtualRegistro; posicaoAtual];
            posicaoDesejadaRegistro = [posicaoDesejadaRegistro; posicaoDesejada];
            pwmAplicadoRegistro = [pwmAplicadoRegistro; 0]; % PWM 0 ao atingir o alvo
            break; % Sai do loop de controle e finaliza o script via 'finally'
        else % Erro fora da zona morta, aplicar controle
            % D. Calcular os termos PID (o termo I é condicionalmente atualizado)
            termoP = Kp * erro;
            
            % Termo Derivativo
            derivadaErro = (erro - erroAnterior) / samplePeriod;
            termoD = Kd * derivadaErro; % Kd é 0 neste caso, então termoD será 0

            % Saída do controlador PID ANTES da saturação e anti-windup (para determinar a necessidade de integrar)
            % Aqui calculamos o pidOutput como se não houvesse saturação, apenas para saber a direção
            % e magnitude bruta que o controlador está pedindo.
            rawPidOutput = termoP + Ki * integralErro; % Usa o integralErro atual

            % E. Anti-Windup por Clamping: Atualiza integralErro APENAS se a saída não estiver saturada
            % ou se o erro e a saturação "concordam" na direção (ajudando a sair da saturação)
            if (rawPidOutput < pwmMax && rawPidOutput > -pwmMax) || ... % Não saturado
               (rawPidOutput >= pwmMax && erro > 0) || ...             % Saturado positivo, mas erro ainda positivo (precisa de mais I)
               (rawPidOutput <= -pwmMax && erro < 0)                   % Saturado negativo, mas erro ainda negativo (precisa de mais I)
                integralErro = integralErro + erro * samplePeriod;
            end

            % Saída final do controlador PID, considerando o termo integral atualizado
            pidOutput = termoP + Ki * integralErro;

            % F. Limitar o PWM calculado
            % Saturamos a saída do controlador para os limites de PWM do motor
            pwmToApply = min(abs(pidOutput), pwmMax);

            % Aplica o PWM mínimo de ativação
            if pwmToApply > 0 && pwmToApply < pwmMinAtivacao
                pwmToApply = pwmMinAtivacao;
            end
            
            if pidOutput > 0 % Motor gira em uma direção (aumenta o ângulo)
                writePWMDutyCycle(a, motorPin1, pwmToApply);
                writePWMDutyCycle(a, motorPin2, 0);
            else % Motor gira na direção oposta (diminui o ângulo)
                writePWMDutyCycle(a, motorPin1, 0);
                writePWMDutyCycle(a, motorPin2, pwmToApply);
            end
        end

        % H. Exibir status no console (para depuração)
        % Inclui rawPidOutput para ver o que o PID quer ANTES da saturação
        fprintf('Set: %.2f | Atual: %.2f | Erro: %.2f | P: %.2f | I: %.2f | D: %.2f | PID Bruto: %.2f | PWM Aplicado: %.2f\n', ...
                posicaoDesejada, posicaoAtual, erro, termoP, Ki * integralErro, termoD, rawPidOutput, pwmToApply);

        % I. Registrar dados para o gráfico
        tempoRegistro = [tempoRegistro; toc(tempoInicioControle)];
        posicaoAtualRegistro = [posicaoAtualRegistro; posicaoAtual];
        posicaoDesejadaRegistro = [posicaoDesejadaRegistro; posicaoDesejada];
        
        % O PWM aplicado precisa ter o sinal para o gráfico representar o controle de forma mais clara
        if pidOutput > 0 
            pwmAplicadoRegistro = [pwmAplicadoRegistro; pwmToApply];
        else 
            pwmAplicadoRegistro = [pwmAplicadoRegistro; -pwmToApply]; % Adiciona o sinal negativo para visualização
        end

        % Atualiza erroAnterior para a próxima iteração
        erroAnterior = erro;

        % J. Pequeno atraso para manter o período de amostragem
        pause(samplePeriod);
    end
    disp('Duração máxima do controle atingida. Parando motor.');
catch ME % Bloco 'catch' para capturar erros ou interrupções (Ctrl+C, etc.)
    if strcmp(ME.identifier, 'MATLAB:KeyboardInterrupt')
        disp('Comando Ctrl+C detectado. Tentando desligar o motor e liberar o objeto Arduino imediatamente...');
    else
        disp(['!!! ERRO INESPERADO DURANTE A OPERAÇÃO: ', ME.message, ' !!!']);
    end
finally
    if isvalid(a) 
        disp(' ');
        disp('--- EXECUTANDO ROTINA DE LIMPEZA FINAL: Parando o motor e fechando conexão ---');
        writePWMDutyCycle(a, motorPin1, 0); 
        writePWMDutyCycle(a, motorPin2, 0);
        delete(a); 
        disp('Conexão com ESP32 fechada.');
    elseif exist('a', 'var') && ~isvalid(a)
        disp('Objeto Arduino já foi invalidado. Nenhuma ação de limpeza adicional necessária.');
    end
end
disp(' ');
disp('--- FIM DO SCRIPT DE CONTROLE ---');

% --- 7. Geração do Gráfico de Controle ---
% ESTE BLOCO DEVE FICAR FORA DO LOOP DE CONTROLE E DO BLOCO TRY/CATCH/FINALLY
if ~isempty(tempoRegistro)
    figure; % Cria uma nova figura

    % Subplot 1: Posição Desejada e Posição Atual
    subplot(2,1,1); % 2 linhas, 1 coluna, primeiro gráfico
    
    % Plot the dynamic data (Position)
    plot(tempoRegistro, posicaoDesejadaRegistro, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Posição Desejada (Set)');
    hold on; % Keep the plot active to add more lines
    plot(tempoRegistro, posicaoAtualRegistro, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Posição Atual');
    
    % Add static lines for the dead zone (only once)
    % Use the last desired position, as it's the target the motor aims for
    final_pos_desejada_plot = posicaoDesejadaRegistro(end); 
    
    % Get the current X-axis limits for drawing horizontal lines across the plot
    current_xlim_plot = [tempoRegistro(1) tempoRegistro(end)]; % Use actual data range for robustness
    
    plot(current_xlim_plot, [final_pos_desejada_plot + erroZonaMorta final_pos_desejada_plot + erroZonaMorta], ...
         'k:', 'DisplayName', 'Zona Morta (+)');
    plot(current_xlim_plot, [final_pos_desejada_plot - erroZonaMorta final_pos_desejada_plot - erroZonaMorta], ...
         'k:', 'DisplayName', 'Zona Morta (-)');
    
    grid on;
    title('Controle de Posição do Motor');
    xlabel('Tempo (s)');
    ylabel('Ângulo (graus)');
    legend('Location', 'best');
    hold off; % Release the plot

    % Subplot 2: PWM Aplicado
    subplot(2,1,2); % 2 linhas, 1 coluna, segundo gráfico
    
    % Plot the dynamic data (PWM)
    plot(tempoRegistro, pwmAplicadoRegistro, 'g-', 'LineWidth', 1.5, 'DisplayName', 'PWM Aplicado');
    hold on; % Keep the plot active
    
    % Add static lines for PWM thresholds (only once)
    % Use the same X-axis limits as the first subplot for consistency
    plot(current_xlim_plot, [pwmMinAtivacao pwmMinAtivacao], 'm--', 'DisplayName', 'PWM Mín. Ativação');
    plot(current_xlim_plot, [-pwmMinAtivacao -pwmMinAtivacao], 'm--', 'HandleVisibility','off'); % Hide legend for negative line
    plot(current_xlim_plot, [0 0], 'k--', 'LineWidth', 0.5, 'DisplayName', 'PWM Zero');
    
    grid on;
    title('Sinal de Controle (PWM Aplicado)');
    xlabel('Tempo (s)');
    ylabel('Duty Cycle (-1 a 1)');
    legend('Location', 'best');
    
    % Adjust Y-axis limits dynamically
    if ~isempty(pwmAplicadoRegistro)
        % Add a small buffer to the max/min PWM values for better visualization
        max_abs_pwm = max(abs(pwmAplicadoRegistro));
        ylim([-max_abs_pwm * 1.1, max_abs_pwm * 1.1]); 
    else
        ylim([-pwmMax - 0.1, pwmMax + 0.1]); % Fallback if no PWM data was recorded
    end
    hold off;

    % Check if sgtitle is available (R2018a and later)
    if exist('sgtitle', 'file') == 2
        sgtitle('Análise do Controlador PID de Posição'); % Overall title for the figure
    else
        set(gcf, 'Name', 'Análise do Controlador PID de Posição'); % Fallback for older versions
    end
else
    disp('Nenhum dado de controle registrado para plotar o gráfico.');
end