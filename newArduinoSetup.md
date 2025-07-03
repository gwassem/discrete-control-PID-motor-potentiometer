## Visão Geral dos Scripts:

1.  **`setupPins.m`**: Conecta à ESP32 e configura os pinos necessários.
2.  **`calibratePotentiometer.m`**: Realiza a calibração do potenciômetro para mapear tensão em graus e salva os parâmetros de calibração.
3.  **`collectSystemResponse.m`**: Aplica um sinal de degrau ou rampa ao motor, coleta os dados de entrada (PWM) e saída (posição do potenciômetro) para uso na *System Identification Toolbox*.
4.  **`runPIDControl.m`**: Implementa o controlador PID usando os ganhos Kp, Ki, Kd obtidos (preferencialmente após usar o *PID Tuner* com o modelo identificado).

-----

### Script 1: `setupPins.m`

Este script será responsável por estabelecer a conexão com a sua ESP32 e configurar os pinos como PWM para o motor e leitura analógica para o potenciômetro.

```matlab
% setupPins.m
% Script para configurar a conexão com a ESP32 e definir os modos dos pinos.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- CONFIGURANDO CONEXÃO E PINOS ---');

% 1. Crie o objeto 'arduino'
% Substitua 'COMx' pela porta serial da sua ESP32 (ex: 'COM3', 'COM4', '/dev/ttyUSB0')
% Se o objeto 'a' já estiver em uso, limpe-o antes de recriar.
if exist('a', 'var') && isvalid(a)
    clear a;
end
try
    a = arduino("COMx", "ESP32 Dev Module"); % Adapte "COMx" e o nome da placa se necessário.
    disp('Conexão com ESP32 estabelecida com sucesso!');
catch ME
    disp(['ERRO ao conectar à ESP32: ', ME.message]);
    disp('Verifique se a placa está conectada, os drivers instalados e a porta COM correta.');
    return; % Sai do script se não conseguir conectar
end

% Defina os pinos que serão usados
global potPin motorPin1 motorPin2; % Declara como global para acesso em outros scripts
potPin = "D35"; % Pino do potenciômetro (ADC)
motorPin1 = "D25"; % Pino de controle do motor (PWM)
motorPin2 = "D26"; % Pino de controle do motor (PWM)

% Configure o modo dos pinos
configurePin(a, motorPin1, "PWM");
configurePin(a, motorPin2, "PWM");

% A leitura do pino analógico D35 já é configurada automaticamente pelo pacote.

disp('Pinos configurados:');
disp(['  Potenciômetro: ', potPin]);
disp(['  Motor IN1: ', motorPin1]);
disp(['  Motor IN2: ', motorPin2]);

% Salva o objeto 'a' e as variáveis de pino em um arquivo .mat
% para que outros scripts possam carregá-los.
save('arduinoSetup.mat', 'a', 'potPin', 'motorPin1', 'motorPin2');
disp('Configuração salva em arduinoSetup.mat.');

disp('--- CONFIGURAÇÃO CONCLUÍDA ---');

% Nota: O objeto 'a' precisa ser persistente (global ou salvo/carregado)
% entre os scripts se você não quiser reconectar a cada vez.
% Usamos 'global' e 'save/load' para isso.
```

-----

### Script 2: `calibratePotentiometer.m`

Este script interativo o guiará pela calibração do potenciômetro, salvando os parâmetros de mapeamento para uso posterior.

```matlab
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
% Este é o valor mais importante a ser ajustado manualmente com base na sua montagem.
% Se o seu motor, com o acoplamento do potenciômetro de 10 voltas,
% pode girar, por exemplo, 5 voltas completas (5 * 360 = 1800 graus), use 1800.
% Se ele tem um limite físico que é equivalente a 7.5 voltas do pot,
% então seria 7.5 * 360 = 2700 graus.
angleRangeDegrees = 1800; % <--- AJUSTE ESTE VALOR para a faixa REAL em graus do seu motor/pot!

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
% Isso será recriado no script de controle.
voltageToDegrees = @(voltage) (voltage - minVoltage) / voltsPerDegree;

disp('--- CALIBRAÇÃO CONCLUÍDA ---');
```

-----

### Script 3: `collectSystemResponse.m`

Este script aplica uma sequência de sinais (degrau ou rampa) ao motor e registra a resposta. Os dados coletados serão usados na *System Identification Toolbox*.

```matlab
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
disp('4. Use as ferramentas da Toolbox para estimar um modelo do seu sistema.');
```

-----

### Script 4: `runPIDControl.m`

Este script implementa o controle PID em tempo real. Os ganhos `Kp`, `Ki`, `Kd` devem ser ajustados após a identificação do sistema e uso do *PID Tuner*.

```matlab
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
```

-----

### Fluxo de Trabalho Passo a Passo com os Novos Scripts:

1.  **Execute `setupPins.m`:**

      * Abra o MATLAB.
      * Salve `setupPins.m` em sua pasta de trabalho.
      * Edite `setupPins.m` para definir a porta `COM` correta da sua ESP32.
      * Execute `setupPins.m` (clique em Run ou digite `setupPins` na Command Window). Isso criará a conexão e salvará `arduinoSetup.mat`.

2.  **Execute `calibratePotentiometer.m`:**

      * Salve `calibratePotentiometer.m` na mesma pasta.
      * **Edite a linha `angleRangeDegrees = 1800;`** para refletir a faixa angular **real** que o seu motor (com o potenciômetro de 10 voltas acoplado) pode girar. Este é um passo **CRÍTICO**.
      * Execute `calibratePotentiometer.m`. Siga as instruções no console para mover o motor manualmente e registrar as posições min/max. Isso salvará `potentiometerCalibration.mat`.

3.  **Execute `collectSystemResponse.m`:**

      * Salve `collectSystemResponse.m` na mesma pasta.
      * Execute `collectSystemResponse.m`. O motor irá se mover de acordo com a sequência de PWM definida. Ao final, os dados serão plotados e salvos em `systemResponseData.mat`.

4.  **Use a System Identification Toolbox (MATLAB):**

      * Após `collectSystemResponse.m` terminar, digite `ident` na Command Window do MATLAB.
      * Na janela da System Identification Toolbox, vá em "Import data" -\> "Time domain data".
      * Selecione "From workspace" e importe a variável `data`.
      * Agora, use as ferramentas da toolbox (como "Estimate" -\> "Transfer function models" ou "Process models") para criar um modelo matemático do seu sistema motor-potenciômetro a partir dos dados coletados. Você provavelmente terá um modelo de primeira ou segunda ordem com ou sem atraso de tempo.
      * Exporte o modelo identificado para o Workspace do MATLAB (ex: `sys_identified`).

5.  **Use o PID Tuner (MATLAB/Simulink):**

      * Com o modelo `sys_identified` no seu Workspace, você pode usar o PID Tuner.
      * Você pode digitar `pidTuner(sys_identified)` na Command Window.
      * A interface do PID Tuner permitirá que você ajuste visualmente os ganhos Kp, Ki, Kd para obter a resposta desejada (sem overshoot, tempo de acomodação rápido, etc.).
      * Anote os valores de Kp, Ki e Kd que o PID Tuner sugere.

6.  **Execute `runPIDControl.m`:**

      * Salve `runPIDControl.m` na mesma pasta.
      * **Edite as linhas `Kp = 0.5; Ki = 0.01; Kd = 0.05;`** e insira os valores de Kp, Ki, Kd que você obteve do PID Tuner.
      * Execute `runPIDControl.m`. O script pedirá o ângulo desejado. Insira um valor e observe o motor se mover para a posição e o gráfico da resposta.
