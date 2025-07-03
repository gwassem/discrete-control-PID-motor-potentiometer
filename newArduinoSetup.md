
````markdown
## Visão Geral dos Scripts:

1.  **`setupPins.m`**: Conecta à ESP32 (temporariamente para verificação e configuração inicial), define os pinos necessários e salva os **nomes dos pinos**. **O objeto `arduino` não é salvo, pois é uma conexão ativa e não pode ser persistente.**
2.  **`calibratePotentiometer.m`**: **Recria a conexão `arduino`**, realiza a calibração do potenciômetro para mapear tensão em graus e salva os parâmetros de calibração.
3.  **`collectSystemResponse.m`**: **Recria a conexão `arduino`**, aplica um sinal de degrau ou rampa ao motor, coleta os dados de entrada (PWM) e saída (posição do potenciômetro) para uso na *System Identification Toolbox*.
4.  **`runPIDControl.m`**: **Recria a conexão `arduino`**, implementa o controlador PID usando os ganhos Kp, Ki, Kd obtidos (preferencialmente após usar o *PID Tuner* com o modelo identificado). Este script agora roda em um **loop contínuo** para manter a posição.

-----

### Script 1: `setupPins.m`

Este script será responsável por estabelecer a conexão com a sua ESP32 e configurar os pinos como PWM para o motor e leitura analógica para o potenciômetro.

```matlab
% setupPins.m
% Script para configurar a conexão com a ESP32 e definir os modos dos pinos.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- CONFIGURANDO CONEXÃO E PINOS ---');

% Defina os pinos que serão usados (serão salvos e carregados por outros scripts)
potPin = "D35";    % Pino do potenciômetro (ADC)
motorPin1 = "D25"; % Pino de controle do motor (PWM)
motorPin2 = "D26"; % Pino de controle do motor (PWM)

% Tenta criar o objeto arduino para verificar a conexão e configurar os pinos.
% Este objeto 'a' é temporário e VAI SER CRIADO NOVAMENTE em outros scripts.
a = []; % Inicializa 'a' como vazio
try
    % Substitua 'COMx' pela porta serial da sua ESP32 (ex: 'COM3', '/dev/ttyUSB0')
    % O nome da placa "ESP32-WROOM-32-DevKitV1" é um exemplo comum.
    % Verifique o nome exato do módulo se este não funcionar.
    a = arduino("COMx", "ESP32-WROOM-32-DevKitV1");
    disp('Conexão com ESP32 estabelecida com sucesso para configuração inicial!');

    % Configure o modo dos pinos
    configurePin(a, motorPin1, "PWM");
    configurePin(a, motorPin2, "PWM");
    disp('Pinos configurados no hardware.');

    % Fecha a conexão imediatamente após a configuração, pois não pode ser salva.
    clear a;
    disp('Conexão temporária com ESP32 fechada. Será reaberta em outros scripts.');

catch ME
    disp('----------------------------------------------------');
    disp(['ERRO ao conectar ou configurar a ESP32: ', ME.message]);
    disp('Verifique os seguintes pontos:');
    disp('1. A ESP32 está conectada ao computador?');
    disp('2. Você substituiu "COMx" pela porta serial correta?');
    disp('3. Os drivers USB da ESP32 estão instalados?');
    disp('4. O nome da placa "ESP32-WROOM-32-DevKitV1" está correto para sua placa?');
    disp('----------------------------------------------------');
    return; % Sai do script se não conseguir conectar
end

disp('Pinos definidos para uso futuro:');
disp(['  Potenciômetro: ', potPin]);
disp(['  Motor IN1: ', motorPin1]);
disp(['  Motor IN2: ', motorPin2]);

% Salva APENAS as variáveis de pino (strings) em um arquivo .mat
% O objeto 'a' NÃO é salvo aqui, pois não pode ser persistente.
save('arduinoPins.mat', 'potPin', 'motorPin1', 'motorPin2');
disp('Definições de pinos salvas em arduinoPins.mat.');

disp('--- CONFIGURAÇÃO CONCLUÍDA ---');
````

-----

### Script 2: `calibratePotentiometer.m`

Este script interativo o guiará pela calibração do potenciômetro, salvando os parâmetros de mapeamento para uso posterior.

```matlab
% calibratePotentiometer.m
% Script para calibrar o potenciômetro (tensão para graus).

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- CALIBRAÇÃO DO POTENCIÔMETRO ---');

% Carrega APENAS as definições de pinos (strings)
try
    load('arduinoPins.mat', 'potPin'); % Apenas 'potPin' necessário aqui
    if ~exist('potPin', 'var')
        error('Definições de pinos não carregadas. Execute setupPins.m primeiro.');
    end
    disp('Definições de pinos carregadas com sucesso!');
catch ME
    disp(['ERRO ao carregar definições de pinos: ', ME.message]);
    disp('Por favor, execute setupPins.m antes de prosseguir.');
    return;
end

% --- RECRIAR O OBJETO ARDUINO NESTE SCRIPT ---
a = []; % Inicializa 'a' como vazio
try
    % Substitua 'COMx' pela porta serial da sua ESP32
    a = arduino("COMx", "ESP32-WROOM-32-DevKitV1"); % Adapte "COMx" e o nome da placa se necessário.
    disp('Conexão com ESP32 estabelecida para este script!');
    % Para calibração, não precisamos configurar PWM, apenas ler ADC.

catch ME
    disp(['ERRO ao reconectar à ESP32: ', ME.message]);
    disp('Verifique se a placa está conectada e a porta COM correta.');
    return; % Sai do script se não conseguir conectar
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

disp('--- CALIBRAÇÃO CONCLUÍDA ---');

% --- NO FINAL DO SCRIPT, SEMPRE LIMPE O OBJETO 'a' ---
clear a;
disp('Conexão com a ESP32 fechada para este script.');
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
    a = arduino("COMx", "ESP32-WROOM-32-DevKitV1");
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
```

-----

### Script 4: `runPIDControl.m`

Este script implementa o controle PID em tempo real. Os ganhos `Kp`, `Ki`, `Kd` devem ser ajustados após a identificação do sistema e uso do *PID Tuner*.

```matlab
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
    a = arduino("COMx", "ESP32-WROOM-32-DevKitV1");
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
    % Você pode implementar aqui um mecanismo para pedir um novo setpoint
    % sem parar o loop principal, por exemplo, lendo do teclado de forma não bloqueante
    % (requer funções mais avançadas do MATLAB ou um timer separado).
    % Por simplicidade, para mudar o setpoint ou parar, você terá que pressionar Ctrl+C
    % na Command Window e rodar o script novamente, ou implementar um loop de setpoint
    % aninhado com leitura não-bloqueante no início.

    % Exemplo de como você poderia permitir mudança de setpoint ou saída (requer mais lógica)
    % if keyboard_input_detected
    %    handle_input();
    % end
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
```

-----

### Fluxo de Trabalho Passo a Passo com os Novos Scripts:

1.  **Execute `setupPins.m`:**

      * Abra o MATLAB.
      * Salve `setupPins.m` em sua pasta de trabalho.
      * Edite `setupPins.m` para definir a porta `COM` correta da sua ESP32.
      * Execute `setupPins.m` (clique em Run ou digite `setupPins` na Command Window). Isso criará a conexão temporariamente e salvará `arduinoPins.mat` com os nomes dos pinos.

2.  **Execute `calibratePotentiometer.m`:**

      * Salve `calibratePotentiometer.m` na mesma pasta.
      * **Edite a linha `angleRangeDegrees = 1800;`** para refletir a faixa angular **real** que o seu motor (com o potenciômetro de 10 voltas acoplado) pode girar. Este é um passo **CRÍTICO**.
      * Execute `calibratePotenciometer.m`. Siga as instruções no console para mover o motor manualmente e registrar as posições min/max. Isso salvará `potentiometerCalibration.mat`.

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
      * Execute `runPIDControl.m`. O script pedirá o ângulo desejado. Insira um valor e observe o motor se mover para a posição. O script continuará rodando, mantendo a posição e reagindo a perturbações, até que você pressione **Ctrl+C** na Command Window do MATLAB para pará-lo.

```
---
```