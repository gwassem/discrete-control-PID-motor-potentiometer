# Controle de Posição de Motor DC com ESP32 e MATLAB

Este repositório contém os scripts MATLAB e a documentação para o desenvolvimento de um sistema de controle de posição de motor DC utilizando uma placa ESP32 como hardware e o MATLAB para lógica de controle, aquisição de dados e ajuste PID.

## Visão Geral do Projeto

O objetivo principal deste projeto é implementar um controlador de posição de malha fechada para um motor DC, utilizando um potenciômetro para feedback de posição e a ESP32 para interface com o hardware. O processo envolve as seguintes etapas:

1.  **Configuração Inicial:** Definição e configuração dos pinos da ESP32.
2.  **Calibração do Sensor:** Mapeamento da leitura do potenciômetro (tensão ADC) para valores angulares (graus).
3.  **Identificação do Sistema:** Coleta de dados de resposta ao degrau de PWM para modelar a dinâmica do motor.
4.  **Controle P Básico (Baseline):** Implementação de um controlador proporcional simples para estabelecer uma linha de base de desempenho.
5.  **Ajuste PID:** Utilização do System Identification Toolbox e PID Tuner do MATLAB para otimizar os ganhos PID.
6.  **Controle PID Completo:** Implementação final do controlador PID com os ganhos ajustados e anti-windup.

## Pré-requisitos

Antes de começar, certifique-se de ter os seguintes softwares e hardwares instalados e configurados:

* **MATLAB:** Versão 2020b ou superior (recomendado).
* **Pacotes de Suporte do MATLAB para Hardware:**
    * `MATLAB Support Package for Arduino Hardware` (incluindo suporte para ESP32).
* **Firmware `server_esp32.ino` na ESP32:** Certifique-se de que sua ESP32 esteja programada com o firmware `server_esp32.ino` (ou equivalente) que permite a comunicação serial com o MATLAB.
* **Hardware:**
    * Placa ESP32-WROOM-DevKitV1 (ou similar).
    * Motor DC.
    * Potenciômetro de múltiplas voltas (ou similar para feedback de posição).
    * Driver de motor (e.g., L298N, ponte H).
    * Fonte de alimentação adequada para o motor.
    * Fios de conexão.

## Estrutura do Repositório

* `setupPins.m`: Script para definir os pinos do hardware e testar a conexão inicial.
* `calibratePotentiometer.m`: Script para calibrar o potenciômetro e mapear a tensão para graus.
* `collectPWMStepResponse.m`: Script para coletar dados de resposta ao degrau de PWM, usados para identificação do sistema.
* `basicPControl.m`: Script para implementar um controlador Proporcional (P) básico, com limites de PWM.
* `fullPIDControl.m`: Script para implementar o controlador PID completo, com os ganhos otimizados e anti-windup robusto.
* `arduinoPins.mat`: Arquivo gerado por `setupPins.m` contendo as definições dos pinos.
* `potentiometerCalibration.mat`: Arquivo gerado por `calibratePotentiometer.m` contendo os parâmetros de calibração do potenciômetro.
* `stepResponseData.mat`: Arquivo gerado por `collectPWMStepResponse.m` contendo os dados para identificação do sistema.
* `basicPControlResponse.mat`: Arquivo gerado por `basicPControl.m` contendo a resposta do controle P para comparação.
* `fullPIDControlResponse.mat`: Arquivo gerado por `fullPIDControl.m` contendo a resposta do controle PID ajustado.

## Guia de Uso

Siga a ordem dos scripts para desenvolver e testar seu sistema de controle.

### 1. Configuração dos Pinos (`setupPins.m`)

Este script define quais pinos da ESP32 serão usados para o potenciômetro e o motor, e verifica a conexão inicial.

```matlab
% Executar este script primeiro
setupPins.m

Observações:

    Ajuste a porta COM ("COM9") e o modelo da sua ESP32 ("ESP32-WROOM-DevKitV1") no script, se necessário.

    As variáveis de pino (potPin, motorPin1, motorPin2) são salvas em arduinoPins.mat para serem carregadas pelos scripts seguintes.

2. Calibração do Potenciômetro (calibratePotentiometer.m)

Este script calibra o potenciômetro, definindo as tensões mínima e máxima que correspondem aos 0 graus e ao ângulo máximo de rotação (e.g., 3600 graus para 10 voltas).
Matlab

% Executar após setupPins.m
calibratePotentiometer.m

Instruções durante a execução:

    O script irá guiá-lo para posicionar o motor/potenciômetro na posição mínima (0 graus) e máxima (e.g., 3600 graus) para registrar as tensões correspondentes.

    Os parâmetros de calibração são salvos em potentiometerCalibration.mat.

3. Coleta de Resposta ao Degrau (collectPWMStepResponse.m)

Este script aplica um degrau de PWM ao motor e coleta os dados de posição ao longo do tempo. Esses dados são cruciais para a identificação do modelo do seu sistema.
Matlab

% Executar após calibratePotentiometer.m
collectPWMStepResponse.m

Observações:

    O sampleRate está definido para 0.02 segundos (50 Hz), o que proporciona uma boa resolução para a identificação do sistema.

    Os dados de tempo, posição e duty cycle de entrada são salvos em stepResponseData.mat.

4. Identificação do Sistema e Ajuste PID

Após coletar os dados de resposta ao degrau, você usará o MATLAB para identificar um modelo do seu sistema e ajustar os ganhos PID.

A. Identificação do Modelo (System Identification Toolbox)

    Abra o System Identification Toolbox no MATLAB digitando ident na linha de comando.

    Importe os dados de stepResponseData.mat. Selecione timeData como tempo, positionData como saída e inputDutyCycle como entrada.

    Utilize as ferramentas do Toolbox para identificar um modelo que represente a dinâmica do seu motor (e.g., um modelo de função de transferência de 1ª ou 2ª ordem).

B. Ajuste dos Ganhos PID (PID Tuner)

    Uma vez que você tenha um modelo identificado (ex: G = tf(...)), abra o PID Tuner:
    Matlab

    pidTuner(G)

    No PID Tuner, você pode ajustar interativamente os ganhos Kp, Ki e Kd para alcançar o desempenho desejado (velocidade, overshoot, erro em regime permanente).

    Anote os valores finais de Kp, Ki e Kd que você determinar como ideais. Eles serão usados no próximo script.

5. Controle P Básico (basicPControl.m)

Este script implementa um controlador Proporcional simples. Ele serve como uma linha de base para comparação com o desempenho do controle PID otimizado.
Matlab

% Executar após calibrar e antes de testar o PID completo
basicPControl.m

Observações:

    Ele inclui as limitações de PWM (min 15%, max 60%).

    Você precisará ajustar o Kp inicial para obter uma resposta razoável.

    Os dados de desempenho são salvos em basicPControlResponse.mat.

6. Controle PID Completo (fullPIDControl.m)

Este é o script final que implementa o controlador PID completo, incluindo a lógica de anti-windup.
Matlab

% Executar após obter os ganhos PID do PID Tuner
fullPIDControl.m

Passo Crítico:

    Antes de executar: Edite o script fullPIDControl.m e substitua os valores de exemplo de Kp, Ki e Kd pelos valores otimizados que você obteve do "PID Tuner".

Matlab

% --- PARÂMETROS DO CONTROLADOR PID ---
% ESTES VALORES SERÃO POPULADOS COM OS RESULTADOS DO PID TUNER
Kp = SEU_VALOR_DE_KP;   % Ganho Proporcional
Ki = SEU_VALOR_DE_KI;   % Ganho Integral
Kd = SEU_VALOR_DE_KD;  % Ganho Derivativo

Características Implementadas:

    Controle PID: Cálculo dos termos Proporcional, Integral e Derivativo.

    Anti-Windup Robusto: A lógica do termo integral foi aprimorada para evitar o acúmulo excessivo quando o atuador está saturado. Isso ajuda a reduzir o overshoot e melhorar o tempo de assentamento.

    Limitação de PWM: Assegura que o motor opere dentro dos limites de segurança e operacionais (15% a 60% de duty cycle).

Os dados de desempenho do controlador PID completo serão salvos em fullPIDControlResponse.mat.

Análise e Comparação

Após executar basicPControl.m e fullPIDControl.m, você terá os arquivos .mat com os dados de resposta. Você pode carregá-los no MATLAB e plotar as curvas de posição e PWM para comparar visualmente o desempenho do controle P básico com o controle PID otimizado. Isso demonstrará as melhorias alcançadas com o ajuste PID.