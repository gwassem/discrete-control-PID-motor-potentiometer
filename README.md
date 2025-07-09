# Controle de Posição de Motor DC com ESP32 e MATLAB (Versão Atualizada)

Este repositório contém os scripts MATLAB e a documentação para o desenvolvimento de um sistema de controle de posição de motor DC utilizando uma placa ESP32 como hardware e o MATLAB para lógica de controle, aquisição de dados e ajuste PID. Esta documentação reflete as versões mais recentes dos scripts fornecidos.

## Visão Geral do Projeto

O objetivo principal deste projeto é implementar um controlador de posição de malha fechada para um motor DC, utilizando um potenciômetro para feedback de posição e a ESP32 para interface com o hardware. O processo envolve as seguintes etapas:

1.  **Configuração Inicial:** Definição e configuração dos pinos da ESP32.
2.  **Calibração do Sensor:** Mapeamento da leitura do potenciômetro (tensão ADC) para valores angulares (graus).
3.  **Coleta de Resposta ao Degrau PWM:** Aquisição de dados para identificação do sistema do motor.
4.  **Controle P Básico (Baseline):** Implementação de um controlador proporcional simples para estabelecer uma linha de base de desempenho.
5.  **Controle PID Completo:** Implementação final do controlador PID com ganhos ajustados e funcionalidade de anti-windup.
6.  **Rotação do Motor até Limites:** Script para movimentação controlada do motor até seus limites físicos ou interrupção manual.

## Pré-requisitos

Antes de começar, certifique-se de ter os seguintes softwares e hardwares instalados e configurados:

* **MATLAB:** Versão 2020b ou superior (recomendado).
* **Pacotes de Suporte do MATLAB para Hardware:** `MATLAB Support Package for Arduino Hardware` e `MATLAB Support Package for ESP32 Hardware`.
* **Drivers da ESP32:** Certifique-se de que os drivers da sua placa ESP32 (e.g., CP210x USB to UART Bridge VCP Drivers) estão instalados e a placa é reconhecida em uma porta COM (Windows) ou `/dev/ttyUSBx`, `/dev/cu.usbserial-xxxx` (Linux/macOS).
* **Hardware:**
    * Placa ESP32 (e.g., ESP32-WROOM-DevKitV1)
    * Motor DC
    * Driver de Motor DC (e.g., L298N)
    * Potenciômetro de 10 voltas ou similar para feedback de posição
    * Fonte de alimentação adequada

## Scripts MATLAB

A seguir, uma descrição detalhada de cada script:

### `1. setupPins.m`

* **Propósito:** Este script é responsável por estabelecer a conexão inicial com a placa ESP32 e configurar os pinos digitais designados para o potenciômetro e os controles do motor (PWM). Ele define as variáveis `potPin`, `motorPin1`, e `motorPin2` e as salva no arquivo `arduinoPins.mat` para uso por outros scripts.
* **Ação:** Cria e configura o objeto `arduino` temporariamente, configurando os pinos do motor como `PWM`.
* **Saída Salva:** `arduinoPins.mat` (contém `potPin`, `motorPin1`, `motorPin2`).

### `2. calibratePotentiometer.m`

* **Propósito:** Realiza a calibração do potenciômetro usado para feedback de posição, mapeando as leituras de tensão do ADC para valores angulares em graus.
* **Pré-requisitos:** Executar `setupPins.m` primeiro para carregar as definições de pino.
* **Ação:** Guia o usuário a girar o potenciômetro/motor para as posições mínima (0 graus) e máxima (e.g., 3600 graus para 10 voltas), registrando as tensões correspondentes e calculando a relação `Volts por Grau`.
* **Saída Salva:** `potentiometerCalibration.mat` (contém `minVoltage`, `maxVoltage`, `angleRangeDegrees`, `voltsPerDegree`).

### `3. collectPWMStepResponse.m`

* **Propósito:** Coleta dados de resposta do motor a uma sequência de três degraus complexos de PWM. Os dados coletados são essenciais para a identificação do sistema no MATLAB System Identification Toolbox.
* **Pré-requisitos:** `setupPins.m` e `calibratePotentiometer.m` devem ser executados previamente.
* **Ação:** Move o motor para uma posição inicial segura (aproximadamente o centro do range) e então aplica uma sequência de degraus de PWM, registrando o tempo, a posição atual e o duty cycle aplicado.
* **Saída Salva:** `multiStepResponseData.mat` (contém `timeData`, `positionData`, `inputDutyCycle`).

### `4. simplePController.m`

* **Propósito:** Implementa um controlador de posição Proporcional (P) simples para o motor DC, servindo como linha de base para o controle de posição.
* **Pré-requisitos:** `setupPins.m` e `calibratePotentiometer.m` devem ser executados antes.
* **Ação:** Solicita uma posição angular desejada ao usuário e tenta alcançá-la usando apenas um ganho proporcional (`Kp`). Inclui proteção de fins de curso (5% dos extremos calibrados) para operação segura.
* **Parâmetros Configuráveis:** `Kp`, `pwmMax`, `pwmMinAtivacao`, `erroZonaMorta`.
* **Características:** Proteção de fins de curso e detecção de "alvo atingido" dentro de uma zona morta.
    * **Fórmula do Erro:** `erro = Posição Desejada - Posição Atual`
        * **Justificativa:** O erro é a diferença entre o que se deseja alcançar (setpoint) e o que se tem atualmente (medida do sensor). Um erro positivo indica que a posição atual está aquém da desejada, e um erro negativo indica que a posição atual ultrapassou a desejada.
    * **Fórmula do PWM Proporcional:** `pwmOutput = Kp * erro`
        * **Justificativa:** A saída do controlador proporcional é diretamente proporcional ao erro. Quanto maior o erro, maior o PWM aplicado (até o limite de `pwmMax`), o que faz com que o motor se mova mais rapidamente em direção ao setpoint.
    * **Fórmula de Saturação do PWM:** `pwmToApply = min(abs(pwmOutput), pwmMax)`
        * **Justificativa:** Garante que o sinal de controle (PWM) não exceda os limites físicos do motor (0 a 100% duty cycle ou 0 a `pwmMax`). A função `abs()` é usada porque o sentido de rotação é definido separadamente.

### `5. ControllerPID.m`

* **Propósito:** Este script implementa um controlador de posição PID completo para o motor DC. É a versão aprimorada do controlador de posição.
* **Pré-requisitos:** `setupPins.m` e `calibratePotentiometer.m` devem ser executados.
* **Ação:** Pede a posição angular desejada ao usuário e utiliza os termos Proporcional (`Kp`), Integral (`Ki`) e Derivativo (`Kd`) para calcular o sinal de controle (PWM).
* **Parâmetros Configuráveis:** `Kp`, `Ki`, `Kd`, `pwmMax`, `pwmMinAtivacao`, `erroZonaMorta`. Os valores de `Kp`, `Ki`, `Kd` devem ser ajustados (preferencialmente via PID Tuner do MATLAB) para otimização.
* **Características Implementadas:**
    * **Controle PID:** Cálculo dos termos Proporcional, Integral e Derivativo.
    * **Anti-Windup:** Implementação de uma lógica de anti-windup para o termo integral (limitando seu acúmulo quando o PWM está saturado).
    * **Limitação de PWM:** Assegura que o motor opere dentro dos limites de segurança e operacionais.
    * **Proteção de Fins de Curso:** Para garantir a operação segura dentro dos limites do motor.
    * **Fórmula do Erro:** `erro = posicaoDesejada - posicaoAtual`
        * **Justificativa:** Similar ao controlador P, o erro é a diferença entre o setpoint e a medição atual da posição.
    * **Cálculo dos Termos PID:**
        * **Termo Proporcional (P):** `termoP = Kp * erro`
            * **Justificativa:** Responde instantaneamente ao erro atual. Um `Kp` maior resulta em uma resposta mais rápida, mas pode causar instabilidade.
        * **Termo Integral (I):** `integralErro = integralErro + erro * samplePeriod` (atualizado condicionalmente) e `termoI = Ki * integralErro`
            * **Justificativa:** Acumula o erro ao longo do tempo para eliminar o erro em regime permanente (steady-state error). `Ki` determina a taxa de acumulação do erro integral.
        * **Termo Derivativo (D):** `derivadaErro = (erro - erroAnterior) / samplePeriod` e `termoD = Kd * derivadaErro`
            * **Justificativa:** Responde à taxa de variação do erro, ajudando a amortecer oscilações e melhorar a resposta a distúrbios. `Kd` influencia a intensidade dessa ação de amortecimento.
    * **Saída Bruta do PID:** `rawPidOutput = termoP + Ki * integralErro + termoD`
        * **Justificativa:** Representa o sinal de controle que o controlador PID deseja aplicar antes de qualquer saturação.
    * **Anti-Windup por Clamping:** O termo integral (`integralErro`) é atualizado **apenas** se a saída bruta do PID (`rawPidOutput`) não estiver saturada (dentro de `+-pwmMax`), ou se a saturação e o erro ainda "concordam" na direção (ajudando o sistema a sair da saturação).
        * `if (rawPidOutput < pwmMax && rawPidOutput > -pwmMax) || (rawPidOutput >= pwmMax && erro > 0) || (rawPidOutput <= -pwmMax && erro < 0)`
        * `integralErro = integralErro + erro * samplePeriod;`
        * **Justificativa:** O anti-windup impede que o termo integral acumule um erro excessivo quando o atuador (motor) atinge seus limites de saturação (ex: PWM máximo/mínimo). Sem isso, o termo integral continuaria crescendo desnecessariamente, causando um overshoot maior e um tempo de acomodação prolongado quando o sistema tentasse retornar ao setpoint.
    * **Fórmula de Saturação do PWM:** `pwmToApply = min(abs(pidOutput), pwmMax)`
        * **Justificativa:** Garante que o sinal de controle final aplicado ao motor esteja dentro dos limites operacionais seguros, prevenindo a sobrecarga do hardware.

### `6. rotateMotorToLimit.m`

* **Propósito:** Permite girar o motor continuamente em uma direção específica (horário ou anti-horário) até que ele atinja um dos limites de rotação predefinidos (5% dos extremos calibrados) ou seja interrompido manualmente pelo usuário.
* **Pré-requisitos:** `setupPins.m` e `calibratePotentiometer.m` devem ser executados previamente.
* **Ação:** Solicita a direção e a potência (PWM) desejada, verifica a posição inicial do motor e, em seguida, aplica o PWM para girar o motor até um dos limites, exibindo a tensão e o ângulo em tempo real.
* **Características:** Detecção de limites de rotação, opção de escolha da direção e interrupção manual (Ctrl+C).