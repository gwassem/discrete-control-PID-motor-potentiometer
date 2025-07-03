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