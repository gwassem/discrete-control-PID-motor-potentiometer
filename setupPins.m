% setupPins.m
% Script para configurar a conexão com a ESP32 e definir os modos dos pinos.

clc; clear all; close all; % Limpa o console, workspace e fecha figuras
disp('--- CONFIGURANDO CONEXÃO E PINOS ---');

% Defina os pinos que serão usados (serão salvos e carregados por outros scripts)
global potPin motorPin1 motorPin2; % Declara como global para acesso em outros scripts
potPin = "D35"; % Pino do potenciômetro (ADC)
motorPin1 = "D25"; % Pino de controle do motor (PWM)
motorPin2 = "D26"; % Pino de controle do motor (PWM)

% Tenta criar o objeto arduino para verificar a conexão e configurar os pinos
% Este objeto 'a' VAI SER CRIADO NOVAMENTE em outros scripts.
a = []; % Inicializa 'a' como vazio
try
    % Substitua 'COMx' pela porta serial da sua ESP32
    a = arduino("COM9", "ESP32-WROOM-DevKitV1");
    disp('Conexão com ESP32 estabelecida com sucesso para configuração inicial!');

    % Configure o modo dos pinos
    configurePin(a, motorPin1, "PWM");
    configurePin(a, motorPin2, "PWM");
    disp('Pinos configurados no hardware.');

    % Fecha a conexão imediatamente após a configuração, pois não pode ser salva.
    clear a;
    disp('Conexão temporária com ESP32 fechada. Será reaberta em outros scripts.');

catch ME
    disp(['ERRO ao conectar à ESP32: ', ME.message]);
    disp('Verifique se a placa está conectada, os drivers instalados e a porta COM correta.');
    return; % Sai do script se não conseguir conectar
end


disp('Pinos definidos para uso futuro:');
disp(['  Potenciômetro: ', potPin]);
disp(['  Motor IN1: ', motorPin1]);
disp(['  Motor IN2: ', motorPin2]);

% Salva APENAS as variáveis de pino (strings) em um arquivo .mat
% O objeto 'a' não será salvo.
save('arduinoPins.mat', 'potPin', 'motorPin1', 'motorPin2');
disp('Definições de pinos salvas em arduinoPins.mat.');

disp('--- CONFIGURAÇÃO CONCLUÍDA ---');