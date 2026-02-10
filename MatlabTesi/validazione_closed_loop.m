%% VALIDAZIONE STABILITÀ CLOSED LOOP (RIGOROSA)
clear all; clc; close all;

% 1. Caricamento e Trim
mainTest2; % Carica parametri
load('trim_data.mat'); % Carica x_trim, u_trim_val trovati dallo script precedente
close all;
params = parametri;

% Definizione del Target coerente col Trim
target_cruise = [25, theta_trim, -10]; % [Vx, Theta, Z]
test_id = 3; % Crociera

fprintf('Analisi di Stabilità attorno al punto di Trim (V=25 m/s)...\n');

% Parametri per differenze finite
n_x = 30; % Numero stati
n_u = 7;  % Numero ingressi
eps_x = 1e-5;
eps_u = 1e-5;

%% 2. CALCOLO MATRICE B (PLANT INPUT JACOBIAN)
% A l'abbiamo già (o la ricalcoliamo al volo per sicurezza)
% B = df/du

fprintf('1. Linearizzazione del PLANT (Calcolo A e B)...\n');
A = zeros(n_x, n_x);
B = zeros(n_x, n_u);

% Funzione wrapper per il plant (Usa U_FORCE tramite global)
% Assicurati che simulazioneVTOL3 abbia la modifica "global U_FORCE_TRIM"
global U_FORCE_TRIM

f0 = get_dynamics(x_trim, u_trim_val, params);

% --- Calcolo A (df/dx) ---
for i = 1:n_x
    x_pert = x_trim; x_pert(i) = x_pert(i) + eps_x;
    f_p = get_dynamics(x_pert, u_trim_val, params);
    x_pert = x_trim; x_pert(i) = x_pert(i) - eps_x;
    f_m = get_dynamics(x_pert, u_trim_val, params);
    A(:,i) = (f_p - f_m) / (2*eps_x);
end

% --- Calcolo B (df/du) ---
for j = 1:n_u
    u_pert = u_trim_val; u_pert(j) = u_pert(j) + eps_u;
    f_p = get_dynamics(x_trim, u_pert, params);
    u_pert = u_trim_val; u_pert(j) = u_pert(j) - eps_u;
    f_m = get_dynamics(x_trim, u_pert, params);
    B(:,j) = (f_p - f_m) / (2*eps_u);
end

%% 3. CALCOLO MATRICE K_eq (CONTROLLER JACOBIAN)
% Linearizziamo la funzione 'controlloVTOL_v3'
% K_eq = du_controllo / dx
% Questo ci dice come il tuo codice reagisce a variazioni di stato

fprintf('2. Linearizzazione del CONTROLLORE (Calcolo K_eq)...\n');
K_eq = zeros(n_u, n_x);

% Output nominale del controllore (dovrebbe essere vicino a u_trim)
u_ctl_0 = controlloVTOL_v3(0, params, x_trim, test_id, target_cruise);

for i = 1:n_x
    x_pert = x_trim; x_pert(i) = x_pert(i) + eps_x;
    u_p = controlloVTOL_v3(0, params, x_pert, test_id, target_cruise);
    
    x_pert = x_trim; x_pert(i) = x_pert(i) - eps_x;
    u_m = controlloVTOL_v3(0, params, x_pert, test_id, target_cruise);
    
    K_eq(:,i) = (u_p - u_m) / (2*eps_x);
end

%% 4. CHIUSURA DEL LOOP E ANALISI
% Dinamica Closed Loop: dx = A*x + B*u
% Controllo: u = K_eq * x (Linearizzato attorno al trim)
% Totale: dx = (A + B*K_eq) * x

A_CL = A + B * K_eq;

eigs_CL = eig(A_CL);

% Plot Autovalori
figure('Name', 'Autovalori Closed Loop');
plot(real(eigs_CL), imag(eigs_CL), 'bx', 'LineWidth', 2, 'MarkerSize', 8);
hold on; grid on; xline(0, 'k--');
xlabel('Reale (\sigma)'); ylabel('Immaginario (j\omega)');
title('Mappa Poli Closed Loop (Sistema Completo)');

% Analisi Risultati
instabili = eigs_CL(real(eigs_CL) > 1e-5);
fprintf('\n=== RISULTATI ANALISI ===\n');
if isempty(instabili)
    fprintf('SUCCESS: Il sistema Closed Loop è ASINTOTICAMENTE STABILE.\n');
    min_damping = min(-real(eigs_CL)./abs(eigs_CL));
    fprintf('Margine di stabilità (Smorzamento minimo): %.3f\n', min_damping);
else
    fprintf('WARNING: Trovati %d autovalori instabili.\n', length(instabili));
    disp(instabili);
    fprintf('Suggerimento: Controlla i guadagni o la dinamica a zero.\n');
end

% --- Funzione Helper per pulizia codice ---
function dx = get_dynamics(x, u, p)
    global U_FORCE_TRIM
    U_FORCE_TRIM = u; % Forza l'input in simulazioneVTOL3
    dx = simulazioneVTOL3(0, x, p, 0, 0, [0 0 0], 0);
end