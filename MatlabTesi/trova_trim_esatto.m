%% SCRIPT PER IL CALCOLO DEL TRIM COMPLETO (6 DOF)
% Trova: Phi, Theta, Psi, Omega, Tilt
clear all; clc; close all;

% 1. Carica Parametri
mainTest2; 
close all;

% 2. Impostazioni Trim
V_target = 25;    % m/s
H_target = -10;   % m

fprintf('Calcolo Trim 6-DOF per V=%.1f m/s...\n', V_target);

% 3. Configurazione Solver (5 Variabili)
% z = [phi, theta, psi, omega, tilt]
phi_guess   = 0;
theta_guess = deg2rad(2);
psi_guess   = 0;
omega_guess = 300;
tilt_guess  = 0;

x0_solver = [phi_guess, theta_guess, psi_guess, omega_guess, tilt_guess];

options = optimset('Display', 'iter', 'TolFun', 1e-9, 'TolX', 1e-9, 'MaxFunEvals', 5000);

% 4. Ottimizzazione
funzione_costo = @(z) cost_trim_6dof(z, parametri, V_target, H_target);
[z_opt, fval] = fminsearch(funzione_costo, x0_solver, options);

% 5. Estrazione Risultati
phi_trim   = z_opt(1);
theta_trim = z_opt(2);
psi_trim   = z_opt(3);
omega_trim = z_opt(4);
tilt_trim  = z_opt(5);

fprintf('\n=== RISULTATI TRIM 6-DOF ===\n');
fprintf('Roll  (Phi)   : %8.4f deg\n', rad2deg(phi_trim));
fprintf('Pitch (Theta) : %8.4f deg\n', rad2deg(theta_trim));
fprintf('Yaw   (Psi)   : %8.4f deg\n', rad2deg(psi_trim));
fprintf('Motori (Omega): %8.2f rad/s\n', omega_trim);
fprintf('Tilt          : %8.4f deg\n', rad2deg(tilt_trim));
fprintf('Residuo Costo : %.2e\n', fval);

% 6. Costruzione x_trim Completo (con Rotazione 3D)
x_trim = zeros(30, 1);
x_trim(1:3) = [0; 0; H_target];

% Calcolo velocità body esatte usando la matrice di rotazione inversa
R = get_rotation_matrix(phi_trim, theta_trim, psi_trim);
V_ned = [V_target; 0; 0];
V_body = R' * V_ned; % R_trasposta porta da NED a Body

x_trim(4) = V_body(1); % u
x_trim(5) = V_body(2); % v (sideslip implicito)
x_trim(6) = V_body(3); % w

% Angoli
x_trim(7) = phi_trim;
x_trim(8) = theta_trim;
x_trim(9) = psi_trim;

% Ratei (fermi)
x_trim(10:12) = 0; 

% Attuatori (Posizioni trim, velocità zero)
x_trim(13) = tilt_trim; x_trim(15) = tilt_trim; 
x_trim(14) = 0; x_trim(16) = 0; x_trim(18) = 0; x_trim(20) = 0;

x_trim(21) = omega_trim; x_trim(23) = omega_trim;
x_trim(22) = 0; x_trim(24) = 0; x_trim(26) = 0;

x_trim(27:30) = 0;

% Salva
u_trim_val = [omega_trim; omega_trim; 0; tilt_trim; tilt_trim; 0; 0];
save('trim_data_6dof.mat', 'x_trim', 'u_trim_val', 'z_opt');
fprintf('Salvato in trim_data_6dof.mat\n');

global U_FORCE_TRIM; U_FORCE_TRIM = [];

%% --- FUNZIONI NESTED ---

function J = cost_trim_6dof(z, params, V_req, H_req)
    % Decodifica
    phi = z(1); th = z(2); psi = z(3);
    om = z(4); ti = z(5);
    
    % Constraints
    if om < 0; J=1e9; return; end
    
    % Calcolo V_body coerente con gli angoli candidati
    R = get_rotation_matrix(phi, th, psi);
    V_b = R' * [V_req; 0; 0];
    
    % Stato temporaneo
    x = zeros(30,1);
    x(3) = H_req;
    x(4) = V_b(1); x(5) = V_b(2); x(6) = V_b(3);
    x(7) = phi;    x(8) = th;     x(9) = psi;
    
    x(13) = ti; x(15) = ti;
    x(21) = om; x(23) = om;
    
    % Input forzato
    u_val = zeros(7,1);
    u_val(1) = om; u_val(2) = om;
    u_val(4) = ti; u_val(5) = ti;
    
    global U_FORCE_TRIM
    U_FORCE_TRIM = u_val;
    
    dx = simulazioneVTOL3(0, x, params, 0, 0, [0 0 0], 0);
    
    % COSTO: Minimizziamo TUTTE le accelerazioni (lineari e angolari)
    % Se c'è asimmetria, dx(5) (v_dot) o dx(10) (p_dot) non sarebbero zero
    % se gli angoli fossero sbagliati.
    
    acc_lin = dx(4)^2 + dx(5)^2 + dx(6)^2;   % u_dot, v_dot, w_dot
    acc_ang = dx(10)^2 + dx(11)^2 + dx(12)^2; % p_dot, q_dot, r_dot
    
    % Pesa molto i momenti per garantire equilibrio rotazionale
    J = 10 * acc_lin + 1000 * acc_ang;
end

function R = get_rotation_matrix(phi, theta, psi)
    % Matrice ZYX standard (NED -> Body se usata diretta, qui serve Body->NED)
    % R = Rz(psi) * Ry(theta) * Rx(phi)
    
    cph = cos(phi); sph = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cps = cos(psi); sps = sin(psi);
    
    R = [ cth*cps,   sph*sth*cps - cph*sps,   cph*sth*cps + sph*sps;
          cth*sps,   sph*sth*sps + cph*cps,   cph*sth*sps - sph*cps;
         -sth,       sph*cth,                 cph*cth ];
end