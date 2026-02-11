%% CALCOLO MATRICI A_CL (CLOSED LOOP) PER LINEARIZZAZIONE VTOL
% Basato sui parametri di 'mainTest2.m' e logica di 'controlloVTOL_v3.m' (Case 1)

clear all; clc;

% --- 1. PARAMETRI FISICI (Dal tuo file mainTest2.m) ---
m = 6.7;        % Massa [kg]
g = 9.81;       % Gravità [m/s^2]
Ixx = 0.237;    % Inerzia Roll [kg*m^2]
Iyy = 0.244;    % Inerzia Pitch [kg*m^2] (Assunto simile a Ixx se non specificato diversamente)

% Coefficienti di smorzamento aerodinamico linearizzato (stimati a v=0)
% In hovering puro sono 0, ma la fusoliera offre resistenza appena ci si muove.
% Dal tuo codice: C_d_y = 3 (molto alto), C_d_x = 0.1
% D = 0.5 * rho * S * Cd * v^2 -> Linearizzato D_lin = rho * S * Cd * v_eq
% A v=0 il termine è nullo, ma per stabilità robusta consideriamo un piccolo smorzamento
Du = 0.1; % Smorzamento X (basso)
Dv = 2.0; % Smorzamento Y (alto, vista la fusoliera laterale)

% --- 2. PARAMETRI CONTROLLORE SMC (Dal tuo file controlloVTOL_v3.m - Case 1) ---

% QUOTA (Z)
lambda_z = 1.5; K_z = 20; Phi_z = 1.5;
% Guadagni Equivalenti PD Linearizzati per SMC: u = Kp*e + Kd*de
% Formula: Kp_eq = (K * lambda) / Phi
% Formula: Kd_eq = lambda + (K / Phi)
kp_z_eq = (K_z * lambda_z) / Phi_z;
kd_z_eq = m * lambda_z + (K_z / Phi_z); % Nota: nel codice c'era un termine misto con massa


% LONGITUDINALE (X -> Pitch)
% Outer Loop (Posizione X)
lambda_x = 0.8; K_x = 8; Phi_x = 2.5;
kp_x_outer = (K_x * lambda_x) / Phi_x;      % Guadagno P su posizione
kd_x_outer = lambda_x + (K_x / Phi_x);      % Guadagno D su velocità

% Inner Loop (Pitch Attitude)
lambda_att = 12.0; K_att = 20.0; Phi_att = 1.8;
kp_theta_inner = (K_att * lambda_att) / Phi_att;
kd_theta_inner = lambda_att + (K_att / Phi_att);

% LATERALE (Y -> Roll)
% Outer Loop (Posizione Y)
lambda_y = 0.8; K_y = 10; Phi_y = 2.5;
kp_y_outer = (K_y * lambda_y) / Phi_y;
kd_y_outer = lambda_y + (K_y / Phi_y);

% Inner Loop (Roll Attitude) - Usiamo gli stessi parametri di assetto del pitch
kp_phi_inner = kp_theta_inner;
kd_phi_inner = kd_theta_inner;


%% --- 3. COSTRUZIONE MATRICI ---

% =========================================================================
% SOTTOSISTEMA 1: VERTICALE (z, vz)
% Stati: x = [z; vz]
% Input: u = Thrust (positivo verso l'alto nel codice, ma z è giù)
% =========================================================================

% Matrice A (Open Loop)
% z_dot = vz
% vz_dot = g - T/m. Linearizzato: delta_vz_dot = -1/m * delta_T
A_vert = [0, 1;
          0, 0];

B_vert = [0;
         -1/m]; 

% Matrice K (Feedback)
% u = -K*x. Il controllo SMC fa u = Kp_eq * (z_des - z) + ...
% Quindi K = [kp_eq, kd_eq]
K_vert = [-kp_z_eq, -kd_z_eq];

% A Closed Loop
A_cl_vert = A_vert - B_vert * K_vert;


% =========================================================================
% SOTTOSISTEMA 2: LONGITUDINALE (x, u, theta, q)
% Stati: x = [x; u; theta; q]
% Input: u = My (Momento Pitch)
% =========================================================================

% Matrice A (Open Loop)
% u_dot = -g * theta (approssimazione piccoli angoli)
A_lon = [0,  1,     0,   0;
         0, -Du/m, -g,   0;
         0,  0,     0,   1;
         0,  0,     0,   0];

B_lon = [0; 0; 0; 1/Iyy];

% Matrice K (Cascaded Feedback)
% Il controllo è a cascata. Dobbiamo "schiacciare" tutto in un'unica K row vector.
% Logica: 
% 1. Outer loop genera theta_des = -(kp_x*x + kd_x*u) / g  (Normalizzato per gravità)
%    (Nota: il divisore 'g' serve perché theta si trasforma in accelerazione g*theta)
% 2. Inner loop genera My = kp_theta*(theta_des - theta) - kd_theta*q
% 3. Sostituendo: My = kp_theta*(-(kp_x/g)*x - (kd_x/g)*u - theta) - kd_theta*q

K_tot_x = (kp_theta_inner * kp_x_outer) / (m*g); % Nota: fattore massa se il loop esterno calcola Forza
K_tot_u = (kp_theta_inner * kd_x_outer) / (m*g);
K_tot_theta = kp_theta_inner;
K_tot_q = kd_theta_inner;

% ATTENZIONE: Nel tuo codice SMC, l'outer loop calcola una FORZA (F_x_req), non direttamente l'angolo.
% F_x_req = m * ...
% sin_theta_des = -F_x_req / Thrust_req (~mg).
% Quindi il fattore di conversione è corretto.

K_lon = [-K_tot_x, -K_tot_u, K_tot_theta, K_tot_q];

% A Closed Loop
A_cl_lon = A_lon - B_lon * K_lon;


% =========================================================================
% SOTTOSISTEMA 3: LATERALE (y, v, phi, p)
% Stati: x = [y; v; phi; p]
% Input: u = Mx (Momento Roll)
% =========================================================================

% Matrice A (Open Loop)
% v_dot = +g * phi (Segno positivo per convenzione NED laterale)
A_lat = [0,  1,     0,   0;
         0, -Dv/m,  g,   0;
         0,  0,     0,   1;
         0,  0,     0,   0];

B_lat = [0; 0; 0; 1/Ixx];

% Matrice K (Cascaded Feedback)
% Simmetrico al longitudinale, ma occhio ai segni.
% Outer loop: F_y_req -> sin_phi_des = F_y_req / mg.
% F_y_req calcolato positivo per correggere errore positivo.
% Se y > 0 (errore negativo), F_y < 0, phi < 0 -> v_dot < 0. Corretto.

K_tot_y   = (kp_phi_inner * kp_y_outer) / (m*g);
K_tot_v   = (kp_phi_inner * kd_y_outer) / (m*g);
K_tot_phi = kp_phi_inner;
K_tot_p   = kd_phi_inner;

K_lat = [K_tot_y, K_tot_v, K_tot_phi, K_tot_p];

% A Closed Loop
A_cl_lat = A_lat - B_lat * K_lat;


%% --- 4. OUTPUT RISULTATI ---
fprintf('=== ANALISI AUTOVALORI CLOSED LOOP (CASE 1 SMC) ===\n\n');

fprintf('--- VERTICALE (z, vz) ---\n');
disp('A_cl_vert ='); disp(A_cl_vert);
fprintf('Autovalori: '); disp(eig(A_cl_vert)');

fprintf('\n--- LONGITUDINALE (x, u, theta, q) ---\n');
disp('A_cl_lon ='); disp(A_cl_lon);
fprintf('Autovalori: '); disp(eig(A_cl_lon)');

fprintf('\n--- LATERALE (y, v, phi, p) ---\n');
disp('A_cl_lat ='); disp(A_cl_lat);
fprintf('Autovalori: '); disp(eig(A_cl_lat)');

%% =========================================================================
% SOTTOSISTEMA 4: IMBARDATA (psi, r)
% Stati: x = [psi; r]
% Input: u = Mz (Momento Yaw)
% =========================================================================

% --- Parametri Specifici ---
Izz = 0.468;  % Inerzia asse Z [kg*m^2] (dal file mainTest2.m)
Dr_yaw = 1.0; % Coefficiente di smorzamento dalla "pinna" (alpha0z in simulazioneVTOL3)

% --- Parametri Controllore SMC Yaw (Case 1) ---
% Dal codice:
% s_psi = de_psi + (lambda_att * 0.7) * e_psi; 
% M_req = (lambda_att * 0.7) * de_psi + (K_att * 0.8) * tanh(...)

% Fattori di scala usati nel codice per ridurre l'aggressività su Yaw
scale_lambda = 0.7;
scale_K = 0.8;

lambda_psi = lambda_att * scale_lambda;
K_psi_smc = K_att * scale_K;
Phi_psi = Phi_att; % Usa lo stesso Phi del roll/pitch

% Guadagni Equivalenti PD Linearizzati
kp_psi = (K_psi_smc * lambda_psi) / Phi_psi;
kd_psi = lambda_psi + (K_psi_smc / Phi_psi);


% --- Costruzione Matrici ---

% Matrice A (Open Loop)
% psi_dot = r
% r_dot = (-Dr * r + Mz) / Izz
A_yaw = [0,       1;
         0, -Dr_yaw/Izz];

B_yaw = [0;
         1/Izz];

% Matrice K (Feedback)
% u = -K*x -> Mz = -kp*psi - kd*r
K_yaw = [kp_psi, kd_psi];

% A Closed Loop
A_cl_yaw = A_yaw - B_yaw * K_yaw;


% --- Output Risultati ---
fprintf('\n--- IMBARDATA (psi, r) ---\n');
fprintf('Smorzamento naturale (Dr): %.2f\n', Dr_yaw);
disp('A_cl_yaw ='); disp(A_cl_yaw);
fprintf('Autovalori: '); disp(eig(A_cl_yaw)');