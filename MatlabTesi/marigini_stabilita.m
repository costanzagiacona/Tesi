%% ANALISI COMPLETA STABILITÀ VTOL
% Architettura: 
% - Hover: Inner (Roll, Pitch, Yaw) / Outer (Z, X, Y)
% - Cruise: Inner (Roll, Pitch, Yaw) / Outer (Z, Vx)

clear all; close all; clc;

%% 1. PARAMETRI DEL SISTEMA
m = 6.7;      % massa UAV VTOL (Kg)  
g = 9.8;    % costante gravitazionele (m/s^2)
k = 7*(10^-5);      % coeff. thrust rotori
rho = 1.225;    % densità dell'aria (kg/m^3)
ala_y = 0.4;
ala_x = 0.15;
s = ala_x * ala_y;      % superficie alare (m^2) 
v_air = 25;   % velocità relativa alla superficie alare (m/s) lungo x
v_limite = 13.89; % -> 50km/h
b = 0.005*k; % rapporto tra momento torcente e forza del rotore
Ixx = 0.237;
Iyy = 0.244;
Izz = 0.468;

% Geometria (per Cruise)
d_ty = 0;  % assumo il tail rotor posto sull'asse X_body
d_tz = 0;

d_my = (1/2)*ala_y;
d_mz = 0;

d_mx = 0.6;
d_tx = -1.2;

T_hover = m*g;      % Spinta totale hovering/cruise

% Attuatori (Motori/Servi)
wn = 2*pi*15;       % 94.2 rad/s
zeta = 0.9;
s = tf('s');
G_act = wn^2 / (s^2 + 2*zeta*wn*s + wn^2);

%% ==========================================
%% DEFINIZIONE LOOP (HOVER)
%% ==========================================

% --- HOVER INNER (Assetto) ---
% 1. Roll (PD)
Kp_h_phi = 40; Kd_h_phi = 8;
C_h_phi = Kp_h_phi + Kd_h_phi*s;
L_h_phi = C_h_phi * G_act * (1/(Ixx*s^2));
W_cl_phi = L_h_phi / (1 + L_h_phi); 

% 2. Pitch (PD)
Kp_h_th = 40; Kd_h_th = 8; 
C_h_th = Kp_h_th + Kd_h_th*s;
L_h_th = C_h_th * G_act * (1/(Iyy*s^2));
W_cl_th = L_h_th / (1 + L_h_th);    

% 3. Yaw (PD)
Kp_h_psi = 15; Kd_h_psi = 5;
C_h_psi = Kp_h_psi + Kd_h_psi*s;
L_h_psi = C_h_psi * G_act * (1/(Izz*s^2));

% --- HOVER OUTER (Posizione) ---
% 4. Quota Z (PD eq)
lam_z=2.5; K_z=60; Phi_z=0.8;
Kp_eq_z = K_z*lam_z/Phi_z; Kd_eq_z = m*lam_z + K_z/Phi_z;
C_h_z = Kp_eq_z + Kd_eq_z*s;
L_h_z = C_h_z * G_act * (1/(m*s^2));

% 5. Posizione X (PD eq)
lam_x=5; K_x=15; Phi_x=1.0; 
Kp_eq_x = K_x*lam_x/Phi_x; Kd_eq_x = m*lam_x + K_x/Phi_x;
C_h_x = Kp_eq_x + Kd_eq_x*s; 
L_h_x = C_h_x * (1/m) * W_cl_th * (1/s^2); 

% 6. Posizione Y (PD eq)
lam_y=7; K_y=20; Phi_y=3.0;
Kp_eq_y = K_y*lam_y/Phi_y; Kd_eq_y = m*lam_y + K_y/Phi_y;
C_h_y = Kp_eq_y + Kd_eq_y*s;
L_h_y = C_h_y * (1/m) * W_cl_phi * (1/s^2);

%% ==========================================
%% DEFINIZIONE LOOP (CRUISE)
%% ==========================================

% --- CRUISE INNER (Assetto) ---
% 7. Roll (PD)
Kp_c_phi = 4.0; Kd_c_phi = 0.8;
C_c_phi = Kp_c_phi + Kd_c_phi*s;
Gain_geo_phi = (T_hover * d_my) / Ixx;
L_c_phi = C_c_phi * G_act * (Gain_geo_phi / s^2);

% 8. Pitch (PD)
Kp_c_th = 2.0; Kd_c_th = 0.5;
C_c_th = Kp_c_th + Kd_c_th*s;
Gain_geo_th = (T_hover * d_mx) / Iyy;
L_c_th = C_c_th * G_act * (Gain_geo_th / s^2);
W_cl_c_th = L_c_th / (1 + L_c_th);

% 9. Yaw (PD)
Kp_c_psi = 3.0; Kd_c_psi = 1.2;
C_c_psi = Kp_c_psi + Kd_c_psi*s;
Gain_geo_psi = (T_hover * d_my) / Izz;
L_c_psi = C_c_psi * G_act * (Gain_geo_psi / s^2);

% --- CRUISE OUTER (Navigazione) ---
% 10. Quota Z (PD)
Kp_c_z = 4.0; Kd_c_z = 6.0;
C_c_z = Kp_c_z + Kd_c_z*s;
L_c_z = C_c_z * G_act * (1/(m*s^2));

% 11. Velocità Vx (PI)
Kp_c_v = 8.0; Ki_c_v = 4.0;
C_c_v = Kp_c_v + Ki_c_v/s;
L_c_v = C_c_v * W_cl_c_th * (g/s);

%% ==========================================
%% PLOTTING (4 FIGURE)
%% ==========================================

% --- FIGURA 1: HOVER INNER (Assetto) ---
figure('Name', '1. HOVER - INNER LOOPS', 'Color', 'w', 'Position', [50, 50, 1000, 600]);
loops = {L_h_phi, L_h_th, L_h_psi};
titles = {'Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)'};
for i=1:3
    subplot(1, 3, i);
    [Gm, Pm, Wcg, Wcp] = margin(loops{i});
    if isnan(Wcp); Wcp=0; end
    margin(loops{i}, 'b'); grid on;
    title(sprintf('%s\nBW: %.1f rad/s, PM: %.1f deg', titles{i}, Wcp, Pm), 'FontSize', 11, 'FontWeight', 'bold');
end
sgtitle('HOVER: Anelli Interni (Assetto)', 'FontSize', 14);

% --- FIGURA 2: HOVER OUTER (Posizione) ---
figure('Name', '2. HOVER - OUTER LOOPS', 'Color', 'w', 'Position', [100, 100, 1000, 600]);
loops = {L_h_z, L_h_x, L_h_y};
titles = {'Quota (Z)', 'Longitudinale (X)', 'Laterale (Y)'};
for i=1:3
    subplot(1, 3, i);
    [Gm, Pm, Wcg, Wcp] = margin(loops{i});
    if isnan(Wcp); Wcp=0; end
    margin(loops{i}, 'r'); grid on;
    title(sprintf('%s\nBW: %.1f rad/s, PM: %.1f deg', titles{i}, Wcp, Pm), 'FontSize', 11, 'FontWeight', 'bold');
end
sgtitle('HOVER: Anelli Esterni (Posizione)', 'FontSize', 14);

% --- FIGURA 3: CRUISE INNER (Assetto) ---
figure('Name', '3. CRUISE - INNER LOOPS', 'Color', 'w', 'Position', [150, 150, 1000, 600]);
loops = {L_c_phi, L_c_th, L_c_psi};
titles = {'Roll Cruise (\phi)', 'Pitch Cruise (\theta)', 'Yaw Cruise (\psi)'};
for i=1:3
    subplot(1, 3, i);
    [Gm, Pm, Wcg, Wcp] = margin(loops{i});
    if isnan(Wcp); Wcp=0; end
    margin(loops{i}, 'g'); grid on;
    title(sprintf('%s\nBW: %.1f rad/s, PM: %.1f deg', titles{i}, Wcp, Pm), 'FontSize', 11, 'FontWeight', 'bold');
end
sgtitle('CRUISE: Anelli Interni (Assetto)', 'FontSize', 14);

% --- FIGURA 4: CRUISE OUTER (Navigazione) ---
figure('Name', '4. CRUISE - OUTER LOOPS', 'Color', 'w', 'Position', [200, 200, 800, 600]);
loops = {L_c_z, L_c_v};
titles = {'Quota Cruise (Z)', 'Velocità (Vx)'};
for i=1:2
    subplot(1, 2, i);
    [Gm, Pm, Wcg, Wcp] = margin(loops{i});
    if isnan(Wcp); Wcp=0; end
    margin(loops{i}, 'm'); grid on;
    title(sprintf('%s\nBW: %.1f rad/s, PM: %.1f deg', titles{i}, Wcp, Pm), 'FontSize', 11, 'FontWeight', 'bold');
end
sgtitle('CRUISE: Anelli Esterni (Navigazione)', 'FontSize', 14);

fprintf('Generazione completata: 4 Figure create.\n');