%% SIMULAZIONE VTOL - SCRIPT PRINCIPALE (VERSIONE CORRETTA E RISTRUTTURATA)

% =========================================================================
% 1. PULIZIA AMBIENTE
% =========================================================================
% Queste righe sono FONDAMENTALI per assicurare che ogni simulazione
% parta da condizioni pulite, resettando anche i controllori.
clear all;
close all;
clc;
clear functions;


% =========================================================================
% 2. DEFINIZIONE DEI PARAMETRI FISICI
% =========================================================================

% parametri VTOL 

% (roll , pitch , yaw) = (phi , theta , psi)

m = 6;      % massa UAV VTOL (Kg)  % NB -> se si cambia questo parametro cambia la nostra stima di C_d e C_l
g = 9.8;    % costante gravitazionele (m/s^2)

k = 7*(10^-5);      % coeff. thrust rotori

%matrice inerzia del corpo (UAV) rispetto al body frame (kg*m^2)
Ixx = 3;
Iyy = 0.5 * Ixx;
Izz = 1.4 * Ixx;

I_body = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];     

%inerzie dei tre rotori : wing_dx , wing_sx , tail (kg*m^2)

I_rotor_xx = 2.5;
I_rotor_yy = 0.5*I_rotor_xx;
I_rotor_zz = 1.4*I_rotor_xx;

I_rotor = (10^-3)*[I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];  

I_rotor_w_dx = I_rotor;
I_rotor_w_sx = I_rotor;
I_rotor_tail = I_rotor;


rho = 1.225;    % densità dell'aria (kg/m^3)
ala_y = 0.4;
ala_x = 0.15;
s = ala_x * ala_y;      % superficie alare (m^2) 

v_air = 25;   % velocità relativa alla superficie alare (m/s) lungo x
v_limite = 13.89; % -> 50km/h

C_d = ((m-1)*g)/(rho*s*(v_air)^2); %1.28;  %coeff. di resistenza (drag) aerodinamica lungo asse X

% scelto in modo tale che se v_x = 90 km/h (25 m/s) la portanza contrasti
% la gravità
C_l = (m*g)/(rho*s*(v_air)^2); %0.854; %coeff. di portanza (lift) aerodinamica 

% scelto in modo tale che sia raggiunta la velocità limite di 50 km/h
% (13.89 m/s) in caduta libera
C_d_z = (m*g)/(rho*s*(v_limite)^2); % coeff. di resistenza (drag) aerodinamica lungo asse z
%C_d_z = 1.2;

%distanza (m) tra centro di massa e rotori (per il calcolo del momento dei thrust)
%lungo i vari assi (X_body , Y_body, Z_body)

d_mx = 0; % assumo il wing rotor posto sull'asse Y_body
d_my = (1/2)*ala_y;
d_mz = 0;

d_tx = 0.3;
d_ty = 0;  % assumo il tail rotor posto sull'asse X_body
d_tz = 0;

b = 0.01; % rapporto tra momento torcente e forza del rotore

%parametri distanza (m) tra centro di massa e forze aerodinamiche (per il calcolo del momento delle forze aerodinamiche)

l_w_dx_x = 0;%(1/3)*ala_x; % distanza per l'ala dx da centro di massa lungo asse X_body
l_w_dx_y = (1/2)*ala_y; % distanza per l'ala dx da centro di massa lungo asse Y_body
l_w_dx_z = 0; % distanza per l'ala dx da centro di massa lungo asse Z_body

l_w_sx_x = 0;%(1/3)*ala_x; % distanza per l'ala sx da centro di massa lungo asse X_body
l_w_sx_y = -(1/2)*ala_y; % distanza per l'ala sx da centro di massa lungo asse Y_body
l_w_sx_z = 0; % distanza per l'ala sx da centro di massa lungo asse Z_body


% Struttura con tutti i parametri

% Parametri generali
parametri.m = m;
parametri.g = g;
parametri.k = k;

% Inerzia corpo
parametri.Ixx = Ixx;
parametri.Iyy = Iyy;
parametri.Izz = Izz;
parametri.I_body = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

% Inerzia rotore con Huygens - Steiner
m_rotor = 0.1;
I_rotor_w_dx = I_rotor + m_rotor*[d_mx^2 0 0;0 d_my^2 0;0 0 d_mz^2];
I_rotor_w_sx = I_rotor + m_rotor*[d_mx^2 0 0;0 d_my^2 0;0 0 d_mz^2];
I_rotor_tail = I_rotor + m_rotor*[d_tx^2 0 0;0 d_ty^2 0;0 0 d_tz^2];

parametri.I_rotor_xx = I_rotor_xx;
parametri.I_rotor_yy = I_rotor_yy;
parametri.I_rotor_zz = I_rotor_zz;
parametri.I_rotor = [I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];

% uso questi
parametri.I_rotor_w_dx = I_rotor_w_dx;
parametri.I_rotor_w_sx = I_rotor_w_sx;
parametri.I_rotor_tail = I_rotor_tail;

% Parametri aerodinamici
parametri.rho = rho;
parametri.s = s;
parametri.C_d = C_d;
parametri.C_d_z = C_d_z;
parametri.C_l = C_l;
parametri.b = b;
parametri.v_air = v_air;

% Parametri ala
parametri.ala_x = ala_x;
parametri.ala_y = ala_y;

% Distanze momento (rotori/forze)
parametri.d_mx = d_mx;
parametri.d_my = d_my;
parametri.d_mz = d_mz;

% Distanze traslazione
parametri.d_tx = d_tx;
parametri.d_ty = d_ty;
parametri.d_tz = d_tz;

% Distanze ala destra
parametri.l_w_dx_x = l_w_dx_x;
parametri.l_w_dx_y = l_w_dx_y;
parametri.l_w_dx_z = l_w_dx_z;

% Distanze ala sinistra
parametri.l_w_sx_x = l_w_sx_x;
parametri.l_w_sx_y = l_w_sx_y;
parametri.l_w_sx_z = l_w_sx_z;
parametri.b = b;

parametri.r_th_w_dx = [d_mx ;  d_my ;  d_mz];
parametri.r_th_w_sx = [d_mx ; -d_my ;  d_mz]; % considero rotore ala dx e sx in posizione simmetrica
parametri.r_th_tail = [d_tx ;  d_ty ;  d_tz];

parametri.l_w_dx = [l_w_dx_x; l_w_dx_y; l_w_dx_z];
parametri.l_w_sx = [l_w_sx_x; l_w_sx_y; l_w_sx_z];

parametri.r_aerodyn_w_dx = parametri.l_w_dx;
parametri.r_aerodyn_w_sx = parametri.l_w_sx;

% =========================================================================
% 4. CONDIZIONI INIZIALI E SETUP SIMULAZIONE
% =========================================================================
tspan = [0 20];      % Intervallo di simulazione [s]
x0 = zeros(26,1);    % Vettore di stato iniziale (tutto a zero)

% Condizioni iniziali specifiche
x0(3) = 0;           % Quota iniziale (z=0)

% Impostazione dei tilt iniziali per il decollo verticale
x0(13)= pi/2;        % theta_1 (rotore dx)
x0(15)= pi/2;        % theta_2 (rotore sx)
x0(17)= pi/2;        % theta_3 (rotore coda, asse principale)
x0(19)= 0;           % theta_4 (rotore coda, per yaw)

% Velocità angolari iniziali dei rotori (tutte a zero)
x0(4) = 0; 
x4eq = x0(4);
omega1_2 = (0.0) * ((parametri.rho * parametri.s * parametri.C_d * (x4eq)^2) / parametri.k);
omega2_2 = (0.0) * ((parametri.rho * parametri.s * parametri.C_d * (x4eq)^2) / parametri.k);
omega3_2 = 0;        

x0(21) = sqrt(omega1_2);
x0(23) = sqrt(omega2_2);
x0(25) = sqrt(omega3_2);

% Opzioni del solutore
options = odeset('RelTol',1e-3, 'AbsTol',1e-6);


% =========================================================================
% 5. ESECUZIONE DELLA SIMULAZIONE
% =========================================================================
fprintf('Avvio simulazione ODE45...\n');
[t, x] = ode45(@(t, x) simulazioneVTOL3(t, x, parametri), tspan, x0, options);
fprintf('Simulazione completata.\n');


% =========================================================================
% 6. POST-PROCESSING E PLOTTING
% =========================================================================
fprintf('Elaborazione e plotting dei risultati...\n');

% Per plottare i comandi, rieseguiamo il controllore sui risultati ottenuti
% NOTA: Assicurati che il nome della funzione sia quello corretto!
% Se il tuo controllore si chiama 'controlloVTOL_v3', usa quello.
U_values = zeros(length(t), 7);
for k = 1:length(t)
    U_values(k,:) = controlloVTOL_v3(t(k), parametri, x(k,:)); 
end

% Estrazione delle variabili di stato per i plot
xp = x(:,1);
yp = x(:,2);
zp = -1 * x(:,3); % Asse z positivo verso l'alto per i grafici

xv = x(:,4);
yv = x(:,5);
zv = -1 * x(:,6);

phi = x(:,7);
theta = x(:,8);
psi = x(:,9);

p = x(:,10);
q = x(:,11);
r = x(:,12);

omega_1 = x(:,21);
omega_2 = x(:,23);
omega_3 = x(:,25);

theta1 = x(:,13);
theta2 = x(:,15);
theta3 = x(:,17);
theta4 = x(:,19);

% --- Plot Principali (Stile Tesi) ---
figure(1)
set(gcf, 'Name', 'Risultati Principali di Volo', 'Position', [100 100 1000 800])
% Velocità lungo x
subplot(3,1,1);
plot(t, xv, 'r', 'LineWidth', 2); hold on;
yline(25, '--k', 'v_{x,des}', 'LabelHorizontalAlignment', 'left', 'FontSize', 12, 'LineWidth', 2);
grid on; ylim([-10 40]);
xlabel('Time [s]'); ylabel('v_x [m/s]');
title('Andamento Velocità');
legend('v_x', 'Location', 'best');
% Velocità lungo z
subplot(3,1,2);
plot(t, zv, 'b', 'LineWidth', 2); hold on;
yline(0, '--k', 'v_{z,des}', 'LabelHorizontalAlignment', 'left', 'FontSize', 12, 'LineWidth', 2);
grid on; ylim([-20 20]);
xlabel('Time [s]'); ylabel('v_z [m/s]');
legend('v_z', 'Location', 'best');
% Quota z
subplot(3,1,3);
plot(t, zp, 'g', 'LineWidth', 2); hold on;
yline(10, '--k', 'z_{des}', 'LabelHorizontalAlignment', 'left', 'FontSize', 12, 'LineWidth', 2);
grid on; ylim([-15 15]);
xlabel('Time [s]'); ylabel('Quota z [m]');
title('Andamento Quota');
legend('z', 'Location', 'best');


% --- Plot Velocità Angolari Rotori ---
figure(2)
set(gcf, 'Name', 'Velocità Rotori', 'Position', [150 150 1000 800])
subplot(3,1,1);
plot(t, omega_1, 'r', 'LineWidth', 2);
grid on; ylabel('[rad/s]');
title('Andamento Velocità Angolare dei Rotori');
legend('\omega_{1}', 'Location', 'best');
subplot(3,1,2);
plot(t, omega_2, 'b', 'LineWidth', 2);
grid on; ylabel('[rad/s]');
legend('\omega_{2}', 'Location', 'best');
subplot(3,1,3);
plot(t, omega_3, 'g', 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('[rad/s]');
legend('\omega_{3}', 'Location', 'best');


% --- Plot Angoli di Tilt ---
figure(3)
set(gcf, 'Name', 'Angoli di Tilt', 'Position', [200 200 1200 900])
subplot(4,1,1);
plot(t, theta1, 'r', 'LineWidth', 2);
ylim([-pi pi]); grid on; ylabel('[rad]');
title('Andamento Angoli di Tilt dei Rotori');
legend('\theta_1 (dx)', 'Location', 'best');
subplot(4,1,2);
plot(t, theta2, 'b', 'LineWidth', 2);
ylim([-pi pi]); grid on; ylabel('[rad]');
legend('\theta_2 (sx)', 'Location', 'best');
subplot(4,1,3);
plot(t, theta3, 'g', 'LineWidth', 2);
ylim([-pi pi]); grid on; ylabel('[rad]');
legend('\theta_3 (coda, tilt principale)', 'Location', 'best');
subplot(4,1,4);
plot(t, theta4, 'k', 'LineWidth', 2);
ylim([-pi pi]); grid on; xlabel('Time [s]'); ylabel('[rad]');
legend('\theta_4 (coda, per yaw)', 'Location', 'best');


fprintf('Plotting completato.\n');