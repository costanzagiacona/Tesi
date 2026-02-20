%% ANALISI DI STABILITÀ LINEARE VTOL - REPORT FINALE
clc; clear; close all;
mainTest2; % Carica i parametri e la struct 'parametri'
close all; % Chiude i grafici aperti da mainTest2

% Tolleranza numerica per gli autovalori (per distinguere lo zero reale dal rumore)
tol = 1e-8;

%% =========================================================================
% CASE 1: HOVERING (VOLO VERTICALE) - 26 STATI
% =========================================================================
fprintf('--- ANALISI CASO 1: HOVERING ---\n');

% 1. Definizione Punto di Equilibrio x_eq
x_eq_hover = zeros(26,1);
x_eq_hover(3) = -10; % Quota desiderata
x_eq_hover(13) = pi/2; % Tilt servi anteriori (Verticale)
x_eq_hover(15) = pi/2;
theta3_ideal = atan2(((-parametri.d_tx * parametri.k) / parametri.b), 1);
x_eq_hover(17) = theta3_ideal; % Tilt coda
x_eq_hover(19) = -pi/2;

% Calcolo spinte di Trim
Thrust_tot_req = parametri.m * parametri.g;
F_tail_z = (parametri.d_mx * Thrust_tot_req) / (parametri.d_mx - parametri.d_tx);
F_front_tot_z = Thrust_tot_req - F_tail_z;

x_eq_hover(21) = sqrt(F_front_tot_z / (2 * parametri.k)); % omega_1
x_eq_hover(23) = x_eq_hover(21);                          % omega_2
x_eq_hover(25) = sqrt(F_tail_z / (parametri.k * sin(theta3_ideal))); % omega_3

% 2. Linearizzazione
A_hover = linearizzaVTOL_ClosedLoop(x_eq_hover, parametri, 1, target);
autov_hover = eig(A_hover);

% 3. Verifica Stabilità
re_hover = real(autov_hover);
if any(re_hover > tol)
    status_hover = 'SISTEMA INSTABILE';
elseif all(re_hover < -tol)
    status_hover = 'SISTEMA ASINTOTICAMENTE STABILE';
else
    status_hover = 'SISTEMA SEMPLICEMENTE STABILE (Marginale)';
end

fprintf('Stato: %s\n', status_hover);
fprintf('Polo più critico (Max Re): %.4f\n\n', max(re_hover));
fprintf('Autovalori:\n');
disp(autov_hover);

%% =========================================================================
% CASE 2: CRUISE (VOLO ORIZZONTALE) - 30 STATI
% =========================================================================
fprintf('--- ANALISI CASO 2: CRUISE (25 m/s) ---\n');

% 1. Definizione Punto di Equilibrio x_eq_cruise
x_eq_cruise = zeros(30, 1); 
x_eq_cruise(3) = -10;
x_eq_cruise(4) = 25;  % Velocità di avanzamento
x_eq_cruise(13) = 0;  % Tilt servi anteriori (Orizzontale)
x_eq_cruise(15) = 0;

% Equilibrio Forze (Thrust = Drag)
F_drag_eq = 0.5 * parametri.rho * (parametri.s * parametri.C_d + parametri.s_body_x * parametri.C_d_x) * 25^2;
omega_front_eq = sqrt((F_drag_eq / 2) / parametri.k);
x_eq_cruise(21) = omega_front_eq;
x_eq_cruise(23) = omega_front_eq;

% 2. Linearizzazione
A_cruise = linearizzaVTOL_ClosedLoop(x_eq_cruise, parametri, 2, target);
autov_cruise = eig(A_cruise);

% 3. Verifica Stabilità
re_cruise = real(autov_cruise);
if any(re_cruise > tol)
    status_cruise = 'SISTEMA INSTABILE';
elseif all(re_cruise < -tol)
    status_cruise = 'SISTEMA ASINTOTICAMENTE STABILE';
else
    status_cruise = 'SISTEMA SEMPLICEMENTE STABILE (Marginale)';
end

fprintf('Stato: %s\n', status_cruise);
fprintf('Polo più critico (Max Re): %.4f\n\n', max(re_cruise));
fprintf('Autovalori:\n');
disp(autov_cruise);

%% =========================================================================
% GRAFICI DI CONFRONTO
% =========================================================================
figure('Name', 'Analisi Spettrale VTOL', 'Position', [100, 100, 1000, 500]);

subplot(1,2,1);
plot(real(autov_hover), imag(autov_hover), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
grid on; xline(0, '--k');
title(['Hovering: ', status_hover]);
xlabel('Re'); ylabel('Im');

subplot(1,2,2);
plot(real(autov_cruise), imag(autov_cruise), 'bd', 'MarkerSize', 8, 'LineWidth', 1.5);
grid on; xline(0, '--k');
title(['Cruise: ', status_cruise]);
xlabel('Re'); ylabel('Im');