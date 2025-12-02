function x_dot = simulazioneVTOL4(t, x, params)

%% 1. INIZIALIZZAZIONE E DEBUG
paramFlag = 0;
vento = 0; % Metti a 1 per testare raffiche di vento

% Estrazione Stati (Rinomina per leggibilità)
% Posizione: x(1-3), Velocità: x(4-6), Angoli: x(7-9), Omega: x(10-12)
phi = x(7);
theta = x(8);
psi = x(9);
p = x(10);
q = x(11);
r = x(12); 

V_body = [x(4); x(5); x(6)]; % velocità nel body frame
Omega_body = [p; q; r];      % velocità angolare body

% === CHIAMATA AL CONTROLLORE ===
u = controlloVTOL_v3(params, x);

%% 2. DINAMICA ATTUATORI (Motori e Servi)
% Simuliamo il ritardo fisico dei motori e dei servocomandi
zeta = 0.8;
omega_n = 2*pi*2;             % Dinamica servi tilt anteriori
zeta_tail = 0.8;
omega_n_tail = 2*pi*4;        % Dinamica servo coda
zeta_rotor = 0.9;
omega_n_rotor = 2*pi*15;      % Dinamica motori (rpm)

% Riferimenti dal controllore
omega1_ref = u(1); omega2_ref = u(2); omega3_ref = u(3);
theta1_des = u(4); theta2_des = u(5); theta3_des = u(6); theta4_des = u(7);

% Equazioni differenziali attuatori (Stati x13 -> x26)
% Tilt Anteriori
dx13 = x(14);
dx14 = -2*zeta*omega_n*x(14) - (x(13)-theta1_des)*omega_n^2;  
dx15 = x(16);
dx16 = -2*zeta*omega_n*x(16) - (x(15)-theta2_des)*omega_n^2;  
% Tilt Coda
dx17 = x(18);
dx18 = -2*zeta_tail*omega_n_tail*x(18) - (x(17)-theta3_des)*omega_n_tail^2;  
dx19 = x(20);
dx20 = -2*zeta_tail*omega_n_tail*x(20) - (x(19)-theta4_des)*omega_n_tail^2; 
% Motori (Velocità angolare eliche)
dx21 = x(22);
dx22 = -2*zeta_rotor*omega_n_rotor*x(22) - (x(21)-omega1_ref)*omega_n_rotor^2; 
dx23 = x(24);
dx24 = -2*zeta_rotor*omega_n_rotor*x(24) - (x(23)-omega2_ref)*omega_n_rotor^2; 
dx25 = x(26);
dx26 = -2*zeta_rotor*omega_n_rotor*x(26) - (x(25)-omega3_ref)*omega_n_rotor^2; 

%% 3. CINEMATICA
R = matriceRotazione(phi, theta, psi); % Body -> Inertial
J = matriceJ(phi, theta, psi);         % Body Rates -> Euler Rates

%% 4. FORZE (BODY FRAME)

% A) GRAVITÀ
F_g_body = F_grav(phi, theta, psi, params.m, params.g);

% B) THRUST (SPINTA MOTORI ATTIVA)
% Stato attuale dei motori (non il desiderato)
input_thrust = [params.k*x(21)^2; params.k*x(23)^2; params.k*x(25)^2; x(13); x(15); x(17); x(19)];
F_th_body = F_thrust(input_thrust); 

% C) AERODINAMICA ALA PRINCIPALE (DINAMICA)
% Calcoliamo Lift e Drag in base all'angolo di attacco reale (Alpha)

% 1. Calcolo Alpha Ala Principale
% Alpha = atan(w/u) -> velocità verticale relativa / velocità orizzontale relativa
alpha_wing = atan2(x(6), x(4)); 

% 2. Coefficiente di Portanza Ala Principale (Linearizzato)
% Cl = 2*pi * (alpha + incidenza + pitch_effettivo)
i_wing = deg2rad(2); % Angolo di calettamento ala (incidenza costruttiva)
Cl_dynamic = 2 * pi * (alpha_wing + i_wing); % Modello lastra piana semplice

% Saturazione stallo (fisica reale)
Cl_dynamic = max(-1.3, min(Cl_dynamic, 1.3));

% 3. Calcolo Forze (Wind Frame)
v_sq_main = x(4)^2 + x(6)^2;
if v_sq_main < 0.1; v_sq_main = 0; end % Evita rumore a fermo

Lift_wing = 0.5 * params.rho * v_sq_main * params.s * Cl_dynamic;
Drag_wing = 0.5 * params.rho * v_sq_main * params.s * params.C_d; 

% 4. Rotazione Forze: Wind Frame -> Body Frame
% Fx = -Drag*cos(alpha) + Lift*sin(alpha)
% Fz = -Drag*sin(alpha) - Lift*cos(alpha)
Fx_aero = -Drag_wing * cos(alpha_wing) + Lift_wing * sin(alpha_wing);
Fz_aero = -Drag_wing * sin(alpha_wing) - Lift_wing * cos(alpha_wing);

% Aggiungiamo Drag parassita della fusoliera
F_drag_body_x = -0.5 * params.rho * abs(x(4))*x(4) * params.s_body_x * params.C_d_x;
F_drag_body_z = -0.5 * params.rho * abs(x(6))*x(6) * params.s_body_z * params.C_d_z;

F_aero_tot = [Fx_aero + F_drag_body_x; 0; Fz_aero + F_drag_body_z];

% D) CORIOLIS
F_cor = F_Coriolis(Omega_body, V_body, params.m);

% E) FORZE TOTALI
F_tot_body = F_g_body + F_th_body + F_aero_tot - F_cor;

% F) DISTURBO VENTO (Opzionale)
if vento == 1 && t > 5
    F_vento_global = [0; 0; 30]; % Raffica in Giù
    F_vento_body = R' * F_vento_global;
    F_tot_body = F_tot_body + F_vento_body;
end

%% 5. MOMENTI (BODY FRAME)

% A) MOMENTO GIROSCOPICO CORPO
M_gyro_body = MomentGyroBody(params.I_body, Omega_body);

% B) MOMENTI THRUST (Spinta differenziale e bracci di leva)
Iz1 = params.I_rotor_w_dx(3,3);
Iz2 = params.I_rotor_w_sx(3,3);
Iz3 = params.I_rotor_tail(3,3);
M_th = M_thrust_2(input_thrust, params.r_th_w_dx, params.r_th_w_sx, params.r_th_tail, ...
                  params.b, params.k, x(22), x(24), x(26), Iz1, Iz2, Iz3);
                  
% C) MOMENTI AERODINAMICI ALA PRINCIPALE (Offset Lift/Drag)
% Se il centro aerodinamico non coincide col centro di massa, si crea momento.
% Approssimiamo usando la funzione MomentAero esistente ma coi nuovi dati
% (O più semplicemente: Assumiamo Ala bilanciata e usiamo la coda per stabilizzare)
% Per ora manteniamo la tua funzione MomentAero ma attenzione che usa Cl statico params.C_l
% PROVIAMO A AZZERARLO per affidarci alla fisica esplicita della coda
M_aero_wing = [0;0;0]; 
% Se vuoi precisione: M_aero_wing = [0; Lift_wing * arm_wing; 0]; ma serve braccio ala.

% D) EFFETTI GIROSCOPICI TILT ROTORI
Omega_rotor_w_dx = [0; x(14); 0];
Omega_rotor_w_sx = [0; x(16); 0];
Omega_rotor_tail = [0; x(18); x(20)];
input_thrust_gyro = [x(21); x(23); x(25); x(13); x(15); x(17); x(19)];
M_gyro_tilt = M_tilt_rotor(input_thrust_gyro, params.I_rotor_w_dx, params.I_rotor_w_sx, params.I_rotor_tail, ...
                           Omega_rotor_w_dx, Omega_rotor_w_sx, Omega_rotor_tail);

% --- E) FISICA STABILIZZATORE DI CODA (CRUCIALE PER VOLO ORIZZONTALE) ---
% Questa sezione simula la coda come una superficie aerodinamica passiva.

% 1. Parametri Coda
s_tail = params.s * 0.25;      % Area coda (stimata 25% ala)
d_tail = params.r_th_tail(1);  % Braccio di leva (negativo, es -1.2m)

% 2. Velocità relativa alla coda
v_sq_tail = x(4)^2 + x(6)^2;        

% 3. Angolo di incidenza locale (Alpha Coda)
% Alpha visto dalla coda + Pitch del corpo + Angolo di calettamento (Trim)
% IMPORTANTE: i_calettamento regola l'equilibrio di salita/discesa.
% Valore positivo = Coda genera portanza su -> Muso giù -> Meno salita.
i_calettamento = deg2rad(2.5); 

alpha_tail = atan2(x(6), x(4)) + x(8) + i_calettamento;

% 4. Coefficiente di Portanza Coda
Cl_tail = 2.0 * pi * alpha_tail;
Cl_tail = max(-1.0, min(Cl_tail, 1.0)); % Saturazione stallo coda

% 5. Momento Generato (M = F * d)
L_tail = 0.5 * params.rho * v_sq_tail * s_tail * Cl_tail;
M_pitch_stab = L_tail * d_tail; 

% 6. Smorzamento Aerodinamico (Damping)
% Serve per fermare le oscillazioni (Delfinamento e Rollio)
Damp_pitch = 15.0; % Smorzamento su q
Damp_roll  = 2.0;  % Smorzamento su p
Damp_yaw   = 2.0;  % Smorzamento su r

M_stab_pinna = [
    -x(10) * (1 + Damp_roll * x(5)^2);  % Smorzamento Roll
    M_pitch_stab - Damp_pitch * x(11);  % Stabilità Pitch (Molla + Ammortizzatore)
    -x(12) * (1 + Damp_yaw * x(5)^2)    % Smorzamento Yaw
];

% --- TOTALE MOMENTI ---
M_tot = -M_gyro_body + M_th + M_aero_wing + M_gyro_tilt + M_stab_pinna; 

%% 6. EQUAZIONI DIFFERENZIALI (OUTPUT)

% Dinamica Posizione
x123_dot = R * V_body;

% Dinamica Velocità Lineare (F = ma)
x456_dot = (1/params.m) * F_tot_body;

% Dinamica Angoli (Cinematica)
x789_dot = J * Omega_body;

% Dinamica Velocità Angolari (Eulero: M = I*w_dot + w x Iw)
% Usiamo mldivide (\) invece di inv() per efficienza e precisione
x101112_dot = params.I_body \ M_tot; 

% Assemblaggio vettore derivata di stato
x_dot = [x123_dot; x456_dot; x789_dot; x101112_dot; ...
         dx13; dx14; dx15; dx16; dx17; dx18; dx19; dx20; ...
         dx21; dx22; dx23; dx24; dx25; dx26];

end