function x_dot = simulazioneVTOL3_linearizzabile(t, x, params, test_id, disturbo, target, simbolico)

% check parametri

paramFlag = 0;

if paramFlag == 1
    % === Stampa dei parametri ===
    fprintf('\n========= PARAMETRI VTOL =========\n');
    fprintf('Massa (m):            %.2f kg\n', params.m);
    fprintf('Gravità (g):          %.2f m/s^2\n', params.g);
    fprintf('Thrust coeff. (k):    %.2f\n', params.k);
    fprintf('Drag coeff. (C_d):    %.2f\n', params.C_d);
    fprintf('Lift coeff. (C_l):    %.2f\n', params.C_l);
    fprintf('Densità aria (rho):   %.2f kg/m^3\n', params.rho);
    fprintf('Dimensioni ala (ala_x): %.2f m\n', params.ala_x);
    fprintf('Dimensioni ala (ala_y): %.2f m\n', params.ala_y);
    fprintf('Superficie alare (s): %.2f m^2\n', params.s);
    fprintf('Velocità aria:        %.2f m/s\n', params.v_air);
    fprintf('b (torque/thrust): %.2f\n', params.b);
    fprintf('\n-- Distanze rotori rispetto al centro di massa --\n');
    disp('r_th_w_dx ='); disp(params.r_th_w_dx);
    disp('r_th_w_sx ='); disp(params.r_th_w_sx);
    disp('r_th_tail ='); disp(params.r_th_tail);

    fprintf('-- Distanze forze aerodinamiche --\n');
    disp('r_aerodyn_w_dx ='); disp(params.r_aerodyn_w_dx);
    disp('r_aerodyn_w_sx ='); disp(params.r_aerodyn_w_sx);
    %disp('r_aerodyn_tail ='); disp(params.r_aerodyn_tail);

    fprintf('-- Matrici di inerzia --\n');
    disp('I_body ='); disp(params.I_body);
    disp('I_rotor_w_dx ='); disp(params.I_rotor_w_dx);
    disp('I_rotor_w_sx ='); disp(params.I_rotor_w_sx);
    disp('I_rotor_tail ='); disp(params.I_rotor_tail);
    fprintf('===================================\n\n');

end

% stati e cinematica
phi = x(7); theta = x(8); psi = x(9);
p = x(10);  q = x(11);    r = x(12); 
V_body = [x(4);x(5);x(6)]; 
Omega_body = [p;q;r]; 

%% SELEZIONE DEL CONTROLLORE LINEARE LISCIO
if test_id == 1
    u = controlloVTOL_lineare(t, params, x, test_id, target);
elseif test_id == 2
    u = controlloVTOL_lineare_cruise(t, params, x, test_id, target);
else
    u = zeros(7,1);
end

%% dinamica attuatori
zeta = 0.8; omega_n = 2*pi*15;
zeta_tail = 0.8; omega_n_tail = 2*pi*15;
zeta_rotor = 0.9; omega_n_rotor = 2*pi*15;

if simbolico == 1
    syms zeta omega_n zeta_tail omega_n_tail zeta_rotor omega_n_rotor
end

theta1_des = u(4); theta2_des = u(5);
theta3_des = u(6); theta4_des = u(7);

%% matrici di rotazione e trasformazione
R = matriceRotazione(phi,theta,psi);  
J = matriceJ(phi,theta,psi); 

%% wind frame 
u_b = x(4); v_b = x(5); w_b = x(6);
Va = norm([u_b, v_b, w_b]); 

% NOTA BENE: Questa discontinuità (Va < 0.1) è innocua per la linearizzazione 
% in crociera perché valuteremo lo Jacobiano in Va = 25 m/s.
if simbolico == 0
    if Va < 0.1
        Va = 0.1; alpha = 0; beta = 0;
    else
        alpha = atan2(w_b, u_b);
        beta = asin(v_b / Va); 
    end
    Rwb = matriceRotazioneWingToBodyFrame(alpha,beta);
    [CL_eff,CD_eff] = coeffAero(alpha,beta,params);

    %% forze aerodinamiche
    F_aeroBody = Drag_body(params.C_d_x,params.C_d_y,params.C_d_z, params.rho,params.s_body_x,params.s_body_y,params.s_body_z,x(4),x(5),x(6));
    F_aeroWing = F_aero_wing(CL_eff,CD_eff,params.C_y, params.rho, params.s,Va,Rwb);
else 
    F_aeroWing = F_aerodyn_wing(params.C_l,params.C_d,0, params.rho ,x(4),x(6), params.s);
    F_aeroBody = Drag_body(params.C_d_x,params.C_d_y,params.C_d_z, params.rho,params.s_body_x,params.s_body_y,params.s_body_z,x(4),x(5),x(6));
end

%% eq. forze BODY FRAME
F_g_body = F_grav(phi , theta , psi , params.m ,params.g); 
input_thrust = [params.k*x(21)^2; params.k*x(23)^2; params.k*x(25)^2; x(13); x(15); x(17); x(19)];
F_th_body = F_thrust(input_thrust); 
F_aero_body = F_aeroWing + F_aeroBody;
F_cor = F_Coriolis(Omega_body,V_body,params.m); 

% DISATTIVAZIONE DISTURBO: Per lo Jacobiano, il disturbo deve essere nullo.
F_disturbo = [0; 0; 0]; 

F_tot_body = F_g_body + F_th_body + F_aero_body - F_cor + F_disturbo; 

%% eq. Momenti
M_gyro_body = MomentGyroBody(params.I_body,Omega_body);

Iz1 = params.I_rotor_w_dx(3,3); Iz2 = params.I_rotor_w_sx(3,3); Iz3 = params.I_rotor_tail(3,3);
M_th = M_thrust_2(input_thrust,params.r_th_w_dx,params.r_th_w_sx,params.r_th_tail,params.b,params.k,x(22),x(24),x(26),Iz1,Iz2,Iz3);
M_aero = MomentAero(params.r_aerodyn_w_dx,params.r_aerodyn_w_sx,params.C_l,params.C_d,0, params.rho ,x(4),x(6), params.s);

Omega_rotor_w_dx = [0;x(14);0];
Omega_rotor_w_sx = [0;x(16);0];
Omega_rotor_tail = [0;x(18);x(20)];
input_thrust_gyro = [x(21);x(23);x(25);x(13);x(15);x(17);x(19)];
M_gyro_tilt = M_tilt_rotor(input_thrust_gyro,params.I_rotor_w_dx,params.I_rotor_w_sx,params.I_rotor_tail,Omega_rotor_w_dx , Omega_rotor_w_sx , Omega_rotor_tail);

alpha0x = 1; alpha1x = 1; alpha0z = 1; alpha1z = 1;
M_stab_pinna = [-x(10)*(alpha0x+alpha1x*x(5)^2); 0; -x(12)*(alpha0z+alpha1z*x(5)^2)];

M_tot = -M_gyro_body + M_th + M_aero + M_gyro_tilt + M_stab_pinna; 

% =========================================================================
% --- AGGIORNAMENTO DINAMICA INTEGRALI (Solo per test_id = 2)
% =========================================================================
V_global_temp = R * V_body; 
vx_global_curr = V_global_temp(1);
vx_target = target(1);    
theta_target = target(2);  
z_target = target(3);

if test_id == 2 || test_id == 3
    x27_dot = vx_target - vx_global_curr;
    x28_dot = theta_target - theta;
    x29_dot = x(3) - z_target;
    x30_dot = 0 - x(2);
end

% === RIMOZIONE RIGOROSA ANTI-WINDUP ===
% Nel modello linearizzato le saturazioni non devono scattare.
% Le equazioni di stato devono essere sempre C1 (lisce e differenziabili).
% Il blocco "if abs(x(27)) > 20..." è stato rimosso appositamente.

%=====================================================================
% Derivate cinematiche e dinamiche
x123_dot = R*V_body;
x1_dot = x123_dot(1); x2_dot = x123_dot(2); x3_dot = x123_dot(3);
x4_dot = (1/params.m)*F_tot_body(1); x5_dot = (1/params.m)*F_tot_body(2); x6_dot = (1/params.m)*F_tot_body(3);
x789_dot = J*Omega_body;
x7_dot = x789_dot(1); x8_dot = x789_dot(2); x9_dot = x789_dot(3);
x_101112_dot = params.I_body \ M_tot; % Ottimizzazione numerica: inv() è meno stabile di \
x10_dot = x_101112_dot(1); x11_dot = x_101112_dot(2); x12_dot = x_101112_dot(3);

% Derivate attuatori
x13_dot = x(14); x14_dot = -2*zeta*omega_n*x(14) -(x(13)-theta1_des)*omega_n^2;  
x15_dot = x(16); x16_dot = -2*zeta*omega_n*x(16) -(x(15)-theta2_des)*omega_n^2;  
x17_dot = x(18); x18_dot = -2*zeta_tail*omega_n_tail*x(18) -(x(17)-theta3_des)*omega_n_tail^2;  
x19_dot = x(20); x20_dot = -2*zeta_tail*omega_n_tail*x(20) -(x(19)-theta4_des)*omega_n_tail^2; 
x21_dot = x(22); x22_dot = -2*zeta_rotor*omega_n_rotor*x(22) -(x(21)-u(1))*omega_n_rotor^2; 
x23_dot = x(24); x24_dot = -2*zeta_rotor*omega_n_rotor*x(24) -(x(23)-u(2))*omega_n_rotor^2; 
x25_dot = x(26); x26_dot = -2*zeta_rotor*omega_n_rotor*x(26) -(x(25)-u(3))*omega_n_rotor^2; 

%% Assemblaggio Vettore Derivate
if simbolico == 1
    x_dot = [x1_dot;x2_dot;x3_dot;x4_dot;x5_dot;x6_dot;x7_dot;x8_dot;x9_dot;x10_dot;x11_dot;x12_dot;x13_dot;x14_dot;x15_dot;x16_dot;x17_dot;x18_dot;x19_dot;x20_dot;x21_dot;x22_dot;x23_dot;x24_dot;x25_dot;x26_dot];
else
    if test_id == 2
        % Modello completo 30 stati per la crociera (PID attivi)
        x_dot = [x1_dot;x2_dot;x3_dot;x4_dot;x5_dot;x6_dot;x7_dot;x8_dot;x9_dot;x10_dot;x11_dot;x12_dot;x13_dot;x14_dot;x15_dot;x16_dot;x17_dot;x18_dot;x19_dot;x20_dot;x21_dot;x22_dot;x23_dot;x24_dot;x25_dot;x26_dot; x27_dot; x28_dot; x29_dot; x30_dot];
    elseif test_id == 1
        % Modello a 26 stati per l'hovering
        x_dot = [x1_dot;x2_dot;x3_dot;x4_dot;x5_dot;x6_dot;x7_dot;x8_dot;x9_dot;x10_dot;x11_dot;x12_dot;x13_dot;x14_dot;x15_dot;x16_dot;x17_dot;x18_dot;x19_dot;x20_dot;x21_dot;x22_dot;x23_dot;x24_dot;x25_dot;x26_dot];
    end
end
end