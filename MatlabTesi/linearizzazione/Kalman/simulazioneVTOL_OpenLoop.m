function x_dot = simulazioneVTOL_OpenLoop(t, x, u, params)
    % simulazioneVTOL_OpenLoop
    % Modello fisico (Plant) ad anello aperto del VTOL.
    % Gli stati sono rigorosamente 26 (dinamica e cinematica corpo + attuatori).
    
    % --- 1. Estrazione stati fisici ---
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10);  q = x(11);    r = x(12); 
    V_body = [x(4); x(5); x(6)]; 
    Omega_body = [p; q; r]; 
    
    u_b = x(4); v_b = x(5); w_b = x(6);

    % --- 2. Estrazione Ingressi di Controllo (u) ---
    % u(1:3) = comandi motori (radici delle spinte proporzionali)
    % u(4:7) = comandi tilt (radianti)
    theta1_des = u(4); theta2_des = u(5);
    theta3_des = u(6); theta4_des = u(7);

    % --- 3. Matrici di Rotazione ---
    R = matriceRotazione(phi, theta, psi);  
    J = matriceJ(phi, theta, psi); 

    % --- 4. Modello Aerodinamico (Semplificato e Liscio) ---
    % Nota: evito operatori logici per garantire la differenziabilità esatta dello Jacobiano
    F_aeroWing = F_aerodyn_wing(params.C_l, params.C_d, 0, params.rho, u_b, w_b, params.s);
    F_aeroBody = Drag_body(params.C_d_x, params.C_d_y, params.C_d_z, params.rho, params.s_body_x, params.s_body_y, params.s_body_z, u_b, v_b, w_b);
    F_aero_body = F_aeroWing + F_aeroBody;

    % --- 5. Equazioni delle Forze (Body Frame) ---
    F_g_body = F_grav(phi, theta, psi, params.m, params.g); 
    input_thrust = [params.k*x(21)^2; params.k*x(23)^2; params.k*x(25)^2; x(13); x(15); x(17); x(19)];
    F_th_body = F_thrust(input_thrust); 
    F_cor = F_Coriolis(Omega_body, V_body, params.m); 

    F_tot_body = F_g_body + F_th_body + F_aero_body - F_cor; % Disturbo nullo nel modello matematico

    % --- 6. Equazioni dei Momenti (Body Frame) ---
    M_gyro_body = MomentGyroBody(params.I_body, Omega_body);

    Iz1 = params.I_rotor_w_dx(3,3); Iz2 = params.I_rotor_w_sx(3,3); Iz3 = params.I_rotor_tail(3,3);
    M_th = M_thrust_2(input_thrust, params.r_th_w_dx, params.r_th_w_sx, params.r_th_tail, params.b, params.k, x(22), x(24), x(26), Iz1, Iz2, Iz3);
    M_aero = MomentAero(params.r_aerodyn_w_dx, params.r_aerodyn_w_sx, params.C_l, params.C_d, 0, params.rho, u_b, w_b, params.s);

    Omega_rotor_w_dx = [0; x(14); 0];
    Omega_rotor_w_sx = [0; x(16); 0];
    Omega_rotor_tail = [0; x(18); x(20)];
    input_thrust_gyro = [x(21); x(23); x(25); x(13); x(15); x(17); x(19)];
    M_gyro_tilt = M_tilt_rotor(input_thrust_gyro, params.I_rotor_w_dx, params.I_rotor_w_sx, params.I_rotor_tail, Omega_rotor_w_dx, Omega_rotor_w_sx, Omega_rotor_tail);

    alpha0x = 1; alpha1x = 1; alpha0z = 1; alpha1z = 1;
    M_stab_pinna = [-p*(alpha0x+alpha1x*v_b^2); 0; -r*(alpha0z+alpha1z*v_b^2)];

    M_tot = -M_gyro_body + M_th + M_aero + M_gyro_tilt + M_stab_pinna; 

    % --- 7. Derivate Cinematiche e Dinamiche ---
    x123_dot = R * V_body;
    x1_dot = x123_dot(1); x2_dot = x123_dot(2); x3_dot = x123_dot(3);
    
    x456_dot = (1/params.m) * F_tot_body;
    x4_dot = x456_dot(1); x5_dot = x456_dot(2); x6_dot = x456_dot(3);
    
    x789_dot = J * Omega_body;
    x7_dot = x789_dot(1); x8_dot = x789_dot(2); x9_dot = x789_dot(3);
    
    x_101112_dot = params.I_body \ M_tot; % Uso operatore \ invece di inv() per stabilità numerica
    x10_dot = x_101112_dot(1); x11_dot = x_101112_dot(2); x12_dot = x_101112_dot(3);

    % --- 8. Dinamica Attuatori (Parametri Costanti del 2° Ordine) ---
    zeta = 0.8; omega_n = 2*pi*15;
    zeta_tail = 0.8; omega_n_tail = 2*pi*15;
    zeta_rotor = 0.9; omega_n_rotor = 2*pi*15;

    x13_dot = x(14); x14_dot = -2*zeta*omega_n*x(14) - (x(13)-theta1_des)*omega_n^2;  
    x15_dot = x(16); x16_dot = -2*zeta*omega_n*x(16) - (x(15)-theta2_des)*omega_n^2;  
    x17_dot = x(18); x18_dot = -2*zeta_tail*omega_n_tail*x(18) - (x(17)-theta3_des)*omega_n_tail^2;  
    x19_dot = x(20); x20_dot = -2*zeta_tail*omega_n_tail*x(20) - (x(19)-theta4_des)*omega_n_tail^2; 
    
    x21_dot = x(22); x22_dot = -2*zeta_rotor*omega_n_rotor*x(22) - (x(21)-u(1))*omega_n_rotor^2; 
    x23_dot = x(24); x24_dot = -2*zeta_rotor*omega_n_rotor*x(24) - (x(23)-u(2))*omega_n_rotor^2; 
    x25_dot = x(26); x26_dot = -2*zeta_rotor*omega_n_rotor*x(26) - (x(25)-u(3))*omega_n_rotor^2; 

    % --- 9. Assemblaggio Vettore (Esattamente 26x1) ---
    x_dot = [x1_dot; x2_dot; x3_dot; x4_dot; x5_dot; x6_dot; ...
             x7_dot; x8_dot; x9_dot; x10_dot; x11_dot; x12_dot; ...
             x13_dot; x14_dot; x15_dot; x16_dot; x17_dot; x18_dot; ...
             x19_dot; x20_dot; x21_dot; x22_dot; x23_dot; x24_dot; ...
             x25_dot; x26_dot];
end