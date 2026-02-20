function u = controlloVTOL_lineare_cruise(t, params, x, test_id, target)
    u = zeros(7,1);

    % --- 1. Stati ---
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10);  q = x(11);    r = x(12);

    R = matriceRotazione(phi, theta, psi);
    V_glob = R * [x(4); x(5); x(6)];
    vx_global = V_glob(1); vy_global = V_glob(2); vz_global = V_glob(3);

    % Lettura Integrali puri (assumiamo che il simulatore linearizzabile NON abbia anti-windup)
    int_err_v = x(27); int_err_theta = x(28); int_err_z = x(29);

    % --- 2. Riferimenti ---
    z_des = target(3); vx_des = target(1); phi_des = 0; psi_des = 0; y_des = 0;

    % --- 3. Outer Loop: Quota ---
    err_h = x(3) - z_des;     
    d_err_h = vz_global;      
    kp_z_th = 0.035; kd_z_th = 0.04; ki_z_th = 0.005;
    
    % NESSUNA SATURAZIONE SUL PITCH. Nel modello lineare, le variabili sono infinitesime.
    theta_ref = kp_z_th * err_h + kd_z_th * d_err_h + ki_z_th * int_err_z;

    % --- 4. Outer Loop: Velocità (X) ---
    e_v = vx_des - vx_global;
    kp_v = 8.0; ki_v = 1.5; 
    
    % Per la linearizzazione, togliamo il sign() e usiamo semplicemente v^2 assumendo v > 0
    F_drag_total = 0.5 * params.rho * (params.s * params.C_d + params.s_body_x * params.C_d_x) * vx_global^2;
    F_gravity_x = params.m * params.g * sin(theta);
    
    % NESSUNA SATURAZIONE. Se Fx_req diventa negativo nella matematica locale, lo accettiamo.
    Fx_req = F_drag_total + F_gravity_x + kp_v * e_v + ki_v * int_err_v;

    % --- 5. Outer Loop: Y ---
    err_y = y_des - x(2);
    de_y = 0 - vy_global;
    kp_y = 0.1; kd_y = 0.5;
    psi_des = kp_y * err_y + kd_y * de_y;

    % --- 6. Inner Loop: Assetto ---
    kp_th = 5.0; kd_th = 1.8; ki_th = 0.01; 
    M_y_req = kp_th * (theta_ref - theta) + kd_th * (0 - q) + ki_th * int_err_theta;
    
    kp_phi = 3.0; kd_phi = 0.8;
    M_x_req = kp_phi * (phi_des - phi) + kd_phi * (0 - p);
    
    kp_psi = 3.0; kd_psi = 1.0;
    M_z_req = kp_psi * (psi_des - psi) + kd_psi * (0 - r);

    % --- 7. Mixer (Semplificato e Liscio) ---
    T_base = Fx_req; % Proiezione cos(theta) rimossa per evitare non linearità spurie
    dT_yaw = M_z_req / (2 * params.d_my);
    
    T_left  = (T_base / 2) + dT_yaw;
    T_right = (T_base / 2) - dT_yaw;

    % ELIMINATE TUTTE LE SATURAZIONI MOTORI (Niente max(0, T_left), niente limiti a 100)
    T_base_eff = T_left + T_right;

    % ELIMINATA PROTEZIONE SINGOLARITÀ MIXER. A regime T_base_eff è > 0.
    tilt_pitch = M_y_req / (T_base_eff * params.d_mx);
    tilt_roll  = M_x_req / (T_base_eff * params.d_my);
    
    ts1 = tilt_pitch - tilt_roll; 
    ts2 = tilt_pitch + tilt_roll; 

    % --- 8. Output ---
    % NESSUNA SATURAZIONE SUI SERVI.
    u(1) = sqrt(T_right / params.k); 
    u(2) = sqrt(T_left / params.k);  
    u(3) = 0; % Coda rigorosamente spenta     
    u(4) = ts1;                      
    u(5) = ts2;                      
    u(6) = 0;                        
    u(7) = 0;
end