function u = controlloVTOL_lineare(t, params, x, test_id, target)
    u = zeros(7,1);
    
    % --- 1. Stato ---
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);
    
    % Ricavo velocità globali
    R = matriceRotazione(phi, theta, psi);
    V_body = [x(4); x(5); x(6)];
    V_global = R * V_body;
    vx_g = V_global(1); vy_g = V_global(2); vz_g = V_global(3);
    
    % --- 2. Riferimenti (Hovering) ---
    z_des = -10;    vz_des = 0;
    y_des = 0;      vy_des = 0;
    x_des = 0;      vx_des = 0; 
    psi_des = 0;    r_des = 0;
    
    %% =========================================================================
    %   OUTER LOOP: POSITION SMC (Rigorosamente Disaccoppiato e C1)
    % =========================================================================
    
    % Asse Z (Quota)
    lambda_z = 2; K_z = 10; Phi_z = 0.8;
    e_z = z_des - x(3);
    de_z = vz_des - vz_g;
    s_z = de_z + lambda_z * e_z;
    
    U_z = params.m * (params.g - (lambda_z * de_z + K_z * tanh(s_z / Phi_z)));
    
    % Assi Y e X (Piano Orizzontale)
    lambda_y = 0.8; K_y = 1.5; Phi_y = 4;
    e_y = y_des - x(2);
    de_y = vy_des - vy_g;
    s_y = de_y + lambda_y * e_y;
    U_y = params.m * (lambda_y * de_y + K_y * tanh(s_y / Phi_y));
    
    lambda_x = 0.8; K_x = 1.5; Phi_x = 4;
    e_x = x_des - x(1);
    de_x = vx_des - vx_g;
    s_x = de_x + lambda_x * e_x;
    U_x = params.m * (lambda_x * de_x + K_x * tanh(s_x / Phi_x));
    
    % --- CALCOLO SPINTA FISICA ---
    % RIMOSSI: max(..., 0.1) e max(min(...))
    cos_factor_z = cos(theta) * cos(phi); 
    Thrust_req = U_z / cos_factor_z;
    
    % Il termine di Drag è stato rimosso per il modello di linearizzazione. 
    % L'operatore sign() in vx^2 * sign(vx) distrugge la derivabilità in zero.
    
    % --- MAPPING DI ASSETTO ---
    sin_phi_des = U_y / Thrust_req;
    phi_des = asin(sin_phi_des); 
    
    cos_phi_des = cos(phi_des); 
    sin_theta_des = -U_x / (Thrust_req * cos_phi_des); 
    theta_des = asin(sin_theta_des); 
    
    %% =========================================================================
    %   INNER LOOP: ATTITUDE SMC
    % =========================================================================
    lambda_att = 12.0; K_att = 10.0; Phi_att = 3;     
    ddphi_des = 0; ddtheta_des = 0; ddpsi_des = 0;
    
    % ROLL
    e_phi = phi_des - phi;
    de_phi = 0 - p; 
    s_phi = de_phi + lambda_att * e_phi;
    acc_phi_virtual = ddphi_des + lambda_att * de_phi + K_att * tanh(s_phi / Phi_att);
    Moment_roll_req = params.Ixx * acc_phi_virtual - (params.Iyy - params.Izz) * q * r;
    
    % PITCH
    e_theta = theta_des - theta;
    de_theta = 0 - q;
    s_theta = de_theta + lambda_att * e_theta;
    acc_theta_virtual = ddtheta_des + lambda_att * de_theta + K_att * tanh(s_theta / Phi_att);
    Moment_pitch_req = params.Iyy * acc_theta_virtual - (params.Izz - params.Iyy) * p * r;
    
    % YAW
    lambda_att_yaw = 8.4;
    % RIMOSSO l'atan2 per la linearizzazione continua
    e_psi = psi_des - psi; 
    de_psi = r_des - r;
    s_psi = de_psi + lambda_att_yaw * e_psi; 
    acc_psi_virtual = ddpsi_des + lambda_att_yaw * de_psi + (K_att*0.8) * tanh(s_psi / Phi_att);
    Moment_yaw_req = params.Izz * acc_psi_virtual - (params.Ixx - params.Iyy) * p * q;
    
    %% =========================================================================
    %   MOTOR MIXING ALGORITHM (MMA)
    % =========================================================================
    theta3_ideal = atan2(((-params.d_tx * params.k) / params.b), 1);
    theta3_actual = x(17); 
    theta4 = -pi/2;
    
    % YAW TILT
    F_front_est = Thrust_req * 0.65; % RIMOSSO il max(..., 2.0)
    delta_tilt_yaw = Moment_yaw_req / (F_front_est * params.d_my); % RIMOSSE saturazioni radianti
    cos_delta = cos(delta_tilt_yaw);
    
    % MIXING LONGITUDINALE
    denom_mix = (params.d_mx * params.k * sin(theta3_actual) * cos_delta) ...
              - (params.d_tx * params.k * sin(theta3_actual) * cos_delta) ...
              + (params.b * cos(theta3_actual) * sin(theta4));
    
    numeratore_coda = (params.d_mx * Thrust_req * cos_delta) - Moment_pitch_req;
    omega3_sq = numeratore_coda / denom_mix; % RIMOSSO max(0,...)
    
    F_tail_z = omega3_sq * params.k * sin(theta3_actual);
    F_front_tot_z = Thrust_req - F_tail_z;
    
    % MIXING LATERALE E DISTRIBUZIONE
    omega_front_sq_base = F_front_tot_z / (2 * params.k * cos_delta); % RIMOSSO max(0,...)
    delta_omega_sq = Moment_roll_req / (params.k * params.d_my * 2 * cos_delta);
    
    omega_dx_sq = omega_front_sq_base - delta_omega_sq;
    omega_sx_sq = omega_front_sq_base + delta_omega_sq;
    
    % Output Finali
    % In un intorno locale dell'equilibrio u > 0, quindi le sqrt non daranno complessi
    u(1) = sqrt(omega_dx_sq);    
    u(2) = sqrt(omega_sx_sq);    
    u(3) = sqrt(omega3_sq);      
    u(4) = pi/2 + delta_tilt_yaw; 
    u(5) = pi/2 - delta_tilt_yaw; 
    u(6) = theta3_ideal; 
    u(7) = -pi/2;
end