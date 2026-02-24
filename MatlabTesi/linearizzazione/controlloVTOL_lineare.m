function u = controlloVTOL_lineare(t, params, x, test_id, target)
    u = zeros(7,1);
    
    % --- 1. Stato ---
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);
    
    R = matriceRotazione(phi, theta, psi);
    V_body = [x(4); x(5); x(6)];
    V_global = R * V_body;
    vx_g = V_global(1); vy_g = V_global(2); vz_g = V_global(3);
    
    % --- 2. Riferimenti (Hovering) ---
    z_des = -10;    vz_des = 0;
    y_des = 0;      vy_des = 0;
    x_des = 0;      vx_des = 0; 
    psi_des = 0;    r_des = 0;
    
    %% --- OUTER LOOP ---
    % Z
    lambda_z = 2; K_z = 10; Phi_z = 0.8;
    e_z = z_des - x(3);
    de_z = vz_des - vz_g;
    s_z = de_z + lambda_z * e_z;
    
    acc_z_req = lambda_z * de_z + (K_z / Phi_z) * s_z;
    cos_factor_z = cos(theta) * cos(phi);
    Thrust_req = params.m * (params.g - acc_z_req) / cos_factor_z;
    
    % Y
    lambda_y = 0.8; K_y = 10; Phi_y = 2.5;
    e_y = y_des - x(2);
    de_y = vy_des - vy_g;
    s_y = de_y + lambda_y * e_y;
    
    cos_factor_y = sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi);
    F_y_req = params.m * (lambda_y * de_y + (K_y / Phi_y) * s_y) / cos_factor_y;
    phi_des = asin(F_y_req / Thrust_req);
    
    % X
    lambda_x = 0.8; K_x = 8; Phi_x = 2.5;
    e_x = x_des - x(1);
    de_x = vx_des - vx_g;
    s_x = de_x + lambda_x * e_x;
    
    cos_factor_x = cos(psi) * cos(theta);
    F_x_req = params.m * (lambda_x * de_x + (K_x / Phi_x) * s_x) / cos_factor_x;
    theta_des = asin(-F_x_req / Thrust_req);
    
    %% --- INNER LOOP ---
    lambda_att = 12.0; K_att = 10.0; Phi_att = 3;     
    
    % ROLL
    e_phi = phi_des - phi;
    de_phi = 0 - p; 
    s_phi = de_phi + lambda_att * e_phi;
    acc_phi_virtual = lambda_att * de_phi + (K_att / Phi_att) * s_phi;
    Moment_roll_req = params.Ixx * acc_phi_virtual; % INERZIA INSERITA
    
    % PITCH
    e_theta = theta_des - theta;
    de_theta = 0 - q;
    s_theta = de_theta + lambda_att * e_theta;
    acc_theta_virtual = lambda_att * de_theta + (K_att / Phi_att) * s_theta;
    Moment_pitch_req = params.Iyy * acc_theta_virtual; % INERZIA INSERITA
    
    % YAW
    e_psi = psi_des - psi; 
    de_psi = r_des - r;
    s_psi = de_psi + (lambda_att * 0.7) * e_psi; 
    acc_psi_virtual = (lambda_att * 0.7) * de_psi + (K_att * 0.8 / Phi_att) * s_psi;
    Moment_yaw_req = params.Izz * acc_psi_virtual; % INERZIA INSERITA
    
    %% --- MIXING ---
    theta3_ideal = atan2(((-params.d_tx * params.k) / params.b), 1);
    theta3_actual = x(17); 
    theta4 = -pi/2;
    
    denom_mix = params.d_mx * params.k * sin(theta3_actual) ...
              - params.d_tx * params.k * sin(theta3_actual) ...
              + params.b * cos(theta3_actual) * sin(theta4);
              
    numeratore_coda = (params.d_mx * Thrust_req) - Moment_pitch_req;
    omega3_sq = numeratore_coda / denom_mix;
    
    F_tail_z = omega3_sq * params.k * sin(theta3_actual);
    F_front_tot_z = Thrust_req - F_tail_z;
    
    omega_front_sq_base = F_front_tot_z / (2 * params.k);
    delta_omega_sq = Moment_roll_req / (params.k * params.d_my * 2);
    
    omega_dx_sq = omega_front_sq_base - delta_omega_sq;
    omega_sx_sq = omega_front_sq_base + delta_omega_sq;
    
    delta_tilt_yaw = Moment_yaw_req / (F_front_tot_z * params.d_my);
    
    % Output Finali
    u(1) = sqrt(omega_dx_sq);    
    u(2) = sqrt(omega_sx_sq);    
    u(3) = sqrt(omega3_sq);      
    u(4) = pi/2 + delta_tilt_yaw; 
    u(5) = pi/2 - delta_tilt_yaw; 
    u(6) = theta3_ideal; 
    u(7) = -pi/2;
end