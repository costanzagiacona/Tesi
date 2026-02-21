function u = controlloVTOL_lineare(t, params, x, test_id, target)
    % CONTROLLOVTOL_LINEARE
    % Regolatore PD Multivariabile - Linearizzazione esatta dell'SMC (Hovering)
    
    u = zeros(7,1);

    % --- 1. Stato e Cinematica ---
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);

    % La matrice di rotazione è differenziabile ovunque tranne nelle singolarità,
    % quindi possiamo mantenerla intatta.
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
    %   OUTER LOOP: PD Equivalente (Controllo di Posizione)
    % =========================================================================

    % --- QUOTA (Z) ---
    lambda_z = 2; K_z = 10; Phi_z = 0.8;
    e_z = z_des - x(3);
    de_z = vz_des - vz_g;
    s_z = de_z + lambda_z * e_z;

    % ELIMINATO il termine non lineare di Drag. 
    % La derivata di sign(v)*v^2 rispetto a v valutata in v=0 è ZERO.
    F_drag_z = 0; 
    
    % SOSTITUITO tanh(s/Phi) con il suo sviluppo in serie di Taylor al primo ordine: (s/Phi)
    u_smc_z = params.m * (lambda_z * de_z) + K_z * (s_z / Phi_z);
    
    % ELIMINATA la saturazione numerica max(..., 0.1) sul coseno. In hovering è pari a 1.
    cos_factor = cos(theta) * cos(phi); 
    Thrust_req = (params.m * params.g - u_smc_z) / cos_factor + F_drag_z;

    % ELIMINATE rigorosamente le saturazioni (Thrust_req < 5 e > 100).
    % Stiamo supponendo di analizzare il sistema nell'intorno dell'equilibrio (65.7 N).

    % --- TRASLAZIONE LATERALE (Y) ---
    lambda_y = 0.8; K_y = 10; Phi_y = 2.5;
    e_y = y_des - x(2);
    de_y = vy_des - vy_g;
    s_y = de_y + lambda_y * e_y;

    cos_factor_y = max(cos(psi) * cos(phi), 0.1);
    F_y_req = (params.m * (lambda_y * de_y) + params.m * K_y * (s_y / Phi_y) ) / cos_factor_y;
    sin_phi_des = F_y_req / Thrust_req;
    
    % ELIMINATE le saturazioni su asin (max e min). 
    % La funzione asin è liscia nell'origine, possiamo lasciarla.
    phi_des = asin(sin_phi_des); 

    % --- TRASLAZIONE LONGITUDINALE (X) ---
    lambda_x = 0.8; K_x = 8; Phi_x = 2.5;
    e_x = x_des - x(1);
    de_x = vx_des - vx_g;
    s_x = de_x + lambda_x * e_x;

    cos_factor_x = max(cos(psi) * cos(theta), 0.1);
    F_x_req = (params.m * (lambda_x * de_x) + params.m * K_x * (s_x / Phi_x)) /cos_factor_x ;
    sin_theta_des = -F_x_req / Thrust_req;
    theta_des = asin(sin_theta_des);

    %% =========================================================================
    %   INNER LOOP: PD Equivalente (Controllo di Assetto)
    % =========================================================================

    lambda_att = 12.0; K_att = 10.0; Phi_att = 3;     

    % --- ROLL ---
    e_phi = phi_des - phi;
    de_phi = 0 - p; 
    s_phi = de_phi + lambda_att * e_phi;
    Moment_roll_req = lambda_att * de_phi + K_att * (s_phi / Phi_att);

    % --- PITCH ---
    e_theta = theta_des - theta;
    de_theta = 0 - q;
    s_theta = de_theta + lambda_att * e_theta;
    Moment_pitch_req = lambda_att * de_theta + K_att * (s_theta / Phi_att);

    % --- YAW ---
    % ELIMINATO atan2(sin, cos) perché genera rumore numerico nel calcolo dello Jacobiano. 
    % In un intorno di zero, e_psi = psi_des - psi è formalmente esatto.
    e_psi = psi_des - psi; 
    de_psi = r_des - r;
    s_psi = de_psi + (lambda_att * 0.7) * e_psi; 
    Moment_yaw_req = (lambda_att * 0.7) * de_psi + (K_att * 0.8) * (s_psi / Phi_att);

    %% =========================================================================
    %   MIXING LISCIO (Senza protezioni logiche e disaccoppiamenti)
    % =========================================================================
    theta3_ideal = atan2(((-params.d_tx * params.k) / params.b), 1);
    theta3_actual = x(17); 
    theta4 = -pi/2;

    denom_mix = params.d_mx * params.k * sin(theta3_actual) ...
              - params.d_tx * params.k * sin(theta3_actual) ...
              + params.b * cos(theta3_actual) * sin(theta4);

    % ELIMINATI if/abs/sign sul denominatore
    numeratore_coda = (params.d_mx * Thrust_req) - Moment_pitch_req;
    
    % ELIMINATI i max(0, ...). Nell'equilibrio, i motori spingono in positivo.
    omega3_sq = numeratore_coda / denom_mix;
    F_tail_z = omega3_sq * params.k * sin(theta3_actual);

    F_front_tot_z = Thrust_req - F_tail_z;

    % ELIMINATA protezione F_safe_for_yaw. A regime, F_front_tot_z è ampio e positivo.
    F_safe_for_yaw = F_front_tot_z; 

    omega_front_sq_base = F_front_tot_z / (2 * params.k);
    delta_omega_sq = Moment_roll_req / (params.k * params.d_my * 2);

    omega_dx_sq = omega_front_sq_base - delta_omega_sq;
    omega_sx_sq = omega_front_sq_base + delta_omega_sq;

    % ELIMINATA saturazione sul tilt differenziale
    delta_tilt_yaw = Moment_yaw_req / (F_safe_for_yaw * params.d_my);

    % --- Output Finali ---
    u(1) = sqrt(omega_dx_sq);    
    u(2) = sqrt(omega_sx_sq);    
    u(3) = sqrt(omega3_sq);      
    u(4) = pi/2 + delta_tilt_yaw; 
    u(5) = pi/2 - delta_tilt_yaw; 
    u(6) = theta3_ideal; 
    u(7) = -pi/2;

end