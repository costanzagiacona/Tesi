function u = controlloVTOL_hover(t, params, x)
    
    % =========================================================================
    % CONTROLLORE v10 - STABILITÀ TRICOTTERO REALE
    % Garantisce T3 > 0 e usa un controllo Yaw dolce (Feedforward + PID)
    % =========================================================================

    u = zeros(7,1);
    
    % --- 1. PARAMETRI GEOMETRICI ---
    % Sovrascriviamo le distanze qui per sicurezza (in caso il main non sia aggiornato)
    % Il tricottero DEVE avere questa configurazione per volare bene:
    x_front =  0.20; % Motori davanti al baricentro
    x_rear  = -0.40; % Coda dietro al baricentro
    y_arm   =  0.40; 
    
    max_thrust = 3 * (params.m * params.g);
    k_thrust = params.k;

    % --- 2. STATO ---
    z = x(3);  vz = x(6);
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);

    % --- 3. OUTER LOOP (POSIZIONE) ---
    z_des = -10;
    Kp_z = -10; Kd_z = -6;
    
    F_z_des = params.m*params.g + Kp_z*(z_des - z) + Kd_z*(0 - vz);
    F_z_des = max(0, min(max_thrust, F_z_des)); 

    % Hovering orizzontale (XY control)
    Kp_xy = 0.5; Kd_xy = 0.8;
    ax_req = Kp_xy*(0 - x(1)) + Kd_xy*(0 - x(4));
    ay_req = Kp_xy*(0 - x(2)) + Kd_xy*(0 - x(5));
    
    % Limitiamo l'inclinazione a 10 gradi (0.17 rad) per stabilità
    theta_des = max(-0.17, min(0.17, -ax_req / params.g));
    phi_des   = max(-0.17, min(0.17,  ay_req / params.g));

    % --- 4. INNER LOOP (ASSETTO) ---
    Kp_att = 15; Kd_att = 4;
    
    M_roll_des  = Kp_att*(phi_des - phi)     + Kd_att*(0 - p);
    M_pitch_des = Kp_att*(theta_des - theta) + Kd_att*(0 - q);
    
    % YAW (IMBARDATA)
    % Qui usiamo un PID standard sulle velocità angolari
    Kp_yaw = 5; Kd_yaw = 2;
    M_yaw_req = Kp_yaw*(0 - psi) + Kd_yaw*(0 - r);

    % --- 5. MIXER GEOMETRICO (ALLOCAZIONE) ---
    % Calcoliamo le spinte esatte per bilanciare F_z e Momenti Pitch/Roll
    
    % T3 (Coda) gestisce il Pitch insieme al fronte
    % Eq: T3 * (x_rear - x_front) = M_pitch - F_z * x_front
    numerator = M_pitch_des - F_z_des * x_front;
    denominator = x_rear - x_front; % Questo sarà negativo (-0.6)
    
    T3 = numerator / denominator;
    
    % Se T3 viene negativo (impossibile), lo portiamo a un minimo vitale
    % Questo è il trucco per evitare che la coda "muoia"
    T3 = max(1.0, T3); 
    
    % Il resto va davanti
    T_front = F_z_des - T3;
    
    % Differenza per il Rollio
    diff_roll = M_roll_des / y_arm;
    
    T1 = (T_front - diff_roll) / 2;
    T2 = (T_front + diff_roll) / 2;
    
    % Saturazioni
    T1 = max(0, min(max_thrust/2, T1));
    T2 = max(0, min(max_thrust/2, T2));
    T3 = max(0, min(max_thrust/2, T3));

    % --- 6. GESTIONE TILT CODA (u7) ---
    % La tua logica stabilizzante: Feedforward + Feedback
    
    % A. TERMINE DI EQUILIBRIO (Feedforward)
    % La coda deve inclinarsi per contrastare la coppia del motore T3.
    % M_reazione = b * T3.
    % Per annullarlo, serve una forza laterale F_lat tale che F_lat * d_tx = -M_reazione
    % F_lat = - (b * T3) / d_tx
    % Essendo F_lat = T3 * sin(theta), otteniamo:
    % sin(theta_eq) = - b / d_tx  (Nota: T3 si semplifica!)
    
    % Se b=0.01 e d_tx=-0.4 -> sin(theta) = 0.025 -> theta = 0.025 rad (1.4 gradi)
    % Questo è l'angolo "naturale" a cui la coda deve stare.
    
    sin_theta_eq = -params.b / x_rear; 
    theta_feedforward = asin(max(-1, min(1, sin_theta_eq)));
    
    % B. TERMINE CORRETTIVO (Feedback PID)
    % M_yaw_req è la coppia richiesta per correggere l'errore.
    % F_lat_corr = M_yaw_req / x_rear
    % theta_corr = asin(F_lat_corr / T3)
    
    force_yaw_corr = M_yaw_req / x_rear;
    theta_feedback = asin(max(-0.5, min(0.5, force_yaw_corr / T3)));
    
    % SOMMA: Angolo Totale
    theta_4_total = theta_feedforward + theta_feedback;
    
    % Saturazione Servo (Max 20 gradi = 0.35 rad)
    u(7) = max(-0.35, min(0.35, theta_4_total));
    
    % ATTENZIONE: Se gira ancora su se stesso, cambia il segno qui sotto a -1
    YAW_DIRECTION = 1; 
    u(7) = u(7) * YAW_DIRECTION;

    % --- 7. OUTPUT MOTORI ---
    u(1) = real(sqrt(T1 / k_thrust));
    u(2) = real(sqrt(T2 / k_thrust));
    u(3) = real(sqrt(T3 / k_thrust));
    
    % Tilt Navicelle (Fisse Verticali)
    u(4) = pi/2; 
    u(5) = pi/2; 
    u(6) = pi/2;

end