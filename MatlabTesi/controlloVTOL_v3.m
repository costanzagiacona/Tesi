function u = controlloVTOL_v3(t, params, x, test_id, target)

% Preallocazione
u = zeros(7,1);

% TEST

% 1 : Controllo verticale

% 2 : Controllo orizzontale 

switch test_id

    case 1
        % =========================================================================
        % SLIDING MODE CONTROL PER CONTROLLO VERTICALE
        % =========================================================================

        % --- 1. Stato e Cinematica ---
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10); q = x(11); r = x(12);

        J = matriceJ(phi,theta,psi);
        Omega = J*[p;q;r];
        p = Omega(1);
        q = Omega(2);
        r = Omega(3);

        R = matriceRotazione(phi, theta, psi);
        V_body = [x(4); x(5); x(6)];
        V_global = R * V_body;
        vx_g = V_global(1); vy_g = V_global(2); vz_g = V_global(3);

        % --- 2. Riferimenti ---
        z_des = -10;    vz_des = 0;
        y_des = 0;      vy_des = 0;
        x_des = 0;      vx_des = 0; 
        psi_des = deg2rad(0);    r_des = 0;

        %% =========================================================================
        %   OUTER LOOP
        % =========================================================================
        
        % Asse Z (Quota)
        lambda_z = 2; K_z = 10; Phi_z = 0.8;
        e_z = z_des - x(3);
        de_z = vz_des - vz_g;
        s_z = de_z + lambda_z * e_z;
        
        % U_z: Forza verticale richiesta 
        U_z = params.m * (params.g - lambda_z * de_z - K_z * tanh(s_z / Phi_z));
        
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

        % --- 2. CALCOLO SPINTA FISICA (Body Frame) ---
        % Proiezione della forza U_z sull'asse verticale del drone
        cos_factor_z = max(cos(theta) * cos(phi), 0.1); % Protezione divisione per zero
        Thrust_req = U_z / cos_factor_z;
        
        % Compensazione del Drag
        F_drag_z = -params.rho * params.s_body_z * params.C_d_z * sign(x(6)) * x(6)^2;
        Thrust_req = Thrust_req + F_drag_z;
        
        % Saturazione fisica della spinta (Sicurezza)
        Thrust_req = max(min(Thrust_req, 100), 5);

        % --- 4. MAPPING DI ASSETTO ---
        
        % ROLLIO (Y -> Phi)
        sin_phi_des = U_y / Thrust_req;
        phi_des = asin(max(min(sin_phi_des, 0.5), -0.5)); % Saturazione a +/- 30°
        
        % BECCHEGGIO (X -> Theta)
        cos_phi_des = max(cos(phi_des), 0.1); % Protezione da divisione per zero

        sin_theta_des = -U_x / (Thrust_req * cos_phi_des); 
        theta_des = asin(max(min(sin_theta_des, 0.5), -0.5)); % Saturazione a +/- 30°
        
        %% =========================================================================
        %   INNER LOOP
        % =========================================================================
        lambda_att = 12.0; 
        K_att = 10.0;      
        Phi_att = 3;     
        
        % accelerazioni angolari desiderate
        ddphi_des = 0; ddtheta_des = 0; ddpsi_des = 0;

        % --- ROLL ---
        e_phi = phi_des - phi;
        de_phi = 0 - p; % Assumendo p_des = 0
        s_phi = de_phi + lambda_att * e_phi;
        
        acc_phi_virtual = ddphi_des + lambda_att * de_phi + K_att * tanh(s_phi / Phi_att);
        Moment_roll_req = params.Ixx * acc_phi_virtual - (params.Iyy - params.Izz) * q * r;

        % --- PITCH ---
        e_theta = theta_des - theta;
        de_theta = 0 - q;
        s_theta = de_theta + lambda_att * e_theta;
        
        acc_theta_virtual = ddtheta_des + lambda_att * de_theta + K_att * tanh(s_theta / Phi_att);
        Moment_pitch_req = params.Iyy * acc_theta_virtual - (params.Izz - params.Iyy) * p * r;

        % --- YAW ---
        lambda_att_yaw = 8.4;
        e_psi = atan2(sin(psi_des - psi), cos(psi_des - psi)); 
        de_psi = r_des - r;
        s_psi = de_psi + lambda_att_yaw * e_psi; 
        
        acc_psi_virtual = ddpsi_des + lambda_att_yaw * de_psi + (K_att*0.8) * tanh(s_psi / Phi_att);
        Moment_yaw_req = params.Izz * acc_psi_virtual - (params.Izz - params.Ixx) * p * q;

        %% =========================================================================
        %   MOTOR MIXING ALGORITHM (MMA)
        % =========================================================================
        
        % --- 0. Parametri di Equilibrio e Stato Attuale ---
        theta3_ideal = atan2(((-params.d_tx * params.k) / params.b), 1);
        theta3_actual = x(17); % Angolo di tilt reale del rotore di coda
        theta4 = -pi/2;        % Configurazione laterale fissa del rotore di coda
        
        % --- YAW - Tilt dei Rotori Anteriori ---
        % Ipotizziamo che il Thrust sia ripartito equamente tra i tre rotori
        F_front_est = max(Thrust_req * 0.65, 2.0); % Protezione: almeno 2N per garantire autorità di Yaw
        delta_tilt_yaw = Moment_yaw_req / (F_front_est * params.d_my);
        
        % Saturazione di sicurezza per il tilt (circa 25-30 gradi)
        max_tilt_rad = deg2rad(30); 
        delta_tilt_yaw = max(min(delta_tilt_yaw, max_tilt_rad), -max_tilt_rad);
        
        % proiezione della spinta sul piano verticale
        % Se delta = 0, cos_delta = 1 (nessuna perdita).
        cos_delta = cos(delta_tilt_yaw);
        
        % ---  Z + PITCH  ---
        denom_mix = (params.d_mx * params.k * sin(theta3_actual) * cos_delta) ...
              - (params.d_tx * params.k * sin(theta3_actual) * cos_delta) ...
              + (params.b * cos(theta3_actual) * sin(theta4));
        
        % Protezione numerica denominatore
        if abs(denom_mix) < 1e-5
        denom_mix = 1e-5 * sign(denom_mix); 
        end
        
        % Calcolo velocità angolare rotore di coda (omega3)
        numeratore_coda = (params.d_mx * Thrust_req * cos_delta) - Moment_pitch_req;
        omega3_sq = max(0, numeratore_coda / denom_mix);
        
        % Spinta verticale effettiva del rotore di coda
        F_tail_z = omega3_sq * params.k * sin(theta3_actual);
        
        % --- ROLL e Distribuzione Frontale ---
        F_front_tot_z = Thrust_req - F_tail_z;
        
        % Calcolo della velocità di base (media) per i motori frontali
        cos_delta = 1;
        omega_front_sq_base = max(0, F_front_tot_z / (2 * params.k * cos_delta));
        
        % Differenziale per il Roll 
        delta_omega_sq = Moment_roll_req / (params.k * params.d_my * 2 * cos_delta);
        
        % Velocità finali motori anteriori
        omega_dx_sq = max(0, omega_front_sq_base - delta_omega_sq);
        omega_sx_sq = max(0, omega_front_sq_base + delta_omega_sq);
        
        % --- 4. Assegnamento Output Finali (u) ---
        u(1) = sqrt(omega_dx_sq);    % Motore Front Destro
        u(2) = sqrt(omega_sx_sq);    % Motore Front Sinistro
        u(3) = sqrt(omega3_sq);      % Motore Coda
        u(4) = pi/2 + delta_tilt_yaw; % Tilt Motore 1 (Rad)
        u(5) = pi/2 - delta_tilt_yaw; % Tilt Motore 2 (Rad)
        u(6) = theta3_ideal;         % Tilt coda
        u(7) = -pi/2;                % Tilt coda

        

    
    case 2
        % =========================================================================
        %  CASE 2: CONTROLLO ORIZZONTALE (CRUISE)
        % 
        % =========================================================================

        % ================================================
        % 1. ESTRAZIONE E PREPARAZIONE STATI
        % ================================================
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10);  q = x(11);    r = x(12);

        % J = matriceJ(phi,theta,psi);
        % Omega = J*[p;q;r];
        % p = Omega(1);
        % q = Omega(2);
        % r = Omega(3);

        % Matrice di rotazione e Velocità nel riferimento Globale (NED)
        R = matriceRotazione(phi, theta, psi);
        V_glob = R * [x(4); x(5); x(6)];
        vx_global = V_glob(1); 
        vy_global = V_glob(2);
        vz_global = V_glob(3); % Positiva verso il basso

        % Lettura Integrali 
        % x(27): Int. Err Velocità
        % x(28): Int. Err Quota
        if length(x) >= 27
            int_err_v     = x(27); 
            int_err_z     = x(28);
        else
            int_err_v = 0; int_err_z = 0;
        end

        % =================================================
        % 2. DEFINIZIONE SETPOINT
        % =================================================
        z_des     = -10; % Quota target (-10 m)
        vx_des    = target(1); % Velocità target (25 m/s)
        phi_des   = 0;

        % =================================================
        % OUTER LOOP
        % ================================================

        % =================================================
        % CONTROLLO QUOTA (Generazione Theta Rif)
        % =================================================
        
        err_h = x(3) - z_des;     % Errore di posizione
        d_err_h = vz_global;      % Derivata (Velocità verticale)
        
        % TUNING QUOTA 
        kp_z_th = 0.1; 
        kd_z_th = 0.09;  
        ki_z_th = 0.005;
        
        theta_cmd_raw = kp_z_th * err_h + kd_z_th * d_err_h + ki_z_th * int_err_z;
        
        % Saturazione Pitch (Safety)
        % In crociera limitiamo il pitch a +/- 20 gradi per evitare stallo
        max_pitch = deg2rad(20); 
        theta_ref = max(-max_pitch, min(max_pitch, theta_cmd_raw));
        

        % =================================================
        % CONTROLLO VELOCITÀ (Generazione Spinta X)
        % =================================================        
        e_v = vx_des - vx_global;
        
        % TUNING VELOCITÀ
        kp_v = 8.0; 
        ki_v = 1.5; 
        
        % Feedforward Aerodinamico (Drag Compensation)
        F_drag_total = 0.5 * params.rho * (params.s * params.C_d + params.s_body_x * params.C_d_x) * vx_global^2;
        
        % Feedforward Gravitazionale 
        F_gravity_x = params.m * params.g * sin(theta);
        
        % Forza Totale Richiesta lungo l'asse X Body
        Fx_req = F_drag_total + F_gravity_x + kp_v * e_v + ki_v * int_err_v;
        
        % Saturazione fisica (No spinta negativa/reverse in volo)
        if Fx_req < 0; Fx_req = 0; end

        % =======================================
        %  CONTROLLO Y
        % =======================================
        y_des = 0;
        err_y = y_des - x(2);
        de_y = 0 - vy_global;

        kp_y = 0.1;
        kd_y = 0.5;

        psi_des = kp_y * err_y + kd_y * de_y;

        % =================================================
        % INNER LOOP: CONTROLLO D'ASSETTO (Momenti)
        % =================================================
        
        % --- PITCH (Momento Y) ---
        e_theta  = theta_ref - theta;
        de_theta = 0 - q; 
        
        kp_th = 5; 
        kd_th = 2.5;   
        M_y_req = kp_th * e_theta + kd_th * de_theta;% + ki_th * int_err_theta;
        
        % --- ROLL (Momento X) ---
        kp_phi = 3; 
        kd_phi = 1.2;
        M_x_req = kp_phi * (phi_des - phi) + kd_phi * (0 - p);
        
        % --- YAW (Momento Z) ---
        kp_psi = 3.0; 
        kd_psi = 1.0;
        M_z_req = kp_psi * (psi_des - psi) + kd_psi * (0 - r);
        
        % =================================================
        % MIXER
        % =================================================
        
        % Parametri geometrici
        d_my = params.d_my; % Braccio laterale
        d_mx = params.d_mx; % Braccio longitudinale
        
        % Calcolo Spinta Totale (T_base)
        % Proiezione: T_base * cos(theta) = Fx_req
        % Protezione: cos(theta) non deve scendere troppo (anche se limitiamo theta a 20°)
        T_base = Fx_req;
        
        % Differenziale di Spinta per Imbardata
        dT_yaw = M_z_req / (2 * d_my);
        
        T_left  = (T_base / 2) + dT_yaw;
        T_right = (T_base / 2) - dT_yaw;

        if T_left > 100; T_left = 100; end
        if T_right > 100; T_right = 100; end 
        
        % Protezione saturazione motori (minimo 0)
        T_left  = max(0, T_left);
        T_right = max(0, T_right);
        
        % Ricalcoliamo T_base effettivo dopo le saturazioni per coerenza nel tilt
        T_base_eff = T_left + T_right;
       
        % PROTEZIONE SINGOLARITÀ MIXER
        % Se la spinta è nulla, non possiamo generare momenti col tilt.
        % Usiamo un valore "fittizio" al denominatore per evitare divisione per zero.
        T_safe_mix = max(T_base_eff, 15.0); % Soglia minima 2 Newton
        
        % Angolo di tilt collettivo per il Pitch
        tilt_pitch = M_y_req / (T_safe_mix * d_mx);

        % Angolo di tilt differenziale per il Roll
        tilt_roll = M_x_req / (T_safe_mix * d_my);
        
        % Combinazione Output Servi
        ts1 = tilt_pitch - tilt_roll; 
        ts2 = tilt_pitch + tilt_roll; 
        
        % Saturazione Servi (Limiti meccanici -45 a +80 gradi)
        ts1 = max(deg2rad(-45), min(deg2rad(80), ts1));
        ts2 = max(deg2rad(-45), min(deg2rad(80), ts2));
        
        % =================================================
        % 7. OUTPUT FINALE (U)
        % =================================================
        
        u(1) = sqrt(T_right / params.k); % Motore 1 (DX)
        u(2) = sqrt(T_left / params.k);  % Motore 2 (SX)
        u(3) = 0;                        % Motore Coda spento in crociera
        u(4) = ts1;                      % Tilt 1
        u(5) = ts2;                      % Tilt 2
        u(6) = 0;                        % Tilt Coda (inutile se spento)
        u(7) = 0;


    otherwise
        fprintf("Controllo selezionato non trovato\n");
        % Se richiami altri case non definiti qui, metti un default
        u = zeros(7,1);
end
end
