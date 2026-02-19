function u = controlloVTOL_v3(t, params, x, test_id, target)

% Preallocazione
u = zeros(7,1);

% TEST

% -1 : Debug (tutto simbolico)

% 0 : Nessun controllo (solo gravità)

% 1 : Controllo verticale

% 2 : Controllo orizzontale 

% 3 : Controllo orizzontale con momenti 

switch test_id

    case -1
        %debug
        syms u1 u2 u3 u4 u5 u6 u7
        
        % variante in assenza di rotore di coda 
        u = [u1;u2;0;u4;u5;0;0];

    case 0
        % Nessun controllo (solo gravità)
        u = zeros(7,1);

    case 1
        % =========================================================================
        % CASE 1: RIGOROUS CASCADED SMC WITH MIXER PROTECTION & CHATTERING FILTER
        % =========================================================================

        % --- 1. Stato e Cinematica ---
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10); q = x(11); r = x(12);

        R = matriceRotazione(phi, theta, psi);
        V_body = [x(4); x(5); x(6)];
        V_global = R * V_body;
        vx_g = V_global(1); vy_g = V_global(2); vz_g = V_global(3);

        % --- 2. Riferimenti (Setpoint) ---
        z_des = -10;    vz_des = 0;
        y_des = 0;      vy_des = 0;
        x_des = 0;      vx_des = 0; 
        psi_des = 0;    r_des = 0;

        %% =========================================================================
        %   OUTER LOOP: POSITION SMC (Parametri per stabilità globale)
        % =========================================================================
        % Lambda basso (risposta dolce), Phi alto (comando pulito per il loop interno)

        % --- QUOTA (Z) ---
        lambda_z = 2; K_z = 10; Phi_z = 0.8;
        e_z = z_des - x(3);
        de_z = vz_des - vz_g;
        s_z = de_z + lambda_z * e_z;

        % F_grav = params.m * params.g * cos(theta) * cos(phi);
        F_drag_z = -params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        % F_lift = -params.C_l*params.rho*params.s*x(4)^2;
        u_smc_z = params.m * (lambda_z * de_z) + K_z * tanh(s_z / Phi_z);
        cos_factor = max(cos(theta) * cos(phi), 0.1); % Protezione divisione per zero
        Thrust_req = (params.m * params.g - u_smc_z + F_drag_z) / cos_factor ;

        % Protezione Saturazione
        if Thrust_req < 5; Thrust_req = 5; end
        if Thrust_req > 100; Thrust_req = 100; end

        % --- TRASLAZIONE LATERALE (Y -> Roll Desiderato) ---
        lambda_y = 0.8; K_y = 10; Phi_y = 2.5;
        e_y = y_des - x(2);
        de_y = vy_des - vy_g;
        s_y = de_y + lambda_y * e_y;

        F_y_req = params.m * (lambda_y * de_y) + K_y * tanh(s_y / Phi_y);
        sin_phi_des = F_y_req / Thrust_req;
        phi_des = asin(max(min(sin_phi_des, 0.5), -0.5)); % Saturazione ~30°

        % --- TRASLAZIONE LONGITUDINALE (X -> Pitch Desiderato) ---
        lambda_x = 0.8; K_x = 8; Phi_x = 2.5;
        e_x = x_des - x(1);
        de_x = vx_des - vx_g;
        s_x = de_x + lambda_x * e_x;

        F_x_req = params.m * (lambda_x * de_x) + K_x * tanh(s_x / Phi_x);
        sin_theta_des = -F_x_req / Thrust_req;
        theta_des = asin(max(min(sin_theta_des, 0.5), -0.5));

        %% =========================================================================
        %   INNER LOOP: ATTITUDE SMC
        % =========================================================================
        % Lambda alto (inseguimento veloce), Phi aumentato per pulire p,q,r

        lambda_att = 12.0; % Più veloce del loop esterno
        K_att = 20.0;      % Ridotto per evitare eccitazione armonica
        Phi_att = 1.8;     % Valore critico 

        % --- ROLL ---
        e_phi = phi_des - phi;
        de_phi = 0 - p; 
        s_phi = de_phi + lambda_att * e_phi;
        Moment_roll_req = lambda_att * de_phi + K_att * tanh(s_phi / Phi_att);

        % --- PITCH ---
        e_theta = theta_des - theta;
        de_theta = 0 - q;
        s_theta = de_theta + lambda_att * e_theta;
        Moment_pitch_req = lambda_att * de_theta + K_att * tanh(s_theta / Phi_att);

        % --- YAW ---
        e_psi = atan2(sin(psi_des - psi), cos(psi_des - psi)); 
        de_psi = r_des - r;
        s_psi = de_psi + (lambda_att * 0.7) * e_psi; 
        Moment_yaw_req = (lambda_att * 0.7) * de_psi + (K_att * 0.8) * tanh(s_psi / Phi_att);

        %% =========================================================================
        %   MIXING E ALLOCAZIONE CON PROTEZIONE ANTIDIVERGENZA
        % =========================================================================
        theta3_ideal = atan2(((-params.d_tx * params.k) / params.b), 1);
        theta3_actual = x(17); 
        theta4 = -pi/2;

        % 1. Mixing Longitudinale (Z + Pitch)
        denom_mix = params.d_mx * params.k * sin(theta3_actual) ...
                  - params.d_tx * params.k * sin(theta3_actual) ...
                  + params.b * cos(theta3_actual) * sin(theta4);

        % Protezione numerica denominatore
        if abs(denom_mix) < 1e-4; denom_mix = 1e-4 * sign(denom_mix); end

        numeratore_coda = (params.d_mx * Thrust_req) - Moment_pitch_req;
        omega3_sq = max(0, numeratore_coda / denom_mix);
        F_tail_z = omega3_sq * params.k * sin(theta3_actual);

        % 2. Mixing Laterale (Roll)
        F_front_tot_z = Thrust_req - F_tail_z;

        % *** PROTEZIONE CRITICA ***
        % Se la spinta anteriore è troppo bassa, il controllo di Yaw via tilt fallisce.
        % Impediamo al denominatore di scendere sotto una soglia di sicurezza.
        F_safe_for_yaw = max(F_front_tot_z, 2.0); 

        omega_front_sq_base = max(0, F_front_tot_z / (2 * params.k));
        delta_omega_sq = Moment_roll_req / (params.k * params.d_my * 2);

        omega_dx_sq = max(0, omega_front_sq_base - delta_omega_sq);
        omega_sx_sq = max(0, omega_front_sq_base + delta_omega_sq);

        % 3. Mixing Yaw (Tilt) con saturazione e protezione
        delta_tilt_yaw = Moment_yaw_req / (F_safe_for_yaw * params.d_my);
        max_tilt_rad = 0.45; % Circa 25 gradi
        delta_tilt_yaw = max(min(delta_tilt_yaw, max_tilt_rad), -max_tilt_rad);

        % --- Output Finali ---
        u(1) = sqrt(omega_dx_sq);    
        u(2) = sqrt(omega_sx_sq);    
        u(3) = sqrt(omega3_sq);      
        u(4) = pi/2 + delta_tilt_yaw; 
        u(5) = pi/2 - delta_tilt_yaw; 
        u(6) = theta3_ideal; 
        u(7) = -pi/2;

    
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

        J = matriceJ(phi,theta,psi);
        Omega = J*[p;q;r];
        p = Omega(1);
        q = Omega(2);
        r = Omega(3);

        % Matrice di rotazione e Velocità nel riferimento Globale (NED)
        R = matriceRotazione(phi, theta, psi);
        V_glob = R * [x(4); x(5); x(6)];
        vx_global = V_glob(1); 
        vy_global = V_glob(2);
        vz_global = V_glob(3); % Positiva verso il basso

        % Lettura Integrali 
        % x(27): Int. Err Velocità
        % x(28): Int. Err Pitch
        % x(29): Int. Err Quota
        if length(x) >= 29
            int_err_v     = x(27); 
            int_err_theta = x(28); 
            int_err_z     = x(29);
        else
            int_err_v = 0; int_err_theta = 0; int_err_z = 0;
        end

        % =================================================
        % 2. DEFINIZIONE SETPOINT
        % =================================================
        z_des     = target(3); % Quota target (-10 m)
        vx_des    = target(1); % Velocità target (25 m/s)
        phi_des   = 0;
        psi_des   = 0;

        % =================================================
        % 3. OUTER LOOP: CONTROLLO QUOTA (Generazione Theta Rif)
        % =================================================
        
        err_h = x(3) - z_des;     % Errore di posizione
        d_err_h = vz_global;      % Derivata (Velocità verticale)
        
        % TUNING QUOTA (Rilassato e Smorzato)
        kp_z_th = 0.035; 
        kd_z_th = 0.04;  
        ki_z_th = 0.005;
        
        theta_cmd_raw = kp_z_th * err_h + kd_z_th * d_err_h + ki_z_th * int_err_z;
        
        % Saturazione Pitch (Safety)
        % In crociera limitiamo il pitch a +/- 20 gradi per evitare stallo
        max_pitch = deg2rad(20); 
        theta_ref = max(-max_pitch, min(max_pitch, theta_cmd_raw));

        % =================================================
        % 4. OUTER LOOP: CONTROLLO VELOCITÀ (Generazione Spinta X)
        % =================================================
        
        e_v = vx_des - vx_global;
        
        % TUNING VELOCITÀ
        kp_v = 8.0; 
        ki_v = 1.5; 
        
        % A. Feedforward Aerodinamico (Drag Compensation)
        F_drag_total = 0.5 * params.rho * (params.s * params.C_d + params.s_body_x * params.C_d_x) * vx_global^2;
        
        % B. Feedforward Gravitazionale 
        % Se il drone cabra (theta > 0), il peso lo tira indietro.
        F_gravity_x = params.m * params.g * sin(theta);
        
        % Forza Totale Richiesta lungo l'asse X Body
        Fx_req = F_drag_total + F_gravity_x + kp_v * e_v + ki_v * int_err_v;
        
        % Saturazione fisica (No spinta negativa/reverse in volo)
        if Fx_req < 0; Fx_req = 0; end

        % =======================================
        %   CONTROLLO LUNGO Y
        % =======================================
        y_des = 0;
        err_y = y_des - x(2);
        de_y = 0 - vy_global;

        kp_y = 0.1;
        kd_y = 0.5;

        psi_des = kp_y * err_y + kd_y * de_y;

        % =================================================
        % 5. INNER LOOP: CONTROLLO D'ASSETTO (Momenti)
        % =================================================
        
        % --- A. PITCH (Momento Y) ---
        e_theta  = theta_ref - theta;
        de_theta = 0 - q; 
        
        kp_th = 5.0; 
        kd_th = 1.8;   
        ki_th = 0.01; 
        M_y_req = kp_th * e_theta + kd_th * de_theta + ki_th * int_err_theta;
        
        % --- B. ROLL (Momento X) ---
        kp_phi = 3.0; 
        kd_phi = 0.8;
        M_x_req = kp_phi * (phi_des - phi) + kd_phi * (0 - p);
        
        % --- C. YAW (Momento Z) ---
        kp_psi = 3.0; 
        kd_psi = 1.0;
        M_z_req = kp_psi * (psi_des - psi) + kd_psi * (0 - r);
        
        % =================================================
        % 6. MIXER & ALLOCAZIONE 
        % =================================================
        
        % Parametri geometrici
        d_my = params.d_my; % Braccio laterale
        d_mx = params.d_mx; % Braccio longitudinale
        
        % Calcolo Spinta Totale (T_base)
        % Proiezione: T_base * cos(theta) = Fx_req
        % Protezione: cos(theta) non deve scendere troppo (anche se limitiamo theta a 20°)
        T_base = Fx_req;
        
        % 1. ALLOCAZIONE YAW (Differenziale di Spinta)
        % M_z = (T_right - T_left) * d_my => Delta_T = M_z / (2 * d_my)
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
        
        % 2. ALLOCAZIONE TILT (Pitch & Roll)
        % M_y = T_base_eff * sin(theta_tilt_common) * d_mx
        % M_x = T_base_eff * sin(theta_tilt_diff) * d_my
        
        % PROTEZIONE SINGOLARITÀ MIXER
        % Se la spinta è nulla, non possiamo generare momenti col tilt.
        % Usiamo un valore "fittizio" al denominatore per evitare divisione per zero.
        T_safe_mix = max(T_base_eff, 2.0); % Soglia minima 2 Newton
        
        % Angolo di tilt collettivo per il Pitch
        % Approssimazione piccoli angoli: sin(x) ~ x
        tilt_pitch = M_y_req / (T_safe_mix * d_mx);
        
        % Angolo di tilt differenziale per il Roll
        tilt_roll  = M_x_req / (T_safe_mix * d_my);
        
        % Combinazione Output Servi
        ts1 = tilt_pitch - tilt_roll; 
        ts2 = tilt_pitch + tilt_roll; 
        
        % Saturazione Servi (Limiti meccanici -45 a +80 gradi)
        ts1 = max(deg2rad(-45), min(deg2rad(80), ts1));
        ts2 = max(deg2rad(-45), min(deg2rad(80), ts2));
        
        % =================================================
        % 7. OUTPUT FINALE (U)
        % =================================================
        % u(1..3) sono Velocità angolari rotori (rad/s)
        % u(4..5) sono angoli tilt (rad)
        
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
