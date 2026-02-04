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
        % --- CONTROLLO VERTICALE
        % angoli di roll ,pitch, yaw
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10); q = x(11); r = x(12);
        R = matriceRotazione(phi,theta,psi);
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body;
        vx_global = V_global(1); 
        vy_global = V_global(2);
        vz_global = V_global(3);

        % 2. Parametri Obiettivo
        z_des = -10;    vz_des = 0;
        y_des = 0;      vy_des = 0;
        x_des = 0;      vx_des = 0; 
        psi_des = 0;    r_des = 0;  
        % psi_des = 45 * (pi/180);  % ~0.785 radianti

        % =================================================
        % Lettura Integrali 
        % =================================================
        if length(x) >= 29
            int_err_x = x(27);
            int_err_theta = x(28); % Integrale Pitch
        else
            int_err_x = 0; int_err_theta = 0; 
        end

        %% OUTER LOOP

        % =========================================================
        %   LOOP Z (QUOTA) 
        % =========================================================
        lambda_z = 2.5; K_z_smc = 60; Phi_z = 0.8;

        e_z = z_des - x(3);           
        de_z = vz_des - vz_global;    
        s_z = de_z + lambda_z * e_z;    
        F_grav = params.m * params.g * cos(theta) * cos(phi); 
        F_drag_z = -params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_lift = -params.C_l*params.rho*params.s*x(4)^2;
        u_smc_z = params.m * lambda_z * de_z + K_z_smc * tanh(s_z / Phi_z);
        % Thrust_req = F_grav + F_drag_z + F_lift - u_smc_z;
        
        % Anche senza conoscere i termini in feedforward, il controllore
        % stabilizza la quota
        Thrust_req = F_grav - u_smc_z;
        if Thrust_req < 1; Thrust_req = 1; end

        % =========================================================
        %   LOOP Y 
        % =========================================================
        lambda_y = 0.8; K_y_smc = 15; Phi_y = 3.0;

        e_y = y_des - x(2);          
        de_y = vy_des - vy_global;   
        s_y = de_y + lambda_y * e_y; 
        F_y_req = params.m * lambda_y * de_y + K_y_smc * tanh(s_y / Phi_y);
        sin_phi_des = F_y_req / Thrust_req;
        sin_phi_des = max(min(sin_phi_des, 0.5), -0.5);

        % =========================================================
        %   LOOP X
        % =========================================================
        lambda_x = 0.8; K_x_smc = 8; Phi_x = 1.0;

        e_x = x_des - x(1);
        de_x = vx_des - vx_global;
        s_x = de_x + lambda_x * e_x;
        % Dentro il loop di controllo
        % fprintf('Integrale X: %f\n', int_err_x);
        Ki_x = 2.0;
        u_int_x = Ki_x * int_err_x;
        F_x_req = params.m * lambda_x * de_x + K_x_smc * tanh(s_x / Phi_x) + u_int_x;

        sin_theta_des = -F_x_req / Thrust_req;
        sin_theta_des = max(min(sin_theta_des, 0.5), -0.5);

        %% INNER LOOP

        % =========================================================
        %   ROLL
        % =========================================================
        kp_phi = 40;   kd_phi = 8; 

        phi_des = asin(sin_phi_des);
        % F_drag_y = params.rho * params.s_body_y * params.C_d_y * sign(x(5)) * x(5)^2;
        e_phi = phi_des - phi;
        de_phi = 0 - p; 
        Moment_roll_req = kp_phi * e_phi + kd_phi * de_phi;

        % =========================================================
        %   PITCH
        % =========================================================
        kp_theta = 40; kd_theta = 8; ki_theta = 1;

        theta_des = asin(sin_theta_des);
        e_theta = theta_des - theta;
        de_theta = 0 - q;
        Moment_pitch_req = kp_theta * e_theta + ki_theta * int_err_theta + kd_theta * de_theta;

        % =========================================================
        %   YAW 
        % =========================================================
        kp_psi = 15;   kd_psi = 5; 

        % Calcolo errore angolo (normalizzazione [-pi,pi] 
        e_psi = psi_des - psi;
        e_psi = atan2(sin(e_psi), cos(e_psi));
        
        de_psi = r_des - r;
        
        % Richiesta di Momento Yaw
        Moment_yaw_req = kp_psi * e_psi + kd_psi * de_psi;

        %%
        % =========================================================
        %   MIXING E ALLOCAZIONE 
        % =========================================================
        theta3_ideal = atan2(((-params.d_tx*params.k)/params.b),1);
        theta3_actual = x(17); 
        theta4 = -pi/2;
        
        % --- 1. Mixing Longitudinale (Z + Pitch) ---
        denom_mix = params.d_mx*params.k*sin(theta3_actual) ...
                  - params.d_tx*params.k*sin(theta3_actual) ...
                  + params.b*cos(theta3_actual)*sin(theta4);
        if abs(denom_mix) < 1e-6; denom_mix = 1e-6; end
        
        numeratore_coda = (params.d_mx * Thrust_req) - Moment_pitch_req;
        omega3_sq = numeratore_coda / denom_mix;
        
        F_tail_z = omega3_sq * params.k * sin(theta3_actual);
        
        % Spinta totale richiesta ai motori anteriori (componente Z)
        F_front_tot_z = Thrust_req - F_tail_z;
        % Protezione per evitare divisioni per zero se i motori anteriori sono spenti
        if F_front_tot_z < 0.1; F_front_tot_z = 0.1; end

        omega_front_sq_base = F_front_tot_z / (2 * params.k);
        
        % --- 2. Mixing Laterale (Roll) ---
        braccio_y = params.d_my;
        delta_omega_sq = Moment_roll_req / (params.k * braccio_y * 2);
        
        omega_dx_sq = omega_front_sq_base - delta_omega_sq; 
        omega_sx_sq = omega_front_sq_base + delta_omega_sq; 
        
        % --- 3. Mixing Yaw (Tilt Differenziale) ---
        delta_tilt_yaw = Moment_yaw_req / (F_front_tot_z * params.d_my);
        
        % Saturazione del tilt per sicurezza (es. max 20 gradi = 0.35 rad)
        max_tilt = 0.35; 
        delta_tilt_yaw = max(min(delta_tilt_yaw, max_tilt), -max_tilt);

        % Assegnazione Tilt (Partendo da pi/2 verticale)
        tilt_1 = pi/2 + delta_tilt_yaw; % Motore DX (1)
        tilt_2 = pi/2 - delta_tilt_yaw; % Motore SX (2)

        % le omega non devono diventare negative
        if omega_dx_sq < 0; omega_dx_sq = 0; end
        if omega_sx_sq < 0; omega_sx_sq = 0; end
        if omega3_sq < 0; omega3_sq = 0; end

        u(1) = sqrt(omega_dx_sq);    
        u(2) = sqrt(omega_sx_sq);    
        u(3) = sqrt(omega3_sq);      
        u(4) = tilt_1; 
        u(5) = tilt_2; 
        u(6) = theta3_ideal; 
        u(7) = -pi/2;
        % fprintf('Theta_Des: %.2f deg | Theta_Real: %.2f deg | Omega_Tail: %.2f\n', ...
    % rad2deg(theta_des), rad2deg(x(8)), u(3));



    case 2
        % =========================================================================
        % CONTROLLO ORIZZONTALE: COORDINATED BANK-TO-TURN 
        % Obiettivo: 
        % 1. Outer Loop: Generare Phi_des per azzerare l'errore laterale Y.
        % 2. Inner Loop: Rollare a Phi_des e usare lo Yaw Rate (r) per coordinare.
        % =========================================================================
    
        % --- 1. ESTRAZIONE STATI E VELOCITÀ ---
        % Assumo che x(4:6) siano velocità nel BODY FRAME [u, v, w]
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10);  q = x(11);    r = x(12);
        
        R = matriceRotazione(phi, theta, psi);
        V_glob = R * [x(4); x(5); x(6)]; % Velocità inerziali NED
        vx_global = V_glob(1); 
        vy_global = V_glob(2); 
        vz_global = V_glob(3);
        
        % Calcolo Airspeed e Groundspeed
        V_tas = max(1.0, norm([x(4); x(5); x(6)])); % True Airspeed (dal Body)
        V_ground_speed = sqrt(vx_global^2 + vy_global^2);
        
        % Gestione Integrali (Anti-windup reset se necessario)
        if length(x) >= 30
            int_err_v = x(27); int_err_theta = x(28); 
            int_err_z = x(29); int_err_y = x(30); 
        else
            int_err_v = 0; int_err_theta = 0; int_err_z = 0; int_err_y = 0;
        end
    
        % --- 2. SETPOINT ---
        vx_des    = target(1);
        theta_des = target(2); 
        z_des     = target(3);
        y_des     = 0;         
        e_y       = y_des - x(2); 
    
        % =================================================
        % 3. GUIDA LATERALE (OUTER LOOP -> GENERA PHI)
        % =================================================
        % Usiamo l'errore di posizione laterale per decidere quanto rollare.
        
        Kp_y = 0.05;   
        Kd_y = 0.5;   % Damping sulla velocità laterale globale
        
        % Integrale attivo solo se siamo veloci e vicini al target
        if V_ground_speed > 5 && abs(e_y) < 5
            Ki_y = 0.005; 
        else
            Ki_y = 0.0;
        end
        
        % Legge di controllo: Più sono lontano (e_y), più devo rollare.
        % Se mi sto muovendo veloce lateralmente (vy_global), devo controrollare.
        phi_cmd_raw = (Kp_y * e_y) + (Ki_y * int_err_y) - (Kd_y * vy_global);
        
        max_bank = deg2rad(35); % Limite fisico di rollio
        phi_des = max(-max_bank, min(max_bank, phi_cmd_raw));
    
        % =================================================
        % 4. CONTROLLO LONGITUDINALE (QUOTA E VELOCITÀ)
        % =================================================
        e_z  = z_des - x(3);
        de_z = 0 - vz_global; 
        
        kp_z = 4.0;
        kd_z = 6.0; 
        ki_z = 0.8;
        az_cmd = kp_z * e_z + kd_z * de_z + ki_z * int_err_z;
        
        % Feedforward forces
        F_lift_wing = -0.5 * params.rho * params.s * params.C_l * V_tas^2;
        bank_factor = 1 / max(0.7, cos(phi)); % Più rollo, più devo spingere per non scendere
        
        Fz_req = (params.m * (params.g - az_cmd) + F_lift_wing) * bank_factor;
        Fz_req = max(-100, Fz_req); % Saturazione
        
        % Speed Control -> Genera Thrust X
        e_v = vx_des - vx_global;
        F_drag = 0.5 * params.rho * params.s * params.C_d * V_tas^2;
        
        kp_x = 4.0; 
        ki_x = 2; 
        Fx_req = F_drag + kp_x * e_v + ki_x * int_err_v;
        Fx_req = max(0.1, Fx_req);
        
        % Calcolo angolo ideale dei motori (Tilt)
        alpha_ideal = atan2(Fz_req, Fx_req);
        alpha_lim = max(deg2rad(-25), min(deg2rad(25), alpha_ideal));
        alpha_servo_base = alpha_lim - theta; 
    
        % =================================================
        % 5. CONTROLLO ASSETTO (INNER LOOP)
        % =================================================
        
        % --- PITCH LOOP ---
        kp_theta = 1; 
        kd_theta = 0.15; 
        ki_theta = 0.7;
        u_pitch_angle = kp_theta*(theta_des - theta) + kd_theta*(0 - q) + ki_theta*int_err_theta;
            
        % --- ROLL LOOP ---
        kp_phi = 2.5;  
        kd_phi = 0.6;
        u_roll_angle = kp_phi * (phi_des - phi) + kd_phi * (0 - p);
        
        % --- YAW LOOP: COORDINATED TURN (FIXED) ---
        % Il rateo di imbardata (r) deve soddisfare la cinematica della virata.
        % Formula: r_req = (g / V) * sin(phi) * cos(theta)        
        
        if V_tas > 5.0
            % Calcolo feedforward cinematico
            r_coordinated = (params.g / V_tas) * sin(phi) * cos(theta);
            
            % Se il rollio è piccolo, forziamo r a zero per stabilità in rettilineo
            if abs(phi) < deg2rad(2)
                r_coordinated = 0;
            end
        else
            % A bassa velocità la coordinazione aerodinamica non ha senso
            r_coordinated = 0; 
        end
        
        % Il controllore insegue il rateo r calcolato (Rate Controller)
        kp_r = 1.5;  
        kd_r = 0.5;  
        
        % Errore = Desiderato - Misurato
        % Semplificazione standard: P sul rateo
        u_yaw_thrust = kp_r * (r_coordinated - r);
    
        % =================================================
        % 6. MIXER & OUTPUT
        % =================================================
        T_tot = sqrt(Fx_req^2 + Fz_req^2);
        
        % Allocazione differenziale per Yaw
        % u_yaw_thrust si somma a un motore e sottrae all'altro
        T1 = (T_tot / 2) - u_yaw_thrust; 
        T2 = (T_tot / 2) + u_yaw_thrust; 
        
        % Tilt dei servomotori (Roll differenziale + Pitch collettivo)
        ts1 = alpha_servo_base + u_pitch_angle - u_roll_angle; 
        ts2 = alpha_servo_base + u_pitch_angle + u_roll_angle; 
        
        % Saturazioni Attuatori
        ts1 = max(deg2rad(-15), min(deg2rad(60), ts1));
        ts2 = max(deg2rad(-15), min(deg2rad(60), ts2));
        
        % Conversione Forza -> RPM (o input adimensionale)
        u(1) = sqrt(max(0, T1) / params.k);
        u(2) = sqrt(max(0, T2) / params.k);
        u(3) = 0; 
        u(4) = ts1; 
        u(5) = ts2; 
        u(6) = 0; 
        u(7) = 0;

    case 3
        % =========================================================================
        % CONTROLLO ORIZZONTALE: COORDINATED BANK-TO-TURN (CORRETTO)
        % Obiettivo: 
        % 1. Outer Loop: Generare Phi_des per azzerare l'errore laterale Y.
        % 2. Inner Loop: Rollare a Phi_des e usare lo Yaw Rate (r) per coordinare.
        % =========================================================================
    
        % --- 1. ESTRAZIONE STATI E VELOCITÀ ---
        % Assumo che x(4:6) siano velocità nel BODY FRAME [u, v, w]
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10);  q = x(11);    r = x(12);
        
        R = matriceRotazione(phi, theta, psi);
        V_glob = R * [x(4); x(5); x(6)]; % Velocità inerziali NED
        vx_global = V_glob(1); 
        vy_global = V_glob(2); 
        vz_global = V_glob(3);
        
        % Calcolo Airspeed e Groundspeed
        V_tas = max(1.0, norm([x(4); x(5); x(6)])); % True Airspeed (dal Body)
        V_ground_speed = sqrt(vx_global^2 + vy_global^2);
        
        % Gestione Integrali (Anti-windup reset se necessario)
        if length(x) >= 30
            int_err_v = x(27); int_err_theta = x(28); 
            int_err_z = x(29); int_err_y = x(30); 
        else
            int_err_v = 0; int_err_theta = 0; int_err_z = 0; int_err_y = 0;
        end
    
        % --- 2. SETPOINT ---
        vx_des    = target(1);
        theta_des = target(2); 
        z_des     = target(3);
        y_des     = 0;         
        e_y       = y_des - x(2); 
    
        % =================================================
        % 3. GUIDA LATERALE (OUTER LOOP -> GENERA PHI)
        % =================================================
        % Usiamo l'errore di posizione laterale per decidere quanto rollare.
        
        Kp_y = 0.05;   
        Kd_y = 0.5;   % Damping sulla velocità laterale globale
        
        % Integrale attivo solo se siamo veloci e vicini al target
        if V_ground_speed > 5 && abs(e_y) < 5
            Ki_y = 0.005; 
        else
            Ki_y = 0.0;
        end
        
        % Legge di controllo: Più sono lontano (e_y), più devo rollare.
        % Se mi sto muovendo veloce lateralmente (vy_global), devo controrollare.
        % phi_cmd_raw = (Kp_y * e_y) + (Ki_y * int_err_y) - (Kd_y * vy_global);
        
        max_bank = deg2rad(35); % Limite fisico di rollio
        % phi_des = max(-max_bank, min(max_bank, phi_cmd_raw));

        
        
        % --- 3. GUIDA LATERALE (Outer Loop) ---
        % Vy_global filtrata per evitare che rumore sulla posizione sporchi il rollio
        % (In un sistema reale, vy_global è già una derivata di y)
        phi_cmd_raw = (Kp_y * e_y) + (Ki_y * int_err_y) - (Kd_y * vy_global);
        phi_des = max(-max_bank, min(max_bank, phi_cmd_raw));
    
        % =================================================
        % 4. CONTROLLO LONGITUDINALE (QUOTA E VELOCITÀ)
        % =================================================
        e_z  = z_des - x(3);
        de_z = 0 - vz_global; 
        
        kp_z = 4.0;
        kd_z = 6.0; 
        ki_z = 0.8;
        az_cmd = kp_z * e_z + kd_z * de_z + ki_z * int_err_z;
        
        % Feedforward forces
        F_lift_wing = -0.5 * params.rho * params.s * params.C_l * V_tas^2;
        bank_factor = 1 / max(0.7, cos(phi)); % Più rollo, più devo spingere per non scendere
        
        Fz_req = (params.m * (params.g - az_cmd) + F_lift_wing) * bank_factor;
        Fz_req = max(-100, Fz_req); % Saturazione
        
        % Speed Control -> Genera Thrust X
        e_v = vx_des - vx_global;
        F_drag = 0.5 * params.rho * params.s * params.C_d * V_tas^2;
        
        kp_x = 4.0; 
        ki_x = 2; 
        Fx_req = F_drag + kp_x * e_v + ki_x * int_err_v;
        Fx_req = max(0.1, Fx_req);
        
        % Calcolo angolo ideale dei motori (Tilt)
        alpha_ideal = atan2(Fz_req, Fx_req);
        alpha_lim = max(deg2rad(-25), min(deg2rad(25), alpha_ideal));
        alpha_servo_base = alpha_lim - theta; 
    
        % =================================================
        % 5. CONTROLLO ASSETTO (INNER LOOP)
        % =================================================
        % --- PARAMETRO DI FILTRO (Costante di tempo) ---
        % Costanti di tempo del filtro derivativo (Frequenze di taglio specifiche)
        tau_pitch = 0.04; % Filtro più pesante per smorzare il modo a 1.96 rad/s
        tau_roll  = 0.01; % Filtro leggero per non perdere reattività
        tau_yaw   = 0.02;

        alpha_p = dt / (tau_roll + dt);
        alpha_q = dt / (tau_pitch + dt);
        alpha_r = dt / (tau_yaw + dt);
    
        % p_filt, q_filt, r_filt sono i segnali "puliti" dalle alte frequenze
        % Per la linearizzazione, questo aggiunge un polo al sistema: s = -1/tau
        p_filt = (1 - alpha_p) * x_prev(10) + alpha_p * p; 
        q_filt = (1 - alpha_q) * x_prev(11) + alpha_q * q;
        r_filt = (1 - alpha_r) * x_prev(12) + alpha_r * r;
        
        % --- PITCH LOOP ---
        kp_theta = 1; 
        kd_theta = 0.15; 
        ki_theta = 0.7;
        u_pitch_angle = kp_theta*(theta_des - theta) + kd_theta*(0 - q_filt) + ki_theta*int_err_theta;            
        
        % --- ROLL LOOP ---
        kp_phi = 2.5;  
        kd_phi = 0.6;
        u_roll_angle = kp_phi * (phi_des - phi) + kd_phi * (0 - p_filt);        
        
        % --- YAW LOOP: COORDINATED TURN (FIXED) ---
        % Il rateo di imbardata (r) deve soddisfare la cinematica della virata.
        % Formula: r_req = (g / V) * sin(phi) * cos(theta)        
        
        if V_tas > 5.0
            % Calcolo feedforward cinematico
            r_coordinated = (params.g / V_tas) * sin(phi) * cos(theta);
            
            % Se il rollio è piccolo, forziamo r a zero per stabilità in rettilineo
            if abs(phi) < deg2rad(2)
                r_coordinated = 0;
            end
        else
            % A bassa velocità la coordinazione aerodinamica non ha senso
            r_coordinated = 0; 
        end
        
        % Il controllore insegue il rateo r calcolato (Rate Controller)
        kp_r = 1.5;  
        kd_r = 0.5;  
        
        % Errore = Desiderato - Misurato
        u_yaw_thrust = kp_r * (r_coordinated - r_filt);
    
        % =================================================
        % 6. MIXER & OUTPUT
        % =================================================
        T_tot = sqrt(Fx_req^2 + Fz_req^2);
        
        % Allocazione differenziale per Yaw
        % u_yaw_thrust si somma a un motore e sottrae all'altro
        T1 = (T_tot / 2) - u_yaw_thrust; 
        T2 = (T_tot / 2) + u_yaw_thrust; 
        
        % Tilt dei servomotori (Roll differenziale + Pitch collettivo)
        ts1 = alpha_servo_base + u_pitch_angle - u_roll_angle; 
        ts2 = alpha_servo_base + u_pitch_angle + u_roll_angle; 
        
        % Saturazioni Attuatori
        ts1 = max(deg2rad(-15), min(deg2rad(60), ts1));
        ts2 = max(deg2rad(-15), min(deg2rad(60), ts2));
        
        % Conversione Forza -> RPM (o input adimensionale)
        u(1) = sqrt(max(0, T1) / params.k);
        u(2) = sqrt(max(0, T2) / params.k);
        u(3) = 0; 
        u(4) = ts1; 
        u(5) = ts2; 
        u(6) = 0; 
        u(7) = 0;

    otherwise
        fprintf("Controllo selezionato non trovato\n");
        % Se richiami altri case non definiti qui, metti un default
        u = zeros(7,1);
end
end
