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
        %  Controllo orizzontale
        %  =========================================================================
    
        % ================================================
        % 1. ESTRAZIONE E PREPARAZIONE STATI
        % Stati angolari e ratei
        % =================================================
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10);  q = x(11);    r = x(12);
        
        % =================================================
        % Calcolo Velocità nel riferimento Globale (NED)
        % =================================================
        R = matriceRotazione(phi, theta, psi);
        V_glob = R * [x(4); x(5); x(6)];
        vx_global = V_glob(1); 
        vy_global = V_glob(2);
        vz_global = V_glob(3);
        
        % =================================================
        % Lettura Integrali 
        % =================================================
        if length(x) >= 29
            int_err_v     = x(27); % Integrale Velocità
            int_err_theta = x(28); % Integrale Pitch
            int_err_z     = x(29);
        else
            int_err_v = 0; int_err_theta = 0; int_err_z = 0;
        end
        
        % =================================================
        % 2. DEFINIZIONE SETPOINT (Guida)
        % =================================================
        % Target operativi per la crociera
        z_des     = target(3);       % Quota target (negativa in NED)
        vx_des    = target(1); % Velocità target (es. 30 m/s)
        theta_des = target(2); % Pitch target (solitamente 0° in crociera)
        phi_des = 0;
        psi_des = 0;

        % =================================================
        % ANNULLAMENTO VELOCITA' Y
        % =================================================
        Kp_vy = 0.1; % Guadagno da tarare
        
        % Se ho velocità laterale positiva, devo rollare in negativo per frenare
        phi_des = -Kp_vy * vy_global;
        
        % Saturazione di sicurezza
        phi_des = max(-0.5, min(0.5, phi_des));

        % =================================================
        % 3. CONTROLLO QUOTA (Loop Z - Outer Loop)
        % =================================================
        % Calcolo errori di posizione e velocità verticale
        e_z  = z_des - x(3);
        de_z = 0 - vz_global;
        
        % PID Quota
        kp_z = 3.0; 
        kd_z = 4.0; 
        ki_z = 0.5;
        az_cmd = kp_z * e_z + kd_z * de_z + ki_z * int_err_z;
        
        % =================================================
        % --- Feedforward Aerodinamico (ALA) ---
        % =================================================
        % Calcolo portanza generata dall'ala (negativa in NED perché verso l'alto)
        % L = 0.5 * rho * S * Cl * V^2
        F_lift_wing = -0.5 * params.rho * params.s * params.C_l * vx_global^2;
        
        % Forza verticale richiesta ai motori (F = m*a - L_ala)
        % Se l'ala sostiene tutto il peso, Fz_req tende a 0.
        Fz_req = params.m * (params.g - az_cmd) + F_lift_wing;
        
        % [FIX FISICO]: Gestione Portanza Eccessiva
        % Se l'ala genera troppa portanza (Fz_req < 0), permettiamo ai motori 
        % di spingere verso il basso (fino a -40N) per non salire incontrollati.
        if Fz_req < -100; Fz_req = -100; end 
    
        % =================================================
        % 4. CONTROLLO VELOCITÀ (Loop X - Outer Loop)
        % =================================================
        e_v = vx_des - vx_global;
        
        % PID Velocità
        kp_v = 8.0; 
        ki_v = 4.0; 
        
        % --- Feedforward Aerodinamico (DRAG) ---
        % D = 0.5 * rho * S * Cd * V^2
        F_drag = 0.5 * params.rho * params.s * params.C_d * vx_global^2;
        
        % Forza orizzontale richiesta (deve vincere Drag + Inerzia)
        Fx_req = F_drag + kp_v * e_v + ki_v * int_err_v;
        
        if Fx_req < -60; Fx_req = -60; end
       
        % =================================================
        % 5. STRATEGIA VETTORIALE (Thrust Vectoring)
        % =================================================
        % Calcolo dell'angolo di spinta ideale nel riferimento GLOBALE
        alpha_ideal = atan2(max(0, Fz_req), Fx_req);
        
        % Limitazione dell'angolo di tilt (Safety)
        % se permettiamo un inclinazione fino a 90°, il drone cade
        % Esempio: Permetti tilt da -15° (picchiata) a +120° (frenata all'indietro)
        % Nota: Verifica la tua convenzione. Se 0=Avanti e 90=Su:
        max_tilt_forward = deg2rad(-15); 
        max_tilt_backward = deg2rad(120); % Oltre la verticale per frenare
        
        alpha_limited = max(max_tilt_forward, min(max_tilt_backward, alpha_ideal));
        
        % Sottraiamo il pitch corrente (theta) per ottenere l'angolo relativo ai servi.
        % Se il drone alza il naso (theta > 0), i servi ruotano giù per compensare.
        alpha_servo_base = alpha_limited - theta;
    
        % =================================================
        % 6. CONTROLLO DI ASSETTO (Inner Loop)
        % =================================================

        % =================================================
        % STRATEGIA: ALTEZZA -> PITCH
        % =================================================
        % Recuperiamo il comando di accelerazione verticale calcolato dal PID Quota
        % az_cmd viene dal blocco "3. CONTROLLO QUOTA"
        % Se az_cmd > 0 significa che voglio scendere (in NED) o frenare la salita.
        
        % Guadagno di conversione: Quanti radianti di pitch per ogni m/s^2 di correzione z?
        % Valore negativo: se devo scendere (az_cmd > 0), devo picchiare (theta < 0).
        K_pitch_z = -0.08; 
        
        % Calcolo del Theta Desiderato Dinamico
        theta_base = target(2); % Di solito 0
        theta_correction = K_pitch_z * az_cmd;
        
        % Somma e Saturazione (Safety: non superare +/- 20 gradi)
        theta_des_new = theta_base + theta_correction;
        theta_des_new = max(deg2rad(-20), min(deg2rad(20), theta_des_new));

        e_theta  = theta_des_new - theta;
        de_theta = 0 - q;
        
        % I guadagni del PID Pitch restano quelli di stabilità
        kp_th = 2.0; kd_th = 0.2; ki_th = 1.0; 
        
        u_pitch_angle = kp_th * e_theta + kd_th * de_theta + ki_th * int_err_theta;
        
        % =================================================
        % --- B. ROLL (Rollio) ---
        % =================================================  
        kp_phi = 0.20;  
        kd_phi = 0.10; 

        u_roll_angle = kp_phi * (phi_des - phi) + kd_phi * (0 - p);
        
        
        % =================================================
        % --- C. YAW (Imbardata) ---
        % =================================================      
        kp_psi = 10;  
        kd_psi = 2.5;  

        u_yaw_thrust = kp_psi * (psi_des - psi) + kd_psi * (0 - r);
    
        % =================================================
        % 7. MIXER / ALLOCAZIONE ATTUATORI
        % =================================================        
        % Spinta Totale richiesta (somma vettoriale)
        T_tot = sqrt(Fx_req^2 + Fz_req^2);
  
        
        % Angoli Servo (Tilt Collettivo + Pitch Corr + Diff Roll)
        ts1 = alpha_servo_base + u_pitch_angle - u_roll_angle; 
        ts2 = alpha_servo_base + u_pitch_angle + u_roll_angle; 
        % DEBUG MONITOR
        % if mod(t, 1) < 0.05 % Stampa ogni secondo circa
        %     fprintf('T=%.1f | V=%.1f | Fx_req=%.1f | Alpha=%.1f deg | Tilt_Servo=%.1f deg\n', ...
        %             t, vx_global, Fx_req, rad2deg(alpha_ideal), rad2deg(ts1));
        % end
        
        % Spinte Motori (Spinta Base -/+ Diff Yaw)
        T1 = (T_tot / 2) - u_yaw_thrust; 
        T2 = (T_tot / 2) + u_yaw_thrust; 
        
        % 8. SATURAZIONE E OUTPUT
        
        % Saturazione fisica dei servi (limiti meccanici)
        % Non voglio che i rotori si inclinino all'infinito
        ts1 = max(deg2rad(-30), min(deg2rad(120), ts1));
        ts2 = max(deg2rad(-30), min(deg2rad(120), ts2));

        % =================================================
        % Assegnazione al vettore di controllo u
        % =================================================
        u(1) = sqrt(max(0, T1) / params.k);
        u(2) = sqrt(max(0, T2) / params.k);
        u(3) = 0; % Motore posteriore spento
        
        u(4) = ts1;
        u(5) = ts2;
        u(6) = 0;   
        u(7) = 0;   

        % fprintf('phi %2f, theta %2f, psi %2f\n', phi, theta, psi);
        

    case 3
        % =========================================================================
        %  Controllo orizzontale
        %  =========================================================================
    
        % ================================================
        % 1. ESTRAZIONE E PREPARAZIONE STATI
        % Stati angolari e ratei
        % =================================================
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10);  q = x(11);    r = x(12);
        
        % =================================================
        % Calcolo Velocità nel riferimento Globale (NED)
        % =================================================
        R = matriceRotazione(phi, theta, psi);
        V_glob = R * [x(4); x(5); x(6)];
        vx_global = V_glob(1); 
        vz_global = V_glob(3);
        
        % =================================================
        % Lettura Integrali 
        % =================================================
        if length(x) >= 29
            int_err_v     = x(27); % Integrale Velocità
            int_err_theta = x(28); % Integrale Pitch
            int_err_z     = x(29);
        else
            int_err_v = 0; int_err_theta = 0; int_err_z = 0;
        end
        
        % =================================================
        % 2. DEFINIZIONE SETPOINT (Guida)
        % =================================================
        % Target operativi per la crociera
        z_des     = target(3);       % Quota target (negativa in NED)
        vx_des    = target(1); % Velocità target (es. 30 m/s)
        theta_des = target(2); % Pitch target (solitamente 0° in crociera)
        phi_des = 0;
        psi_des = 0;

        % =================================================
        % 3. CONTROLLO QUOTA (Loop Z - Outer Loop)
        % =================================================
        % Calcolo errori di posizione e velocità verticale
        e_z  = z_des - x(3);
        de_z = 0 - vz_global;
        
        % PID Quota
        kp_z = 4.0; 
        kd_z = 6.0; 
        ki_z = 0.5;
        az_cmd = kp_z * e_z + kd_z * de_z + ki_z * int_err_z;
        
        % =================================================
        % --- Feedforward Aerodinamico (ALA) ---
        % =================================================
        % Calcolo portanza generata dall'ala (negativa in NED perché verso l'alto)
        % L = 0.5 * rho * S * Cl * V^2
        F_lift_wing = -0.5 * params.rho * params.s * params.C_l * vx_global^2;
        
        % Forza verticale richiesta ai motori (F = m*a - L_ala)
        % Se l'ala sostiene tutto il peso, Fz_req tende a 0.
        Fz_req = params.m * (params.g - az_cmd) + F_lift_wing;
        
        % [FIX FISICO]: Gestione Portanza Eccessiva
        % Se l'ala genera troppa portanza (Fz_req < 0), permettiamo ai motori 
        % di spingere verso il basso (fino a -40N) per non salire incontrollati.
        if Fz_req < -80; Fz_req = -80; end 
    
        % =================================================
        % 4. CONTROLLO VELOCITÀ (Loop X - Outer Loop)
        % =================================================
        e_v = vx_des - vx_global;
        
        % PID Velocità
        kp_v = 8.0; 
        ki_v = 4.0; 
        
        % --- Feedforward Aerodinamico (DRAG) ---
        % D = 0.5 * rho * S * Cd * V^2
        F_drag = 0.5 * params.rho * params.s * params.C_d * vx_global^2;
        
        % Forza orizzontale richiesta (deve vincere Drag + Inerzia)
        Fx_req = F_drag + kp_v * e_v + ki_v * int_err_v;
        
        % per evitare singolarità matematiche
        if Fx_req < 0.1; Fx_req = 0.1; end
       
        % =================================================
        % 5. STRATEGIA VETTORIALE (Thrust Vectoring)
        % =================================================
        % Calcolo dell'angolo di spinta ideale nel riferimento GLOBALE
        alpha_ideal = atan2(Fz_req, Fx_req);
        
        % Limitazione dell'angolo di tilt (Safety)
        % se permettiamo un inclinazione fino a 90°, il drone cade
        max_tilt_safe = deg2rad(45); 
        alpha_limited = max(-max_tilt_safe, min(max_tilt_safe, alpha_ideal));
        
        % Sottraiamo il pitch corrente (theta) per ottenere l'angolo relativo ai servi.
        % Se il drone alza il naso (theta > 0), i servi ruotano giù per compensare.
        alpha_servo_base = alpha_limited - theta;
    
        % =================================================
        % 6. CONTROLLO DI ASSETTO (Inner Loop)
        % =================================================

        % =================================================
        % --- A. PITCH (Beccheggio) ---
        % =================================================
        e_theta  = theta_des - theta;
        de_theta = 0 - q;
        
        kp_th = 2; 
        kd_th = 0.3; 
        ki_th = 1.5;  
        
        % L'output del PID pitch è un angolo correttivo da sommare al tilt base
        u_pitch_angle = kp_th * e_theta + kd_th * de_theta + ki_th * int_err_theta;
        
        % =================================================
        % --- B. ROLL (Rollio) ---
        % =================================================  
        kp_phi = 0.20;  
        kd_phi = 0.10; 

        u_roll_angle = kp_phi * (phi_des - phi) + kd_phi * (0 - p);
        
        
        % =================================================
        % --- C. YAW (Imbardata) ---
        % =================================================      
        kp_psi = 10;  
        kd_psi = 2.5;  

        u_yaw_thrust = kp_psi * (psi_des - psi) + kd_psi * (0 - r);
    
        % =================================================
        % 7. MIXER / ALLOCAZIONE ATTUATORI
        % =================================================        
        % Spinta Totale richiesta (somma vettoriale)
        T_tot = sqrt(Fx_req^2 + Fz_req^2);
  
        
        % Angoli Servo (Tilt Collettivo + Pitch Corr + Diff Roll)
        ts1 = alpha_servo_base + u_pitch_angle - u_roll_angle; 
        ts2 = alpha_servo_base + u_pitch_angle + u_roll_angle; 
        
        % Spinte Motori (Spinta Base -/+ Diff Yaw)
        T1 = (T_tot / 2) - u_yaw_thrust; 
        T2 = (T_tot / 2) + u_yaw_thrust; 
        
        % 8. SATURAZIONE E OUTPUT
        
        % Saturazione fisica dei servi (limiti meccanici)
        % Non voglio che i rotori si inclinino all'infinito
        ts1 = max(deg2rad(-30), min(deg2rad(80), ts1));
        ts2 = max(deg2rad(-30), min(deg2rad(80), ts2));
        
        % =================================================
        % Assegnazione al vettore di controllo u
        % =================================================
        u(1) = sqrt(max(0, T1) / params.k);
        u(2) = sqrt(max(0, T2) / params.k);
        u(3) = 0; % Motore posteriore spento
        
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
