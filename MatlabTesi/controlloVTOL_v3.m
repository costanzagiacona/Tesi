function u = controlloVTOL_v3(t, params, x, test_id)

% Preallocazione
u = zeros(7,1);
t = 0;

% test_id = 6;
% TEST

% -1 : Debug (tutto simbolico)

% 0 : Nessun controllo (solo gravità)

% 1 : compensazione attrito lungo X 

% 2 : controllo velocità lungo X e posizione e velocità lungo Z

% 3 : volo verticale (senza momento torcente)  
% NB: per il test 3:
% dmx = -(1/2)*dtx (serve a compensare momento di pitch)
% usare M_thrust_noTorc (tolgo momento torcente)
% mettere il rotore di coda inclinato verticalmente (x0(17)= pi/2; x0(19)=0)

% 4 : volo verticale (con momento torcente)  

% angoli di roll ,pitch, yaw
phi = x(7); theta = x(8); psi = x(9);
p = x(10); q = x(11); r = x(12);
R = matriceRotazione(phi,theta,psi);
V_body = [x(4);x(5);x(6)]; % velocità nel body frame
V_global = R*V_body;
vx_global = V_global(1); 
vy_global = V_global(2);
vz_global = V_global(3);

persistent err_v_int err_z_int;     


switch test_id

    case -1
        %debug
        syms u1 u2 u3 u4 u5 u6 u7
        u = [u1;u2;u3;u4;u5;u6;u7];
        
        % variante in assenza di rotore di coda 
        u = [u1;u2;0;u4;u5;0;0];

    case 0
        % Nessun controllo (solo gravità)
        u = zeros(7,1);

    case 1

        u = zeros(7,1);

        x4eq = 25;

        D=(params.rho*params.s*params.C_d*(x4eq)^2);
  
        omega1_2 = (0.5)*(D/params.k);
        omega2_2 = (0.5)*(D/params.k);
        omega3_2 = (0.0)*(D/params.k);


        u(1) = sqrt(omega1_2);
        u(2) = sqrt(omega2_2);
        u(3) = sqrt(omega3_2);  



    case 2

        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % controllo 

        u = zeros(7,1);
        
        vx_des = 25;
        kp_x = 5;
        F_x_des = params.rho*params.s*params.C_d*sign(x(4))*x(4)^2;
        F_x_des = F_x_des +kp_x*(vx_des-x(4));

        kp_z = -3;
        kd_z = -5;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s*params.C_d_z*sign(x(6))*x(6)^2;      
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD


        % theta_bar = atan2(F_z_des,F_x_des);
        % 
        % u(4) = theta_bar;
        % u(5) =u(4);
        % u(1) = sqrt((F_z_des/(2*sin(u(4))))/params.k);
        % u(2)=u(1);

        T_tot = sqrt(F_x_des^2 + F_z_des^2);
        theta_bar = atan2(F_z_des,F_x_des);

        % Assegna ai due rotori anteriori
        T_i = T_tot/2;

        u(1) = sqrt(T_i/params.k);   % omega1
        u(2) = u(1);                 % omega2
        u(4) = theta_bar;             % tilt rotore 1
        u(5) = theta_bar;             % tilt rotore 2

    
    case 3

        % NB: per questo test annullare momento torcente
        % e dmx = -(1/2)*dtx  (serve a compensare momento di pitch)

        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        %vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % controllo

        u = zeros(7,1);

        % durante la fase di volo verticale, i rotori generano thrust verso
        % l'alto

        u(4)=pi/2;
        u(5)=pi/2;
        u(6)=pi/2;
        u(7)=0;

        kp_z = -5;
        kd_z = -10;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

        T_tot = F_z_des;
        T_i = T_tot/3;

        u(1) = sqrt(T_i/params.k);   % omega1
        u(2) = u(1);                 % omega2
        u(3) = u(1);                 % omega3

    case 4

        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame  

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        %vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % controllo

        u = zeros(7,1);

        kp_z = -5;
        kd_z = -10;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        % con nuovo modello
        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        % con vecchio modello
        % F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s*params.C_d_z*sign(x(6))*x(6)^2;
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

        theta3 = atan2(((-params.d_tx*params.k)/params.b),1);
        omega3_2 = (params.d_mx*F_z_des)/(params.d_mx*params.k*sin(theta3)+params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));
        omega_2= (F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k);

        
        %check
        % term1 = -params.d_tx*params.k*omega3_2*cos(theta3);
        % term2 = -params.b*omega3_2*sin(theta3);
        % term = term1+term2;
        % term3 = 2*params.d_mx*params.k*omega_2;
        % term4 = params.d_tx*params.k*sin(theta3)*omega3_2;
        % term5 = -params.b*cos(theta3)*omega3_2;
        % term = term3+term4+term5;
        % term6 = 2*params.k*omega_2;
        % term7 = F_z_des-params.k*sin(theta3)*omega3_2;
        % term = term6 -term7;

        u(1)=sqrt(omega_2);
        u(2)=u(1);
        u(3)=sqrt(omega3_2);
        u(4)=pi/2;
        u(5)=pi/2;
        u(6)=theta3;
        u(7)=-pi/2;

    case 5
        % --- CONTROLLO VERTICALE ROBUSTO + YAW CON TILT (X, Y, Z, PSI) ---
        u = zeros(7,1);

        % 2. Parametri Obiettivo
        z_des = -10;    vz_des = 0;
        y_des = -10;      vy_des = 0;
        x_des = 0;      vx_des = 0; 
        psi_des = 0;    r_des = 0;  
        % psi_des = 45 * (pi/180);  % ~0.785 radianti

        % 3. Parametri Controllori
        % Z (Quota)
        lambda_z = 2.5; K_z_smc = 60; Phi_z = 0.8;
        % Y (Laterale)
        lambda_y = 0.8; K_y_smc = 15; Phi_y = 1.0;
        % X (Longitudinale)
        lambda_x = 0.8; K_x_smc = 8; Phi_x = 1.0;
        
        % PD Attitudine (Roll & Pitch)
        kp_phi = 40;   kd_phi = 8; 
        kp_theta = 40; kd_theta = 8;

        % NUOVO: PD Yaw (Imbardata)
        kp_psi = 15;   kd_psi = 5; 

        % =========================================================
        %   LOOP Z (QUOTA) - Invariato
        % =========================================================
        e_z = z_des - x(3);           
        de_z = vz_des - vz_global;    
        s_z = de_z + lambda_z * e_z;    
        F_grav = params.m * params.g * cos(theta) * cos(phi); 
        F_drag_z = -params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_lift = -params.C_l*params.rho*params.s*x(4)^2;
        u_smc_z = params.m * lambda_z * de_z + K_z_smc * tanh(s_z / Phi_z);
        Thrust_req = F_grav + F_drag_z + F_lift - u_smc_z;
        if Thrust_req < 1; Thrust_req = 1; end

        % =========================================================
        %   LOOP Y (LATERALE -> ROLLIO) - Invariato
        % =========================================================
        e_y = y_des - x(2);          
        de_y = vy_des - vy_global;   
        s_y = de_y + lambda_y * e_y; 
        F_y_req = params.m * lambda_y * de_y + K_y_smc * tanh(s_y / Phi_y);
        sin_phi_des = F_y_req / Thrust_req;
        sin_phi_des = max(min(sin_phi_des, 0.5), -0.5); 
        phi_des = asin(sin_phi_des);
        F_drag_y = params.rho * params.s_body_y * params.C_d_y * sign(x(5)) * x(5)^2;
        e_phi = phi_des - phi;
        de_phi = 0 - p; 
        Moment_roll_req = kp_phi * e_phi + kd_phi * de_phi;

        % =========================================================
        %   LOOP X (LONGITUDINALE -> PITCH) - Invariato
        % =========================================================
        e_x = x_des - x(1);
        de_x = vx_des - vx_global;
        s_x = de_x + lambda_x * e_x;
        F_x_req = params.m * lambda_x * de_x + K_x_smc * tanh(s_x / Phi_x);
        sin_theta_des = -F_x_req / Thrust_req;
        sin_theta_des = max(min(sin_theta_des, 0.5), -0.5);
        theta_des = asin(sin_theta_des);
        e_theta = theta_des - theta;
        de_theta = 0 - q;
        Moment_pitch_req = kp_theta * e_theta + kd_theta * de_theta;

        % =========================================================
        %   NUOVO LOOP: YAW (IMBARDATA)
        % =========================================================
        % Calcolo errore angolo (gestione wrap -pi/pi opzionale ma consigliata)
        e_psi = psi_des - psi;
        e_psi = atan2(sin(e_psi), cos(e_psi));
        % Se necessario normalizzare tra -pi e pi: e_psi = atan2(sin(e_psi), cos(e_psi));
        
        de_psi = r_des - r;
        
        % Richiesta di Momento Yaw
        Moment_yaw_req = kp_psi * e_psi + kd_psi * de_psi;

        % =========================================================
        %   MIXING E ALLOCAZIONE AGGIORNATA
        % =========================================================
        theta3_ideal = atan2(((-params.d_tx*params.k)/params.b),1);
        theta3_actual = x(17); 
        
        % --- 1. Mixing Longitudinale (Z + Pitch) ---
        denom_mix = params.d_mx*params.k*sin(theta3_actual) ...
                  - params.d_tx*params.k*sin(theta3_actual) ...
                  + params.b*cos(theta3_actual);
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
        % Per generare Yaw, tiltiamo i motori in direzioni opposte.
        % Momento Yaw = (F_motore * sin(tilt)) * braccio_y * 2 (circa)
        % Assumendo piccoli angoli: sin(delta) ~ delta.
        % Forza orizzontale disponibile = F_front_tot_z (approx, assumendo tilt piccoli)
        
        % Calcolo angolo di tilt differenziale richiesto (in radianti)
        % Nota: F_front_tot_z agisce come guadagno di autorità. Più spinta ho, meno tilt serve.
        delta_tilt_yaw = Moment_yaw_req / (F_front_tot_z * params.d_my);
        
        % Saturazione del tilt per sicurezza (es. max 20 gradi = 0.35 rad)
        max_tilt = 0.35; 
        delta_tilt_yaw = max(min(delta_tilt_yaw, max_tilt), -max_tilt);

        % Assegnazione Tilt (Partendo da pi/2 verticale)
        % Segni: Dipendono dalla geometria esatta. 
        % Logica standard: Per Yaw positivo (naso a sinistra), 
        % Motore DX spinge indietro (Tilt > 90), Motore SX spinge avanti (Tilt < 90).
        tilt_1 = pi/2 + delta_tilt_yaw; % Motore DX (1)
        tilt_2 = pi/2 - delta_tilt_yaw; % Motore SX (2)

        % Saturazioni Motori
        if omega_dx_sq < 0; omega_dx_sq = 0; end
        if omega_sx_sq < 0; omega_sx_sq = 0; end
        if omega3_sq < 0; omega3_sq = 0; end

        u(1) = sqrt(omega_dx_sq);    
        u(2) = sqrt(omega_sx_sq);    
        u(3) = sqrt(omega3_sq);      
        u(4) = tilt_1;  % Tilt Destro Modulato
        u(5) = tilt_2;  % Tilt Sinistro Modulato
        u(6) = theta3_ideal; 
        u(7) = -pi/2;
    
    case 6

        % === RIFERIMENTI ===
        V_des = 25;    % Manteniamo questa velocità costante
    
        % === 1. CONTROLLO DI VELOCITA' (CRUISE CONTROL) ===
        % Usiamo un PI (Proporzionale-Integrale) per annullare l'errore statico
        Kp_v = 5;  % Guadagno proporzionale (regola la reattività)
        Ki_v = 3;   % Guadagno integrale (elimina l'errore a regime dovuto al Drag)
        
        % Variabile persistente per l'integratore
        % persistent err_v_int;
        if isempty(err_v_int) || x(1) == 0 % Reset all'inizio
            err_v_int = 0;
        end
        
        % Calcolo errore
        err_v = V_des - x(4);% vx_global;
        
        % Accumulo errore (integrale) 
        err_v_int = err_v_int + err_v * 0.01; % Assumiamo dt approx
        err_v_int = max(-50, min(50, err_v_int)); % Saturazione integrale
        
        % Feedforward: Calcoliamo la spinta necessaria per vincere il Drag a 25m/s
        F_drag = 0.5*params.rho*params.s_body_x*params.C_d_x*sign(x(4))*x(4)^2;
        F_drag_ali = params.rho*params.s*params.C_d*sign(x(4))*x(4)^2;
        
        % Comando Totale di Spinta
        Thrust_cmd = F_drag + F_drag_ali + Kp_v*err_v + Ki_v*err_v_int;
        Thrust_cmd = max(0, Thrust_cmd); % Non possiamo avere spinta negativa
    
        % === 3. ASSEGNAZIONE AI MOTORI ===
        % Ripartiamo la spinta sui due motori anteriori
        T_dx = 0.5 * Thrust_cmd;
        T_sx = 0.5 * Thrust_cmd;
        
        % Saturazione fisica (minimo 0)
        T_dx = max(0, T_dx);
        T_sx = max(0, T_sx);
    
        % Convertiamo Forza in Velocità angolare (rad/s)
        % T = k * omega^2 -> omega = sqrt(T/k)
        omega_1 = sqrt(T_dx / params.k); % Motore DX
        omega_2 = sqrt(T_sx / params.k); % Motore SX
        omega_3 = 0;                     % Coda SPENTA
    
        % === 4. SERVOMOTORI (TILT) ===
        % Tutto bloccato a 0 gradi (spinta orizzontale)
        tilt_1 = 0; 
        tilt_2 = 0;
        
        % Coda 
        tilt_3 = 0;
        tilt_4 = 0;
    
        u = [omega_1; omega_2; omega_3; tilt_1; tilt_2; tilt_3; tilt_4];
    
      
    case 7
        % =================================
        %      VOLO ORIZZONTALE STABILE
        %      SOLO PID - NON ROBUSTO
        % =================================


        % --- PARAMETRI OBIETTIVO ---
        V_des = 25; z_des = -10;
        theta_des = 0;
        phi_des = 0;
        psi_des = 0;
        
        % --- 1. CONTROLLO QUOTA (Fz richiesta) ---
        e_z = z_des - x(3);
        de_z = 0 - vz_global;
        if t == 0; err_z_int = 0; end
        err_z_int = err_z_int + e_z * 0.01;
        err_z_int = max(min(err_z_int, 5), -5);

        % Calcoliamo quanta forza verticale serve per stare a -10m
        % Usiamo un PID che genera un'accelerazione verticale desiderata
        kp_z = 12.0; ki_z = 3.0; kd_z = 8.0;
        az_cmd = kp_z*e_z + ki_z*err_z_int + kd_z*de_z;
        Fz_req = params.m * (params.g - az_cmd); % Forza in Newton verso l'alto

        % --- 2. CONTROLLO VELOCITÀ (Fx richiesta) ---
        e_v = V_des - vx_global;
        F_drag = 0.5 * params.rho * params.s * params.C_d * vx_global^2;
        kp_v = 15.0;
        Fx_req = F_drag + kp_v * e_v;

        % --- 3. VETTORAMENTO (Calcolo Angolo Servo e Spinta Totale) ---
        % Invece di far beccheggiare il drone, incliniamo i motori
        % T_tot = sqrt(Fx^2 + Fz^2)
        Thrust_total = sqrt(Fx_req^2 + Fz_req^2);
        % alpha_base è l'angolo che i motori DEVONO avere per bilanciare Fx e Fz
        alpha_base = atan2(Fx_req, Fz_req); 

        alpha_servo = alpha_base - x(8);

        % --- 4. STABILIZZAZIONE ASSETTO (Inner Loop) ---
        % Teniamo la fusoliera piatta (theta = 0) per ridurre il drag
        % Se theta > 0 (muso su), i servi devono inclinarsi in avanti (alpha aumenta)
        kp_pitch = 1.5; kd_pitch = 0.2;
        u_pitch = kp_pitch * (theta_des - x(8)) - kd_pitch * x(11);

        % --- 5. MIXER FINALE ---
        T_base = Thrust_total / 2;
        
        % Rollio (Differenziale di spinta)
        kp_roll = 20; kd_roll = 5;
        u_roll = kp_roll * (phi_des - x(7)) - kd_roll * x(10);
        
        T_dx = T_base - u_roll;
        T_sx = T_base + u_roll;

        % Tilt (Base vettoriale + correzione Pitch + correzione Yaw)
        u_yaw = 1.0 * (psi_des - x(12)); % Yaw damping
        
        u(4) = alpha_servo + u_pitch + u_yaw; % Servo DX
        u(5) = alpha_servo + u_pitch - u_yaw; % Servo SX
        
        % Output Motori
        u(1) = sqrt(max(0, T_dx) / params.k);
        u(2) = sqrt(max(0, T_sx) / params.k);
        u(3) = 0; % Coda off

    case 8
        % =========================================================
        %  CONTROLLO ORIZZONTALE A CASCATA (SMC -> ANGOLI -> PD)
        %  Posizione (SMC) --> Assetto (PD) --> Mixer
        % =========================================================
        
        % --- 0. AGGIORNAMENTO CINEMATICA ---
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10); q = x(11); r = x(12);
        
        R = matriceRotazione(phi,theta,psi);
        V_body = [x(4);x(5);x(6)];
        V_glob = R*V_body;
        vx_global = V_glob(1); 
        vy_global = V_glob(2);
        vz_global = V_glob(3);

        % --- 1. PARAMETRI OBIETTIVO ---
        x_des = 100;    vx_des = 25; % Esempio: vai a X=100 a 25 m/s
        y_des = 0;      vy_des = 0;  % Resta al centro
        z_des = -10;    vz_des = 0;  % Quota costante
        r_des = 0;   % Naso avanti
        
        % --- 2. TUNING CONTROLLORI ---
        
        % OUTER LOOP (SMC): Posizione -> Forza/Angolo
        % Tuning "Soft" per evitare chattering sugli angoli
        lam_x = 2; K_x = 10.0; Phi_x = 2.0;
        lam_y = 0.8; K_y = 5.0; Phi_y = 1.0;
        lam_z = 3.0; K_z = 25.0; Phi_z = 1.0;
        
        % INNER LOOP (PD): Assetto -> Momento
        % Guadagni alti per risposta rapida
        kp_phi = 40;   kd_phi = 8; 
        kp_theta = 12; kd_theta = 3;
        kp_psi = 15;   kd_psi = 5;

        % =========================================================
        %   A. LOOP Z (QUOTA) -> SPINTA TOTALE
        % =========================================================
        
        e_z  = z_des - x(3);
        de_z = vz_des - vz_global;
        s_z  = de_z + lam_z * e_z;
        
        % Feedforward (Lift & Drag & Gravità)
        % Nota: Lift dipende da VX, non VZ!
        % F_lift = params.C_l * params.rho * params.s * vx_global^2;
        F_lift = 0;
        % Proiezione gravità sull'asse Z del corpo (per compensare inclinazioni)
        F_grav_body = params.m * params.g / (cos(theta)*cos(phi) + 1e-3);
        
        % SMC Output (Accelerazione richiesta compensata)
        u_smc_z = params.m * lam_z * de_z + K_z * tanh(s_z / Phi_z);
        
        % Spinta Totale (Thrust)
        % F_tot = Peso - Lift - Controllo
        Thrust_req = F_grav_body - F_lift - u_smc_z;
        
        % Saturazione Thrust (Minimo 1N per evitare div/0)
        Thrust_req = max(Thrust_req, 1.0);

        % =========================================================
        %   B. LOOP X (LONGITUDINALE) -> PITCH DESIDERATO
        % =========================================================
        
        e_x  = x_des - x(1);
        de_x = vx_des - vx_global;
        s_x  = de_x + lam_x * e_x;
        
        F_drag_x = 0.5 * params.rho * params.s_body_x * params.C_d_x * vx_global^2 * sign(vx_global);
        
        % Forza richiesta lungo X
        F_x_req = F_drag_x + params.m * lam_x * de_x + K_x * tanh(s_x / Phi_x);
        
        % Conversione Forza X -> Angolo Pitch (Theta)
        % Fx ~ -Thrust * sin(theta)  => sin(theta) = -Fx / Thrust
        sin_theta_des = -F_x_req / Thrust_req;
        
        % Saturazione Angolo (Max 30 gradi = 0.5) per sicurezza
        sin_theta_des = max(min(sin_theta_des, 0.5), -0.5);
        theta_des = asin(sin_theta_des);
        
        % PD PITCH: Calcolo Momento Y
        e_theta  = theta_des - theta;
        de_theta = 0 - q;
        Moment_pitch_req = kp_theta * e_theta + kd_theta * de_theta;

        % =========================================================
        %   C. LOOP Y (LATERALE) -> ROLL DESIDERATO
        % =========================================================
        
        e_y  = y_des - x(2);
        de_y = vy_des - vy_global;
        s_y  = de_y + lam_y * e_y;
        
        % Forza richiesta lungo Y
        F_y_req = params.m * lam_y * de_y + K_y * tanh(s_y / Phi_y);
        
        % Conversione Forza Y -> Angolo Roll (Phi)
        % Fy ~ Thrust * sin(phi)
        sin_phi_des = F_y_req / Thrust_req;
        
        % Saturazione Angolo
        sin_phi_des = max(min(sin_phi_des, 0.5), -0.5);
        phi_des = asin(sin_phi_des);
        
        % PD ROLL: Calcolo Momento X
        e_phi  = phi_des - phi;
        de_phi = 0 - p;
        Moment_roll_req = kp_phi * e_phi + kd_phi * de_phi;

        % =========================================================
        %   D. LOOP PSI (YAW) -> MOMENTO Z
        % =========================================================
        psi_des = 0;
        % psi_des = 45 * (pi/180);

        e_psi = psi_des - psi;
        % Gestione wrapping angolo (-pi a pi)
        e_psi = atan2(sin(e_psi), cos(e_psi)); 
        de_psi = r_des - r;
        
        % PD YAW
        Moment_yaw_req = kp_psi * e_psi + kd_psi * de_psi;
        

        % =========================================================
        %   E. MIXER & ALLOCAZIONE (CORRETTO CON SEGNI)
        % =========================================================
        
        d_y = params.d_my; 
        h_z = params.d_mz;         
        
        % --- PARAMETRI DI SEGNO (TUNING) ---
        % Se il drone diverge (va all'infinito), INVERTI questi segni.
        SIGN_ROLL  = -1;  % Prova 1 o -1 (Scambia Motore DX/SX)
        SIGN_PITCH =  1;  % Prova 1 o -1 (Inverte il senso del Tilt)
        SIGN_YAW   =  -1;  % Prova 1 o -1
        
        % 1. ROLL -> Spinta Differenziale
        % M_x = (T_sx - T_dx) * d_y
        % Se SIGN_ROLL è sbagliato, il drone ruoterà sempre più forte in Roll.
        delta_T_roll = (Moment_roll_req * SIGN_ROLL) / (2 * d_y);
        
        % 2. PITCH -> Tilt Collettivo
        % Se SIGN_PITCH è sbagliato, il drone farà capriole in avanti/indietro.
        % NOTA: Ho rimosso il "meno" esplicito e uso SIGN_PITCH per decidere.
        sin_alpha_pitch = (Moment_pitch_req * SIGN_PITCH) / (Thrust_req * h_z + 1e-3);
        
        % Saturazione Argomento asin
        sin_alpha_pitch = max(min(sin_alpha_pitch, 0.8), -0.8);
        alpha_coll = asin(sin_alpha_pitch);
        
        % 3. YAW -> Tilt Differenziale
        sin_alpha_yaw = (Moment_yaw_req * SIGN_YAW) / (Thrust_req * d_y + 1e-3);
        sin_alpha_yaw = max(min(sin_alpha_yaw, 0.5), -0.5);
        alpha_diff = asin(sin_alpha_yaw);
        
        % --- CALCOLO FINALE ATTUATORI ---
        
        T_dx = (Thrust_req / 2) - delta_T_roll;
        T_sx = (Thrust_req / 2) + delta_T_roll;
        
        alpha_dx = alpha_coll + alpha_diff;
        alpha_sx = alpha_coll - alpha_diff;
        
        % --- SATURAZIONI DI SICUREZZA ---
        T_dx = max(0, T_dx);
        T_sx = max(0, T_sx);
        
        % Limite Servi ridotto per evitare stalli aerodinamici simulati
        lim_servo = 0.6; % +/- 35 gradi
        alpha_dx = max(min(alpha_dx, lim_servo), -lim_servo);
        alpha_sx = max(min(alpha_sx, lim_servo), -lim_servo);
        
        % Output U
        u(1) = sqrt(T_dx / params.k);
        u(2) = sqrt(T_sx / params.k);
        u(3) = 0; 
        u(4) = alpha_dx;
        u(5) = alpha_sx;
        u(6) = 0; u(7) = 0;


    case 9
    % =========================================================
    %  CONTROLLO ORIZZONTALE A CASCATA (CORRETTO & STABILIZZATO)
    %  SMC (Posizione Globale) -> Rotazione -> PD (Assetto) -> Mixer
    % =========================================================

    % --- 0. AGGIORNAMENTO CINEMATICA ---
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);

    R = matriceRotazione(phi,theta,psi);
    V_body = [x(4);x(5);x(6)];
    V_glob = R*V_body;
    vx_global = V_glob(1); 
    vy_global = V_glob(2);
    vz_global = V_glob(3);

    % --- 1. PARAMETRI OBIETTIVO ---
    x_des = 100;    vx_des = 25; % Target: vai a X=100
    y_des = 0;      vy_des = 0;  % Target: resta su Y=0
    z_des = -10;    vz_des = 0;  % Target: Quota -10m
    
    % --- 2. TUNING CONTROLLORI (Rilassati per stabilità) ---
    
    % OUTER LOOP (SMC): Posizione -> Forza
    % Valori ridotti per evitare scatti violenti
    lam_x = 1.5; K_x = 3.0; Phi_x = 2.0;
    lam_y = 1.5; K_y = 3.0; Phi_y = 2.0;
    lam_z = 2.0; K_z = 10.0; Phi_z = 1.0; 
    
    % INNER LOOP (PD): Assetto -> Momento
    kp_phi = 25;   kd_phi = 5; 
    kp_theta = 25; kd_theta = 5;
    kp_psi = 15;   kd_psi = 4;

    % =========================================================
    %   A. LOOP Z (QUOTA) -> SPINTA TOTALE
    % =========================================================
    
    e_z  = z_des - x(3);
    de_z = vz_des - vz_global;
    s_z  = de_z + lam_z * e_z;
    
    % Feedforward Gravità compensata dal Tilt corrente
    F_grav_body = params.m * params.g / (cos(theta)*cos(phi) + 1e-3);
    
    % SMC Output Z
    u_smc_z = params.m * lam_z * de_z + K_z * tanh(s_z / Phi_z);
    
    % Spinta Totale (Thrust)
    Thrust_req = F_grav_body - u_smc_z;
    Thrust_req = max(Thrust_req, 1.0); % Saturazione minima

    % =========================================================
    %   B & C. LOOP XY (POSIZIONE) -> PITCH & ROLL
    %   Logica: Calcolo Forza Globale -> Ruoto in Body -> Calcolo Angoli
    % =========================================================
    
    % 1. Calcolo richieste nel sistema GLOBALE
    
    % --- Asse X Globale ---
    e_x  = x_des - x(1);
    de_x = vx_des - vx_global;
    s_x  = de_x + lam_x * e_x;
    
    F_drag_x = 0.5 * params.rho * params.s_body_x * params.C_d_x * vx_global^2 * sign(vx_global);
    u_smc_x_glob = params.m * lam_x * de_x + K_x * tanh(s_x / Phi_x);
    F_x_glob_req = F_drag_x + u_smc_x_glob;

    % --- Asse Y Globale ---
    e_y  = y_des - x(2);
    de_y = vy_des - vy_global;
    s_y  = de_y + lam_y * e_y;
    
    u_smc_y_glob = params.m * lam_y * de_y + K_y * tanh(s_y / Phi_y);
    F_y_glob_req = u_smc_y_glob;

    % 2. ROTAZIONE VETTORE FORZA (Globale -> Body)
    % Usiamo la matrice di rotazione inversa (trasposta) su Z
    % Questo proietta la forza desiderata "Nord/Est" sugli assi "Naso/Destra" del drone
    
    c_psi = cos(psi);
    s_psi = sin(psi); % Nota: Nessun segno meno qui! La matematica fa il resto.
    
    F_x_body_req =  F_x_glob_req * c_psi + F_y_glob_req * s_psi;
    F_y_body_req = -F_x_glob_req * s_psi + F_y_glob_req * c_psi;

    % 3. CONVERSIONE FORZA BODY -> ANGOLI PITCH/ROLL
    
    % Pitch (Theta) gestisce la forza X Body
    % Per andare avanti (X+), il drone deve picchiare giù (Theta negativo)
    sin_theta_des = -F_x_body_req / Thrust_req;
    sin_theta_des = max(min(sin_theta_des, 0.5), -0.5); % Max ~30 gradi
    theta_des = asin(sin_theta_des);
    
    % Roll (Phi) gestisce la forza Y Body
    % Per andare a destra (Y+), il drone deve rollare a destra (Phi positivo)
    sin_phi_des = F_y_body_req / Thrust_req;
    sin_phi_des = max(min(sin_phi_des, 0.5), -0.5);
    phi_des = asin(sin_phi_des);

    % --- LOOP PD ASSETTO ---
    
    % PD Pitch
    e_theta  = theta_des - theta;
    de_theta = 0 - q;
    Moment_pitch_req = kp_theta * e_theta + kd_theta * de_theta;
    
    % PD Roll
    e_phi  = phi_des - phi;
    de_phi = 0 - p;
    Moment_roll_req = kp_phi * e_phi + kd_phi * de_phi;

    % =========================================================
    %   D. LOOP PSI (YAW) -> MOMENTO Z
    % =========================================================
    
    % Qui impostiamo l'orientamento del naso.
    % Prova 0 prima. Se stabile, prova 45 * (pi/180).
    psi_des = 45 * (pi/180); 
    r_des = 0;

    e_psi = psi_des - psi;
    e_psi = atan2(sin(e_psi), cos(e_psi)); % Gestione wrapping angolo
    de_psi = r_des - r;
    
    Moment_yaw_req = kp_psi * e_psi + kd_psi * de_psi;

    % =========================================================
    %   E. MIXER & ALLOCAZIONE (CRITICO)
    % =========================================================
    
    d_y = params.d_my; 
    h_z = params.d_mz;         
    
    % --- CONFIGURAZIONE SEGNI ---
    % IMPORTANTE: Se il drone diverge su un asse, inverti SOLO quel segno.
    
    SIGN_ROLL  =  1;  % Se oscilla lateralmente (DX/SX), prova -1
    SIGN_PITCH =  1;  % Se fa capriole avanti/indietro, prova -1
    SIGN_YAW   =  1;  % Se ruota su se stesso senza fermarsi, prova -1
    
    % 1. ROLL -> Spinta Differenziale (Motore SX vs DX)
    % M_roll > 0 => Roll a DX => SX aumenta, DX diminuisce
    delta_T_roll = (Moment_roll_req * SIGN_ROLL) / (2 * d_y);
    
    % 2. PITCH -> Tilt Collettivo (Entrambi i servi avanti/indietro)
    % M_pitch > 0 => Naso SU => Servi tirano indietro (segno dipende dai servi)
    sin_alpha_pitch = (Moment_pitch_req * SIGN_PITCH) / (Thrust_req * h_z + 1e-3);
    sin_alpha_pitch = max(min(sin_alpha_pitch, 0.8), -0.8);
    alpha_coll = asin(sin_alpha_pitch);
    
    % 3. YAW -> Tilt Differenziale (Servo SX vs DX)
    sin_alpha_yaw = (Moment_yaw_req * SIGN_YAW) / (Thrust_req * d_y + 1e-3);
    sin_alpha_yaw = max(min(sin_alpha_yaw, 0.5), -0.5);
    alpha_diff = asin(sin_alpha_yaw);
    
    % --- CALCOLO U (OUTPUT AI MOTORI) ---
    
    T_dx = (Thrust_req / 2) - delta_T_roll;
    T_sx = (Thrust_req / 2) + delta_T_roll;
    
    alpha_dx = alpha_coll + alpha_diff;
    alpha_sx = alpha_coll - alpha_diff;
    
    % Saturazioni fisiche
    T_dx = max(0, T_dx);
    T_sx = max(0, T_sx);
    
    lim_servo = 0.6; % +/- 35 gradi circa
    alpha_dx = max(min(alpha_dx, lim_servo), -lim_servo);
    alpha_sx = max(min(alpha_sx, lim_servo), -lim_servo);
    
    % Assegnazione vettore di controllo
    u(1) = sqrt(T_dx / params.k);
    u(2) = sqrt(T_sx / params.k);
    u(3) = 0; 
    u(4) = alpha_dx;
    u(5) = alpha_sx;
    u(6) = 0; u(7) = 0;

    otherwise
        error('Controllo non valido');
end

end
