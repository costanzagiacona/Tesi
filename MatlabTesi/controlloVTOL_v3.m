function u = controlloVTOL_v3(params, x, test_id)

% Preallocazione
u = zeros(7,1);

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
V_global = R*V_body ;
vx_global = V_global(1); 
vy_global = V_global(2);
vz_global = V_global(3);

persistent err_v_int;
persistent err_z_int;

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
        
        % 1. Estrazione Stato

        
        V_body = [x(4);x(5);x(6)]; 
        V_global = R*V_body ;
        vx_global = V_global(1); vy_global = V_global(2); vz_global = V_global(3);
        u = zeros(7,1);

        % 2. Parametri Obiettivo
        z_des = -10;    vz_des = 0;
        y_des = 0;      vy_des = 0;
        x_des = 0;      vx_des = 0; 
        psi_des = 0;    r_des = 0;  % NUOVO: Obiettivo Yaw
        % psi_des = 45 * (pi/180);  % ~0.785 radianti

        % 3. Parametri Controllori
        % ... (Z, Y, X rimangono uguali) ...
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
        % === STATI ===
        V_body = [x(4);x(5);x(6)]; 
        V_global = R*V_body ;
        vx_global = V_global(1);
        vy_global = V_global(2);
        vz_global = V_global(3);

        p = x(10);     % Rate di rollio (per smorzamento)
        r = x(12);     % Rate di imbardata (per smorzamento)
    
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
    % --- CONTROLLO ORIZZONTALE (CRUISE - STABILE & PRECISO) ---
    % Versione finale: Oscillazioni rimosse + Integratore Quota aggiunto
    
    % 1. Parametri Obiettivo
    V_des  = 25;      
    z_des  = -10;     
    vz_des = 0;
    y_des  = 0;       

    % =========================================================
    %   A. LOOP LONGITUDINALE (Thrust & Pitch)
    % =========================================================

    % -- 1. Controllo Velocità -> Spinta (Thrust) --
    e_v = V_des - x(4); 
    
    % Integratore Velocità
    if x(1) == 0; err_v_int = 0; end
    err_v_int = err_v_int + e_v * 0.01; 
    err_v_int = max(min(err_v_int, 20), -20);

    F_drag = 0.5 * params.rho * params.s_body_x * params.C_d_x * x(4)^2 + ...
             params.rho * params.s * params.C_d * x(4)^2;

    Kp_v = 10; Ki_v = 2;
    Thrust_req = F_drag + Kp_v*e_v + Ki_v*err_v_int;
    Thrust_req = max(Thrust_req, 1.0); 

    % -- 2. Controllo Quota -> Pitch (Theta) --
    e_z = z_des - x(3);        
    de_z = vz_des - vz_global; 
    
    % [NOVITA'] Integratore sulla Quota (err_z_int)
    % Serve a mantenere z=-10 esatto anche se il peso non è perfettamente bilanciato
    if x(1) == 0; err_z_int = 0; end
    err_z_int = err_z_int + e_z * 0.01;
    err_z_int = max(min(err_z_int, 5), -5); % Anti-windup piccolo

    % PID Quota (Lento ma preciso con l'integratore)
    kp_z = 0.005;  
    ki_z = 0.001;  % Termine nuovo per precisione
    kd_z = 0.02;   

    % Theta di trim (Angolo base di volo)
    theta_trim = 0.05; 
    
    theta_des = theta_trim + kp_z*e_z + ki_z*err_z_int + kd_z*de_z;
    theta_des = max(min(theta_des, 0.18), -0.18); % Limitiamo a ~10 gradi

    % -- 3. Rate Loop Pitch --
    e_theta = theta_des - theta;

    % Tuning Finale Pitch:
    % Kp alzato leggermente (da 2.5 a 3.0) per più precisione ora che è stabile
    kp_pitch = 3.0;  
    kd_pitch = 0.9;  

    Moment_pitch_req = kp_pitch*e_theta - kd_pitch*q;

    % =========================================================
    %   B. LOOP LATERALE (Roll & Yaw)
    % =========================================================

    e_y = y_des - x(2);
    kp_y = 0.12; kd_y = 0.3; % Kp_y ridotto leggermente per evitare scatti iniziali

    phi_des = -(kp_y * e_y + kd_y * vy_global);
    
    % Limite Roll ridotto per entrate più morbide
    phi_des = max(min(phi_des, 0.4), -0.4); 

    e_phi = phi_des - phi;
    kp_roll = 18; kd_roll = 3.5; 
    Moment_roll_req = kp_roll*e_phi - kd_roll*p;

    % Yaw Damping
    r_des = 0; 
    kp_yaw = 15; 
    Moment_yaw_req = kp_yaw * (r_des - r);

    % =========================================================
    %   C. MIXER (Tricopter Tilt-Front)
    % =========================================================

    T_base = Thrust_req / 2;
    d_y = params.d_my; 
    delta_T_yaw = Moment_yaw_req / d_y;

    T_dx = T_base - delta_T_yaw;
    T_sx = T_base + delta_T_yaw;
    T_dx = max(0.1, T_dx); T_sx = max(0.1, T_sx);

    d_x = params.d_mx; 
    F_z_pitch = Moment_pitch_req / d_x; 
    F_z_roll = Moment_roll_req / (d_y * 2);

    T_calc = T_dx + T_sx; 
    tilt_pitch = F_z_pitch / (T_calc/2); 
    tilt_roll = F_z_roll / (T_calc/2);

    tilt_1_dx = tilt_pitch - tilt_roll; 
    tilt_2_sx = tilt_pitch + tilt_roll; 

    % Limite Servo (Soft saturation)
    lim_tilt = 0.20; 
    tilt_1_dx = max(min(tilt_1_dx, lim_tilt), -lim_tilt);
    tilt_2_sx = max(min(tilt_2_sx, lim_tilt), -lim_tilt);

    % Output
    u(1) = sqrt(T_dx / params.k); 
    u(2) = sqrt(T_sx / params.k); 
    u(3) = 0;                     
    u(4) = tilt_1_dx;             
    u(5) = tilt_2_sx;             
    u(6) = 0; 
    u(7) = 0;

    case 8
    % =========================================================================
    %   CASE 8: ROBUST SMC + DYNAMIC ALLOCATION (FUSIONE)
    %   Obiettivo: Volo stabile senza alettoni, usando i motori in modo ottimale.
    % =========================================================================
    
    % --- 1. SETPOINTS ---
    V_des  = 25;      % m/s
    z_des  = -10;     % m (quota di volo)
    y_des  = 0;       % m
    psi_des = 0;      % rad
    
    % --- 2. GUIDANCE SMC (Calcolo Forze/Momenti Desiderati) ---
    % Nota: Usiamo 'phi_smc' come "Boundary Layer" per ammorbidire il chattering.
    
    % A. Controllo Velocità -> Forza X (Fx)
    e_v = V_des - x(4);
    phi_v = 2.0; % Boundary layer ampio per la velocità
    K_v = 20;    % Guadagno spinta (Newton)
    
    % Stima Drag (Feedforward)
    F_drag = 0.5 * 1.225 * params.s * 0.04 * x(4)^2; 
    
    % SMC Legge: F = F_drag + K * tanh(e / phi)
    F_x_req = F_drag + K_v * tanh(e_v / phi_v);
    F_x_req = max(F_x_req, 1.0); % Mai negativo
    
    % B. Controllo Quota -> Pitch Desiderato -> Momento Pitch (My)
    e_z = z_des - x(3);
    vz = x(6);
    
    % Superficie di scorrimento quota (S_z)
    lambda_z = 0.8; 
    S_z = e_z - (vz / lambda_z); % Rallentato per evitare spike
    
    % Output: Pitch Desiderato
    K_theta = 0.3; % Max pitch command (rad) ~17 deg
    theta_des = K_theta * tanh(S_z) + 0.05; % +0.05 trim
    
    % Loop Interno Pitch (SMC)
    e_theta = theta_des - theta;
    q_rate = q;
    
    % Superficie Pitch
    lambda_q = 4.0;
    S_q = lambda_q * e_theta - q_rate;
    
    % Momento Pitch Richiesto (My)
    K_My = 5.0; % Nm (Newton per metro)
    phi_My = 0.5; % Boundary layer
    M_y_req = K_My * tanh(S_q / phi_My);
    
    % C. Controllo Laterale -> Roll Desiderato -> Momento Roll (Mx)
    e_y = y_des - x(2);
    vy = x(5);
    
    % Superficie Y
    lambda_y = 0.6;
    S_y = e_y - (vy / lambda_y);
    
    % Roll Desiderato
    K_phi = 0.4; % Max roll (rad) ~23 deg
    phi_des_val = -K_phi * tanh(S_y); % Segno meno per coordinata Y
    
    % Loop Interno Roll (SMC)
    e_phi_roll = phi_des_val - phi;
    p_rate = p;
    
    lambda_p = 5.0;
    S_p = lambda_p * e_phi_roll - p_rate;
    
    K_Mx = 4.0; % Nm
    phi_Mx = 0.5;
    M_x_req = K_Mx * tanh(S_p / phi_Mx);
    
    % D. Controllo Yaw -> Momento Yaw (Mz)
    e_psi = psi_des - x(1);
    r_rate = r;
    
    lambda_r = 3.0;
    S_r = lambda_r * e_psi - r_rate;
    
    K_Mz = 3.0; % Nm
    phi_Mz = 0.5;
    M_z_req = K_Mz * tanh(S_r / phi_Mz);

    % =========================================================================
    %   3. DYNAMIC ALLOCATION (Mixer Geometrico)
    %   Adattato da Mousaei et al. per Tri-Rotor
    % =========================================================================
    
    % Vettore Wrench Richiesto
    W_des = [F_x_req; M_x_req; M_y_req; M_z_req];
    
    % Geometria (DA VERIFICARE SUL TUO MODELLO!)
    L_y = 0.5; % Braccio laterale
    L_x = 0.3; % Braccio longitudinale
    L_r = 0.6; % Braccio coda
    
    T_op = F_x_req / 2; % Spinta operativa approssimata per motore
    
    % Coefficienti Efficacia
    K_roll_tilt  = T_op * L_y; 
    K_pitch_tilt = T_op * L_x;
    K_pitch_rear = L_r;
    K_yaw_diff   = L_y;
    
    % Matrice A (Linearizzata)
    % Cols: [T_dx, T_sx, T_rear, d_dx, d_sx]
    % ATTENZIONE AI SEGNI QUI SOTTO:
    % Rollio: Se Servo DX su -> Fz pos -> Ala DX su -> Roll NEGATIVO.
    % Quindi nella colonna d_dx mettiamo un termine negativo per Mx.
    
    A = [ ...
        1,            1,            0,             0,             0;            ... % Fx
        0,            0,            0,             -K_roll_tilt,  K_roll_tilt;  ... % Mx (Roll)
        0,            0,            K_pitch_rear,  K_pitch_tilt,  K_pitch_tilt; ... % My (Pitch)
        K_yaw_diff,   -K_yaw_diff,  0,             0,             0             ... % Mz (Yaw)
    ];
    
    % Risoluzione Pseudo-Inversa
    u_raw = pinv(A) * W_des;
    
    % Estrazione Comandi
    T_dx_raw = u_raw(1);
    T_sx_raw = u_raw(2);
    T_rr_raw = u_raw(3);
    d_dx_raw = u_raw(4);
    d_sx_raw = u_raw(5);
    
    % =========================================================================
    %   4. SATURAZIONI FISICHE (Safety Layer)
    % =========================================================================
    
    % Motori Anteriori (mai sotto il minimo per mantenere autorità tilt)
    T_dx = max(1.0, min(T_dx_raw, 35));
    T_sx = max(1.0, min(T_sx_raw, 35));
    
    % Motore Posteriore (solo spinta positiva)
    T_rr = max(0.0, min(T_rr_raw, 20));
    
    % Servi Tilt (Limiti meccanici in volo)
    % 0.05 rad è il trim (~3 gradi su)
    % Limiti: -15 deg (-0.26 rad) a +45 deg (0.78 rad)
    d_dx = max(-0.26, min(d_dx_raw + 0.05, 0.78));
    d_sx = max(-0.26, min(d_sx_raw + 0.05, 0.78));
    
    % =========================================================================
    %   5. OUTPUT
    % =========================================================================
    u(1) = sqrt(T_dx / params.k); 
    u(2) = sqrt(T_sx / params.k); 
    u(3) = sqrt(T_rr / params.k); % Motore posteriore ATTIVO per pitch
    u(4) = d_dx;
    u(5) = d_sx;
    u(6) = 0; 
    u(7) = 0;
    
    otherwise
        error('Controllo non valido');
end

end
