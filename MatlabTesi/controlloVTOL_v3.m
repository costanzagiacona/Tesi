function u = controlloVTOL_v3(t, params, x, test_id)

% Preallocazione
u = zeros(7,1);
time = 0;

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
   
persistent integral_error_theta
persistent err_z_int err_v_int;

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
        y_des = 0;      vy_des = 0;
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
       

        % Tilt (Base vettoriale + correzione Pitch + correzione Yaw)
        u_yaw = 1.0 * (psi_des - x(12)); % Yaw damping
        
        u(4) = alpha_servo + u_pitch + u_roll; % Servo DX
        u(5) = alpha_servo + u_pitch - u_roll; % Servo SX
        T_dx = T_base - u_yaw;
        T_sx = T_base + u_yaw;
        
        % Output Motori
        u(1) = sqrt(max(0, T_dx) / params.k);
        u(2) = sqrt(max(0, T_sx) / params.k);
        u(3) = 0; % Coda off

    case 8
        % =========================================================
        %  CONTROLLO VOLO ORIZZONTALE 
        % =========================================================
    
        % --- 0. DATI INIZIALI ---
        % Estrazione stati
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10); q = x(11); r = x(12);
    
        % Velocità nel frame inerziale (necessaria per SMC posizionale)
        R = matriceRotazione(phi,theta,psi);
        V_body = [x(4);x(5);x(6)];
        V_glob = R*V_body;
        vx_global = V_glob(1); 
        vy_global = V_glob(2);
        vz_global = V_glob(3);
    
    
        % --- 1. TARGET ---
        vx_des = 25;       % Velocità crociera
        z_des  = -10;      % Quota (NED: -10 = 10m altezza)
        y_des  = 0;        % Centro linea
        theta_des = 0;
        phi_des = 0;
        psi_des = 0;       % Prua

        % --- 1. CONTROLLO QUOTA (Fz richiesta) ---
        e_z = z_des - x(3);
        de_z = 0 - vz_global;
        if time == 0; err_z_int = 0; end
        err_z_int = err_z_int + e_z * 0.01;
        err_z_int = max(min(err_z_int, 5), -5);

        % Calcoliamo quanta forza verticale serve per stare a -10m
        % Usiamo un PID che genera un'accelerazione verticale desiderata
        kp_z = 10.0; ki_z = 3.0; kd_z = 8.0;
        az_cmd = kp_z*e_z + ki_z*err_z_int + kd_z*de_z;
        F_lift = -0.5 * params.rho * params.s * params.C_l * vx_global^2;
        Fz_req = params.m * (params.g - az_cmd) + F_lift; % Forza in Newton verso l'alto

        % --- 2. CONTROLLO VELOCITÀ (Fx richiesta) ---
        e_v = vx_des - vx_global;
        F_drag = 0.5 * params.rho * params.s * params.C_d * vx_global^2;
        kp_v = 15.0;
        Fx_req = F_drag + kp_v * e_v;

        % --- 3. VETTORAMENTO (Calcolo Angolo Servo e Spinta Totale) ---
        % Invece di far beccheggiare il drone, incliniamo i motori
        % T_tot = sqrt(Fx^2 + Fz^2)
        T_total = sqrt(Fx_req^2 + Fz_req^2);

        % =====================================================
        %   TUNING PARAMETRI
        % =====================================================

        K_theta = 2.0; Phi_theta = 1.0; lam_theta = 3.0;
        K_phi = 2.0; Phi_phi = 1.0; lam_phi = 3.0;
        K_psi = 2.0; Phi_psi = 1.0; lam_psi = 3.0;

        % ==================================================
        %   PITCH con INTEGRALE PROTETTO
        % ==================================================
        
        % Gestione memoria variabile
        % persistent integral_error_theta
        if isempty(integral_error_theta)
            integral_error_theta = 0;
        end
    
        dt = 0.01; 
        e_theta = theta_des - theta;
        de_theta = 0 - q;
        
        % --- PROTEZIONE 1: Anti-Windup (Clamping) ---
        % Impediamo all'integrale di diventare un numero mostruoso
        max_integral = 10; % Valore di saturazione dell'integrale
        integral_error_theta = integral_error_theta + e_theta * dt;
        integral_error_theta = max(-max_integral, min(max_integral, integral_error_theta));
        
        % Tuning
        Ki_theta = 0.5; % Abbassalo un po' all'inizio
        
        % Superficie di scorrimento (senza integrale dentro, per ora)
        s_theta = de_theta + lam_theta * e_theta;
        
        % Legge di controllo
        M_y_req = params.Iyy * lam_theta * de_theta ...
                + params.Iyy * K_theta * tanh(s_theta/Phi_theta) ...
                + Ki_theta * integral_error_theta; 
                
        u_pitch = M_y_req / (T_total * params.d_mx); 
    

        % ==================================================
        %   ROLL
        % ==================================================
    
        e_phi = phi_des - phi;
        de_phi = 0 - p;
        s_phi = de_phi + lam_phi * e_phi;

        M_x_req = params.Ixx * lam_phi * de_phi + params.Ixx * K_phi * tanh(s_phi/Phi_phi);
        u_roll = M_x_req / (T_total * params.d_my); 

        % ==================================================
        %   Yaw
        % ==================================================
    
        e_psi = psi_des - psi;
        de_psi = 0 - r;
        s_psi = de_psi + lam_psi * e_psi;

        M_z_req = params.Izz * lam_psi * de_psi + params.Izz * K_psi * tanh(s_psi/Phi_psi);
        u_yaw = M_z_req / (2 * params.d_my);

        % 4. MIXER (Configurazione AEREO)
        % Roll -> Tilt Differenziale
        % Yaw  -> Spinta Differenziale
        
        % Angolo base dei servi (per compensare l'assetto del corpo, come da tuoi appunti)
        alpha_base = 0; % Angolo di montaggio standard (es. 0 o 90 gradi)
        alpha_servo = alpha_base; %- theta; % Compensazione attiva del beccheggio fusoliera
   

        % Calcolo angoli servi (in radianti)
        % Motore 1 (destro), Motore 2 (sinistro)
        theta_servo_1 = alpha_servo + u_pitch + u_roll; 
        theta_servo_2 = alpha_servo + u_pitch - u_roll; 
    
        % --- PROTEZIONE 2: Saturazione Attuatori ---
        servo_lim = deg2rad(45); % Limite fisico a 45 gradi
        theta_servo_1 = max(-servo_lim, min(servo_lim, theta_servo_1));
        theta_servo_2 = max(-servo_lim, min(servo_lim, theta_servo_2));
    
        % Calcolo Spinte motori (in Newton)
        T1 = (T_total / 2) - u_yaw;
        T2 = (T_total / 2) + u_yaw;
    
        % Saturazione minima (i motori non possono girare al contrario o spegnersi in volo)
        T1 = max(0, T1);
        T2 = max(0, T2);

        u(1) = sqrt(T1 / params.k);
        u(2) = sqrt(T2 / params.k);
        u(3) = 0; 
        u(4) = theta_servo_1;
        u(5) = theta_servo_2;
        u(6) = 0; u(7) = 0;

    case 9
        % =========================================================
        %   CONTROLLO TRANSIZIONE "CLIMB & BLEND"
        %   Obiettivo: Salire, Accelerare, Spegnere Coda
        % =========================================================
        d_mx = params.d_mx;
        d_my = params.d_my;
        d_mz = params.d_mz;
        d_tx = params.d_tx;


        % 1. TIMING E SCHEDULING
        % Aumentiamo il tempo per dare modo all'ala di lavorare
        T_transizione = 20.0; 
        
        % Sigma: 0 (Inizio Hover) -> 1 (Fine Cruise)
        sigma = max(0, min(1, t / T_transizione));
        
        % Profilo Tilt Base: 90° -> 0° (Coseno per morbidezza)
        theta_base = (pi/2) * (cos(sigma * pi/2))^0.8; % Esponente <1 per accelerare tilt all'inizio
        
        % Profilo Velocità Target: 0 -> 28 m/s (Leggero overshoot per sicurezza stallo)
        vx_des = 28 * sigma;
        
        % 2. STRATEGIA DI QUOTA "SAFETY ARC"
        % Saliamo da -10 a -15/-18 durante la fase critica (sigma 0.3 - 0.7)
        % poi torniamo a -10.
        z_start = -10;
        z_safe = -18; % 8 metri di buffer
        
        if sigma < 0.5
            % Salita: da -10 a -18
            z_des = z_start + (z_safe - z_start) * (sigma / 0.5);
        else
            % Discesa controllata (Glide): da -18 a -10
            progress_desc = (sigma - 0.5) / 0.5;
            z_des = z_safe + (z_start - z_safe) * progress_desc;
        end
        
        % 3. OUTER LOOP (Controllo Traiettoria)
        
        % Controllo Quota (Z)
        e_z = z_des - x(3);
        de_z = 0 - vz_global;
        kp_z = 8.0; kd_z = 4.0;
        acc_z_des = kp_z * e_z + kd_z * de_z;
        
        % Stima Lift Alare
        % Limitiamo la stima per non "fidarci" troppo dell'ala finché non siamo veloci
        F_lift_est = 0.5 * params.rho * params.s * params.C_l * vx_global^2;
        F_lift_est = min(F_lift_est, params.m * params.g * 1.1);
        
        % Forza Verticale che devono generare i MOTORI
        F_z_req = params.m * (params.g - acc_z_des) - F_lift_est;
        
        % Controllo Velocità (X)
        e_vx = vx_des - vx_global;
        kp_v = 4.0;
        F_drag_est = 0.5 * params.rho * params.s * params.C_d * vx_global^2;
        F_x_req = F_drag_est + kp_v * e_vx;
        
        % Spinta Totale Richiesta (Vettoriale approssimata)
        % Nota: F_x la otteniamo col coseno del tilt, F_z col seno.
        % T_tot approx:
        T_tot_req = sqrt(F_x_req^2 + F_z_req^2);
        
        % 4. INNER LOOP (PID Assetto)
        theta_ref = 0; % Fusoliera orizzontale
        phi_ref = 0;
        psi_ref = 0;
        
        kp_pitch = 12; kd_pitch = 4;
        kp_roll = 15;  kd_roll = 5;
        kp_yaw = 8;    kd_yaw = 2;
        
        M_x_cmd = kp_roll * (phi_ref - x(7)) - kd_roll * x(10);
        M_y_cmd = kp_pitch * (theta_ref - x(8)) - kd_pitch * x(11);
        M_z_cmd = kp_yaw * (psi_ref - x(9)) - kd_yaw * x(12);
        
        % =========================================================
        %   5. MIXING IBRIDO (CROSS-FADING)
        % =========================================================
        
        % Definiamo due strategie di attuazione:
        % A) HOVER STRATEGY (Coda attiva per Pitch)
        % B) CRUISE STRATEGY (Coda spenta, Pitch via Tilt Collettivo)
        
        % Peso transizione
        w_cruise = sigma^2; % Inizia lento, finisce deciso (curva quadratica)
        w_hover = 1 - w_cruise;
        
        % --- Strategia A: HOVER (Pitch -> T_rear) ---
        % Sistema semplificato: T_front sostiene Z, T_rear bilancia M_y
        % Braccio anteriore efficace nel body frame
        arm_f = sin(theta_base)*d_mx - cos(theta_base)*d_mz;
        
        % Matrice [Tf; Tr] * [coeffs] = [F_z; M_y]
        A_hov = [sin(theta_base), 1; 
                 arm_f,           d_tx];
        res_hov = pinv(A_hov) * [F_z_req; M_y_cmd];
        
        T_front_hov = res_hov(1);
        T_rear_hov  = res_hov(2);
        d_theta_pitch_hov = 0; % In hover non usiamo tilt collettivo per il pitch
        
        % --- Strategia B: CRUISE (Pitch -> Delta Tilt Anteriore) ---
        % T_rear forzato a 0.
        % Il Pitch si controlla inclinando i motori anteriori (Tilt Collettivo)
        % u_pitch = M_y / (T_tot * d_mx) [cite: 108]
        
        T_rear_cr = 0; 
        T_front_cr = T_tot_req; % Tutto il thrust davanti
        
        if T_front_cr > 1
            % Variazione di angolo necessaria per generare M_y
            % Delta Fz = M_y / d_mx -> Delta Theta approx Fz / T
            d_theta_pitch_cr = M_y_cmd / (T_front_cr * d_mx);
        else
            d_theta_pitch_cr = 0;
        end
        
        % --- BLENDING FINALE ---
        
        % Spinta Anteriore
        T_front_final = w_hover * T_front_hov + w_cruise * T_front_cr;
        
        % Spinta Posteriore (deve andare a 0)
        T_rear_final = w_hover * T_rear_hov + w_cruise * T_rear_cr;
        
        % Correzione Tilt per Pitch (nasce progressivamente)
        % Saturiamo la correzione per sicurezza (max 15 gradi)
        d_theta_pitch_mix = w_cruise * d_theta_pitch_cr;
        d_theta_pitch_mix = max(-0.25, min(0.25, d_theta_pitch_mix));
        
        % 6. ROLL & YAW MIXING
        
        % Roll: Spinta Diff (Hover) -> Tilt Diff (Cruise)
        d_thrust_roll = (M_x_cmd * w_hover) / (2 * d_my);
        if T_front_final > 1
            d_tilt_roll = (M_x_cmd * w_cruise) / (T_front_final * d_my);
        else
            d_tilt_roll = 0;
        end
        
        % Yaw: Tilt Diff (Hover) -> Spinta Diff (Cruise)
        % Attenzione: In crociera usiamo spinta differenziale per yaw [cite: 111]
        d_thrust_yaw = (M_z_cmd * w_cruise) / (2 * d_my);
        if T_front_final > 1
             d_tilt_yaw = (M_z_cmd * w_hover) / (T_front_final * d_my);
        else
             d_tilt_yaw = 0;
        end
        
        % 7. ASSEGNAZIONE ATTUATORI
        
        % Motori Anteriori (T + Roll_Diff_Thrust + Yaw_Diff_Thrust)
        T1 = (T_front_final / 2) - d_thrust_roll - d_thrust_yaw;
        T2 = (T_front_final / 2) + d_thrust_roll + d_thrust_yaw;
        T3 = T_rear_final;
        
        % Saturazioni minime
        T1 = max(0, T1); T2 = max(0, T2); T3 = max(0, T3);
        
        u(1) = sqrt(T1 / params.k);
        u(2) = sqrt(T2 / params.k);
        u(3) = sqrt(T3 / params.k);
        
        % Servi (Tilt Base + Pitch_Corr + Roll_Corr + Yaw_Corr)
        % DX
        u(4) = theta_base + d_theta_pitch_mix - d_tilt_roll + d_tilt_yaw;
        % SX
        u(5) = theta_base + d_theta_pitch_mix + d_tilt_roll - d_tilt_yaw;
        
        % Coda
        u(6) = pi/2; 
        u(7) = 0;
    otherwise
        % Se richiami altri case non definiti qui, metti un default
        u = zeros(7,1);
end
end
