function u = controlloVTOL_v3(params, x)

% Preallocazione
u = zeros(7,1);

test_id = 6;
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

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

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

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
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
        % 1. Estrazione Stato e Velocità Inerziali
        phi = x(7);
        theta = x(8);
        psi = x(9);
        R = matriceRotazione(phi,theta,psi); 
        V_body = [x(4);x(5);x(6)]; 
        V_global = R*V_body ;
        vz_global = V_global(3);

        % 2. Inizializzazione Controllo
        u = zeros(7,1);
        z_des = -10;    % Target quota
        vz_des = 0;     % Target velocità (hovering)
        
        % 3. Parametri SMC (Sliding Mode Control)
        lambda_z = 2.5;  % Reattività convergenza errore
        K_smc = 30;      % Guadagno Robusto (Newton). Aumenta se scende troppo.
        Phi = 0.5;       % Strato limite (evita chattering)

        % 4. Calcolo Errori e Superficie
        e_z = z_des - x(3);           
        de_z = vz_des - vz_global;    
        s = de_z + lambda_z * e_z;    % Superficie di scorrimento

        % --- CALCOLO FORZE FISICHE (in NED) ---
        % Gravità (Positiva verso il basso)
        F_gravity = params.m * params.g * cos(x(8)) * cos(x(7));
        
        % Drag Aerodinamico (Se scende è opposto -> negativo. Se sale è opposto -> positivo)
        % Questa formula restituisce la forza nel frame Body/Global allineata a Z
        F_drag_aero = -params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        
        % Lift Ali (se presente)
        F_lift = -params.C_l*params.rho*params.s*x(4)^2;

        % --- LEGGE DI CONTROLLO (CALCOLO DIRETTO DEL THRUST) ---
        % Equazione: T = mg + F_aero - (Termine_Controllo)
        % Il segno meno davanti all'SMC è fondamentale:
        % Se s è negativo (devo salire), tanh è -1. -(-1) diventa +1. AUMENTA IL THRUST.
        
        u_controllo = params.m * lambda_z * de_z + K_smc * tanh(s / Phi);
        
        Thrust_req = F_gravity + F_drag_aero + F_lift - u_controllo;
        
        % --- MIXING ---
        theta3_ideal = atan2(((-params.d_tx*params.k)/params.b),1);
        theta3_actual = x(17); 

        denom_mix = params.d_mx*params.k*sin(theta3_actual) + params.b*cos(theta3_actual) - params.d_tx*params.k*sin(theta3_actual);
        if abs(denom_mix) < 1e-6; denom_mix = 1e-6; end

        % Qui Thrust_req è già positivo e corretto
        omega3_sq = (params.d_mx * Thrust_req) / denom_mix;
        omega2_sq = (Thrust_req - omega3_sq*params.k*sin(theta3_actual)) / (2*params.k);
        
        % Saturazione sicurezza
        if omega2_sq < 0; omega2_sq = 0; end
        if omega3_sq < 0; omega3_sq = 0; end

        % 8. Assegnazione Output
        u(1) = sqrt(omega2_sq);      % Rotore Anteriore DX
        u(2) = u(1);                 % Rotore Anteriore SX (simmetrico)
        u(3) = sqrt(omega3_sq);      % Rotore Coda
        
        u(4) = pi/2;                 % Servo Ant DX
        u(5) = pi/2;                 % Servo Ant SX
        u(6) = theta3_ideal;         % Servo Coda (Target ideale)
        u(7) = -pi/2;

    case 6
        % --- CONTROLLO COMPLETO ROBUSTO (X, Y, Z) ---
        
        % 1. Estrazione Stato
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10); q = x(11); r = x(12);
        
        R = matriceRotazione(phi,theta,psi); 
        V_body = [x(4);x(5);x(6)]; 
        V_global = R*V_body ;
        vx_global = V_global(1);
        vy_global = V_global(2);
        vz_global = V_global(3);

        u = zeros(7,1);

        % 2. Parametri Obiettivo
        z_des = -10;    vz_des = 0;
        y_des = 0;      vy_des = 0;
        x_des = 0;      vx_des = 0; % Vogliamo stare fermi anche su X

        % 3. Parametri Controllori
        % Z (Quota)
        lambda_z = 2.5; K_z_smc = 60; Phi_z = 0.8;
        % Y (Laterale)
        lambda_y = 0.8; K_y_smc = 15; Phi_y = 1.0;
        % X (Longitudinale)
        lambda_x = 0.8; K_x_smc = 8; Phi_x = 1.0;
        
        % PD Attitudine (Roll & Pitch)
        % Nota: Ho abbassato i guadagni come discusso per stabilità
        kp_phi = 40;   kd_phi = 8; 
        kp_theta = 40; kd_theta = 8;

        % =========================================================
        %   LOOP Z (QUOTA)
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
        %   LOOP Y (LATERALE -> ROLLIO)
        % =========================================================
        e_y = y_des - x(2);          
        de_y = vy_des - vy_global;   
        s_y = de_y + lambda_y * e_y; 
        
        F_y_req = params.m * lambda_y * de_y + K_y_smc * tanh(s_y / Phi_y);
        
        sin_phi_des = F_y_req / Thrust_req;
        sin_phi_des = max(min(sin_phi_des, 0.5), -0.5); 
        phi_des = asin(sin_phi_des);
        
        e_phi = phi_des - phi;
        de_phi = 0 - p; 
        Moment_roll_req = kp_phi * e_phi + kd_phi * de_phi;

        % =========================================================
        %   LOOP X (LONGITUDINALE -> PITCH)
        % =========================================================
        e_x = x_des - x(1);
        de_x = vx_des - vx_global;
        s_x = de_x + lambda_x * e_x;

        % Forza longitudinale richiesta
        F_x_req = params.m * lambda_x * de_x + K_x_smc * tanh(s_x / Phi_x);

        % Conversione Forza X -> Angolo Pitch
        % Attenzione ai segni: Per andare avanti (+X), serve Fx positiva.
        % Il vettore Thrust inclinato in avanti crea una Fx positiva se Theta è NEGATIVO.
        % Fx approx -Thrust * sin(theta)
        % Quindi sin(theta) = -Fx / Thrust
        sin_theta_des = -F_x_req / Thrust_req;
        sin_theta_des = max(min(sin_theta_des, 0.5), -0.5);
        theta_des = asin(sin_theta_des);

        % Controllo PD Pitch (Momento Y)
        e_theta = theta_des - theta;
        de_theta = 0 - q;
        Moment_pitch_req = kp_theta * e_theta + kd_theta * de_theta;

        % =========================================================
        %   MIXING E ALLOCAZIONE
        % =========================================================
        theta3_ideal = atan2(((-params.d_tx*params.k)/params.b),1);
        theta3_actual = x(17); 
        
        % --- 1. Mixing Longitudinale (Z + Pitch) ---
        % Qui dobbiamo bilanciare sia la Spinta Totale che il Momento di Pitch
        
        % Denominatore base (Equilibrio statico)
        denom_mix = params.d_mx*params.k*sin(theta3_actual) ...
                  - params.d_tx*params.k*sin(theta3_actual) ...
                  + params.b*cos(theta3_actual);
        if abs(denom_mix) < 1e-6; denom_mix = 1e-6; end
        
        % Modifica Cruciale: Inseriamo il Momento Pitch nel calcolo della coda
        % Se Moment_pitch_req > 0 (voglio alzare il muso), devo ridurre la coda
        % (perché la coda ha braccio negativo d_tx, quindi spinta coda crea momento giù)
        % Formula derivata:
        numeratore_coda = (params.d_mx * Thrust_req) - Moment_pitch_req;
        
        omega3_sq = numeratore_coda / denom_mix;
        
        % I motori anteriori prendono il resto della spinta verticale
        F_tail_z = omega3_sq * params.k * sin(theta3_actual);
        F_front_tot_z = Thrust_req - F_tail_z;
        omega_front_sq_base = F_front_tot_z / (2 * params.k);
        
        % --- 2. Mixing Laterale (Roll) ---
        braccio_y = params.d_my;
        delta_omega_sq = Moment_roll_req / (params.k * braccio_y * 2);
        
        omega_dx_sq = omega_front_sq_base - delta_omega_sq; 
        omega_sx_sq = omega_front_sq_base + delta_omega_sq; 
        
        % Saturazioni
        if omega_dx_sq < 0; omega_dx_sq = 0; end
        if omega_sx_sq < 0; omega_sx_sq = 0; end
        if omega3_sq < 0; omega3_sq = 0; end

        u(1) = sqrt(omega_dx_sq);    
        u(2) = sqrt(omega_sx_sq);    
        u(3) = sqrt(omega3_sq);      
        u(4) = pi/2; u(5) = pi/2; u(6) = theta3_ideal; u(7) = -pi/2;

    otherwise
        error('Controllo non valido');
end

end