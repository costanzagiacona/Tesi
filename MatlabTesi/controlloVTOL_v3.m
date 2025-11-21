function u = controlloVTOL_v3(t, params, x)

% Preallocazione
u = zeros(7,1);
test_id = 2;

% Variabili persistenti
persistent flight_phase; 
persistent integral_psi_error;
persistent last_t;

% Inizializzazione
if isempty(flight_phase) || t == 0
    flight_phase = 1; 
    integral_psi_error = 0;
    last_t = t;
end

% Calcolo dinamico di dt
dt = t - last_t;
last_t = t;

d_mx = params.d_mx;
d_tx = abs(params.d_tx);

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
        % --- Estrazione degli stati ---
        z = x(3);
        phi = x(7); theta = x(8); psi = x(9);
        r = x(12);
        p = x(10); q = x(11);
        
        
        R = matriceRotazione(phi,theta,psi);
        V_body = [x(4);x(5);x(6)];
        V_global = R*V_body;
        vz_global = V_global(3);

        % --- Parametri del controllore ---
        z_des_decollo = -10;
        vx_des_avanti = 25;

        % --- GESTORE DELLA FASE DI VOLO ---
        if flight_phase == 1 && z <= z_des_decollo
            flight_phase = 2;
            fprintf('Transizione alla FASE 2: Volo Orizzontale a t=%.2f s\n', t);
        end
        
        if flight_phase == 1
            % ***************************************************************
            % ** FASE 1: DECOLLO VERTICALE (CORRETTO)                      **
            % ***************************************************************
            
            u(4) = pi/2; % Tilt rotore 1 (dx)
            u(5) = pi/2; % Tilt rotore 2 (sx)
            u(6) = pi/2; % Tilt rotore 3 (asse principale, theta_3)
            
            % ======================================================================
            % CORREZIONE #1: I guadagni devono essere NEGATIVI e più robusti
            % Un errore di posizione negativo (z_des - z < 0) deve produrre una
            % forza di feedback POSITIVA (verso l'alto).
            Kp_z = -50.0; 
            Kd_z = -20.0;
            % ======================================================================

            feedback_force = Kp_z*(z_des_decollo - z) + Kd_z*(0 - vz_global);
            gravity_comp = params.m * params.g;
            drag_comp_z = -params.rho * params.s * params.C_d_z * sign(x(6)) * x(6)^2;
            F_des = max(0, feedback_force + gravity_comp + drag_comp_z);
           

            % Normalizziamo il comando di spinta tra 0 e 1 (pratica comune)
            % Assumiamo una spinta massima per motore di 2 volte il peso del drone
            max_thrust_per_motor = (params.m * params.g) * 2;
            max_total_thrust = 3 * max_thrust_per_motor;
            thrust_cmd = F_des / max_total_thrust;
            thrust_cmd = max(0, min(1, thrust_cmd)); % Saturazione tra 0 e 1
        
            % --- Comandi di Assetto (Roll, Pitch) ---
            % Controllori PD per stabilizzare roll e pitch a zero.
            % L'output è il momento desiderato attorno agli assi x e y.
            phi_des = 0; theta_des = 0;
            Kp_phi = 20; Kd_phi = 5;     % Guadagni Roll
            Kp_theta = 20; Kd_theta = 5;  % Guadagni Pitch
            
            
            % Questi sono i momenti correttivi che vogliamo applicare
            roll_input  = Kp_phi * (phi_des - phi)   + Kd_phi * (0 - p);
            pitch_input = Kp_theta * (theta_des - theta) + Kd_theta * (0 - q);
            
            % 3. MIXER: TRADUZIONE DEI COMANDI IN VELOCITÀ DEI MOTORI
            % Questa è la logica standard per un tricottero a "Y"
            
            % La spinta base è uguale per tutti
            T_base = thrust_cmd;
            
            % Applica i comandi di assetto
            % Un 'pitch_input' positivo (comando per abbassare il muso) DEVE ridurre la spinta del rotore di coda.
            % Un 'roll_input' positivo (comando per inclinare a destra) DEVE ridurre T1 e aumentare T2.
            T1 = T_base - pitch_input - roll_input;
            T2 = T_base - pitch_input + roll_input;
            T3 = T_base + pitch_input;
            
            % Saturazione finale per assicurare che i comandi siano tra [0, 1]
            T1 = max(0, min(1, T1));
            T2 = max(0, min(1, T2));
            T3 = max(0, min(1, T3));
            
            % De-normalizza i comandi in spinte reali [N]
            Thrust_1 = T1 * max_thrust_per_motor;
            Thrust_2 = T2 * max_thrust_per_motor;
            Thrust_3 = T3 * max_thrust_per_motor;
        
            % 4. CONTROLLO D'IMBARDATA (YAW) CON TILT DEL ROTORE DI CODA
            % Questa parte rimane concettualmente simile
            psi_des = 0;
            Kp_psi = 2.0; Ki_psi = 0.5; Kd_psi = 1.0;
            psi_error = psi_des - psi;
            if dt > 1e-6
                integral_psi_error = integral_psi_error + psi_error * dt;
            end
            derivative_psi_error = 0 - r;
            yaw_input = Kp_psi * psi_error + Ki_psi * integral_psi_error + Kd_psi * derivative_psi_error;
        
            max_tilt_yaw = 20 * (pi/180);
            u(7) = max(-max_tilt_yaw, min(max_tilt_yaw, yaw_input)); % Comando a theta_4
        
            % 5. ASSEGNAZIONE DEGLI OUTPUT FINALI
            u(1) = sqrt(max(0, Thrust_1 / params.k));
            u(2) = sqrt(max(0, Thrust_2 / params.k));
            u(3) = sqrt(max(0, Thrust_3 / params.k)); % T3 NON sarà più zero!
        
            % Stampa di debug per verificare
            if mod(t, 0.5) < dt % Stampa ogni mezzo secondo
                fprintf('t=%.2f | T1=%.2f, T2=%.2f, T3=%.2f\n', t, Thrust_1, Thrust_2, Thrust_3);
            end


        else % flight_phase == 2
            % -------------------------------------------------------------
            % FASE 2: VOLO ORIZZONTALE MANTENENDO QUOTA Z = -10
            % -------------------------------------------------------------

            phi = x(7);
            theta = x(8);
            psi = x(9);

            % Velocità nel frame inerziale
            R = matriceRotazione(phi,theta,psi);
            V_body = [x(4); x(5); x(6)];
            V_global = R * V_body;

            vx_global = V_global(1);
            vz_global = V_global(3);

            % -----------------------------------------
            % CONTROLLO DI AVANZAMENTO (asse X)
            % -----------------------------------------
            vx_des = 0;
            kp_x = 0;

            % drag aerodinamico lungo x
            Drag_X = params.rho * params.s * params.C_d * sign(x(4)) * x(4)^2;

            Fx_des = Drag_X + kp_x*(vx_des - x(4));

            % -----------------------------------------
            % CONTROLLO DI QUOTA (asse Z)
            % -----------------------------------------
            z_des = -10; 
            vz_des = 0;

            Kp_z = -3;
            Kd_z = -5;

            % PD su quota
            Fz_PD = Kp_z*(z_des - x(3)) + Kd_z*(vz_des - vz_global);

            % portanza aerodinamica
            Lift = -params.C_l * params.rho * params.s * x(4)^2;

            % drag verticale
            Drag_Z = -params.rho * params.s * params.C_d_z * sign(x(6)) * x(6)^2;

            % forza verticale richiesta
            Fz_des = params.m * params.g  + Fz_PD;

            % -----------------------------------------
            % FORZA TOTALE E TILT
            % -----------------------------------------
            T_tot = sqrt(Fx_des^2 + Fz_des^2);
            theta_bar = atan2(Fz_des, Fx_des);

            % -----------------------------------------
            % ASSEGNAZIONE AI ROTORI ANTERIORI
            % -----------------------------------------
            T_i = T_tot / 2;

            % thrust → velocità motori
            u(1) = sqrt(max(0, T_i / params.k));  % omega1
            u(2) = u(1);                           % omega2

            % tilt dei due rotori anteriori
            u(4) = theta_bar;   % tilt rotore destro
            u(5) = theta_bar;   % tilt rotore sinistro

            % Rotore posteriore (qui lo lasci neutro)
            u(3) = 0;
            u(6) = 0;
            u(7) = 0;
        end


    otherwise
        error('Controllo non valido');
end

end