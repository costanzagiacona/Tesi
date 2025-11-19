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

    case { -1, 0, 1}
        u = zeros(7,1);

    case 2
        % --- Estrazione degli stati ---
        z = x(3);
        phi = x(7); theta = x(8); psi = x(9);
        r = x(12);
        
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
           

            if d_tx < 1e-6
                error('La distanza del rotore di coda (d_tx) non può essere zero.');
            end
            
            T1 = F_des / (2 * (1 + d_mx / d_tx));
            T2 = T1;
            T3_verticale = F_des - 2 * T1;
           
            % --- CONTROLLO YAW ---
            theta_4_static = asin(params.b / (params.k * d_tx));
            Kp_psi = 2.0; Ki_psi = 0.5; Kd_psi = 1.0;
            psi_error = 0 - psi;

            if dt > 1e-6
                integral_psi_error = integral_psi_error + psi_error * dt;
            end
            
            derivative_psi_error = 0 - r;
            pid_yaw_output = Kp_psi * psi_error + Ki_psi * integral_psi_error + Kd_psi * derivative_psi_error;
            theta_4_des = theta_4_static + pid_yaw_output;
            
            max_tilt_yaw = 20 * (pi/180);
            u(7) = max(-max_tilt_yaw, min(max_tilt_yaw, theta_4_des)); % Comando a theta_4

            % Calcolo spinta totale rotore di coda
            if abs(cos(u(7))) > 1e-6
                T3_totale = T3_verticale / cos(u(7));
            else
                T3_totale = 0;
            end
            
            u(1) = sqrt(max(0, T1 / params.k));
            u(2) = sqrt(max(0, T2 / params.k));
            u(3) = sqrt(max(0, T3_totale / params.k));

        else % flight_phase == 2
            % --- FASE 2: VOLO ORIZZONTALE ---
            % (Logica invariata)
            kp_x = 5;
            F_x_des = params.rho*params.s*params.C_d*sign(x(4))*x(4)^2 + kp_x*(vx_des_avanti - x(4));

            kp_z = -3; kd_z = -5;
            z_des = z_des_decollo;
            F_z_des = -params.C_l*params.rho*params.s*x(4)^2 + params.m*params.g*cos(theta)*cos(phi) - params.rho*params.s*params.C_d_z*sign(x(6))*x(6)^2;      
            F_z_des = F_z_des + kp_z*(z_des - x(3)) + kd_z*(0 - vz_global);

            T_tot = sqrt(F_x_des^2 + F_z_des^2);
            theta_bar = atan2(F_z_des, F_x_des);

            T_i = T_tot / 2;
            u(1) = sqrt(T_i / params.k); u(2) = u(1);
            u(4) = theta_bar; u(5) = theta_bar;
            u(3) = 0; u(6) = 0; u(7) = 0;
        end

    otherwise
        error('Controllo non valido');
end

end