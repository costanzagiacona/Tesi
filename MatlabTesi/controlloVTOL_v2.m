function u = controlloVTOL_v2(params, x)

% Preallocazione
u = zeros(7,1);

test_id = 2;
% TEST
% -1 : Debug (tutto simbolico)
% 0 : Nessun controllo (solo gravità)
% 1 : compensazione attrito lungo X 




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

    otherwise
        error('Controllo non valido');
end

end