function M_tilt = M_tilt_rotor(input_thrust,I_rotor_w_dx,I_rotor_w_sx,I_rotor_tail,Omega_rotor_w_dx , Omega_rotor_w_sx , Omega_rotor_tail)

u1 = input_thrust(1);
u2 = input_thrust(2);
u3 = input_thrust(3);
theta_1 = input_thrust(4);
theta_2 = input_thrust(5);
theta_3 = input_thrust(6); % primo grado di libertà per il tail rotor (angolo tra asse X_body , Z_body)
theta_4 = input_thrust(7); % secondo grado di libertà per il tail rotor (angolo tra asse X_body , Y_body)

% if u1 > 0
%  omega1 = (u1/k)^(1/2);
% else
%     disp('errore : omega1 negativo');
%     omega1 = 0; % per sicurezza
% end
% 
% if u2 > 0
%  omega2 = (u2/k)^(1/2);
% else
%     disp('errore : omega2 negativo');
%     omega2 = 0; % per sicurezza
% end
% 
% if u3 > 0
%  omega3 = (u3/k)^(1/2);
% else
%     disp('errore : omega3 negativo');
%     omega3 = 0; % per sicurezza
% end

dir2 = -1; % rotore anteriore sx contro-rotante rispetto al rotore anteriore dx

omega1 = u1;
omega2 = dir2*u2;
omega3 = u3;

% versore asse rotore 1
e1 = [cos(theta_1);0;-sin(theta_1)];

% versore asse rotore 2
e2 = [cos(theta_2);0;-sin(theta_2)];

% versore asse rotore 3
e3 = [cos(theta_3)*cos(theta_4);cos(theta_3)*sin(theta_4);-sin(theta_3)];

omega_spin_1 = omega1*e1;
omega_spin_2 = omega2*e2;
omega_spin_3 = omega3*e3;

% M_tilt_1 = cross(I_rotor_w_dx*omega_spin_1,Omega_rotor_w_dx);
% M_tilt_2 = cross(I_rotor_w_sx*omega_spin_2,Omega_rotor_w_sx);
% M_tilt_3 = cross(I_rotor_tail*omega_spin_3,Omega_rotor_tail);


M_tilt_1 = cross(Omega_rotor_w_dx,I_rotor_w_dx*omega_spin_1);
M_tilt_2 = cross(Omega_rotor_w_sx,I_rotor_w_sx*omega_spin_2);
M_tilt_3 = cross(Omega_rotor_tail,I_rotor_tail*omega_spin_3);

M_tilt = M_tilt_1 + M_tilt_2 + M_tilt_3;

end