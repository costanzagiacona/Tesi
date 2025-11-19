function [F] = F_thrust(u)


theta_1 = u(4);
theta_2 = u(5);
theta_3 = u(6); % primo grado di libertà per il tail rotor (angolo tra asse X_body , Z_body)
theta_4 = u(7); % secondo grado di libertà per il tail rotor (angolo tra asse X_body , Y_body)


F = [cos(theta_1) cos(theta_2) cos(theta_3)*cos(theta_4); 0 0 cos(theta_3)*sin(theta_4);-sin(theta_1) -sin(theta_2) -sin(theta_3)]*[u(1);u(2);u(3)];

end