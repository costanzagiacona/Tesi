function M_th = M_thrust_noTorc (u, r_dx,r_sx, r_tail,b,k,omega1_dot,omega2_dot,omega3_dot,Iz1,Iz2,Iz3)

theta_1 = u(4);
theta_2 = u(5);
theta_3 = u(6); % primo grado di libertà per il tail rotor (angolo tra asse X_body , Z_body)
theta_4 = u(7); % secondo grado di libertà per il tail rotor (angolo tra asse X_body , Y_body)

% Forze in body frame, una per rotore
F_dx = [cos(theta_1); 0; -sin(theta_1)] * u(1);
F_sx = [cos(theta_2); 0; -sin(theta_2)] * u(2);
F_tail = [cos(theta_3)*cos(theta_4); cos(theta_3)*sin(theta_4); -sin(theta_3)] * u(3);


% Momenti individuali
M1 = cross(r_dx, F_dx);
M2 = cross(r_sx, F_sx);
M3 = cross(r_tail, F_tail);

% calcolo momento torcente

dir2 = -1; % rotore anteriore sx contro-rotante rispetto al rotore anteriore dx
if u(1)==0
test = 1;
end
omega1_2 = (u(1)/k);
omega2_2 = dir2*(u(2)/k);
omega3_2 = (u(3)/k);


% versore asse rotore 1
e1 = [cos(theta_1);0;-sin(theta_1)];

% versore asse rotore 2
e2 = [cos(theta_2);0;-sin(theta_2)];

% versore asse rotore 3
e3 = [cos(theta_3)*cos(theta_4);cos(theta_3)*sin(theta_4);-sin(theta_3)];

omega_spin_1_2 = omega1_2*e1;
omega_spin_2_2 = omega2_2*e2;
omega_spin_3_2 = omega3_2*e3;

acc1 = omega1_dot*e1;
acc2 = dir2*omega2_dot*e2;
acc3 = omega3_dot*e3;

% M_torc_1 = b*omega_spin_1_2 ;%+ Iz1*acc1;
% M_torc_2 = b*omega_spin_2_2 ;%+ Iz2*acc2;
% M_torc_3 = b*omega_spin_3_2 ;%+ Iz3*acc3;

% per test senza momento torcente
M_torc_1 = [0;0;0];
M_torc_2 = [0;0;0];
M_torc_3 = [0;0;0];

% Momento totale nel body frame
M_th = M1 + M2 + M3 + M_torc_1 + M_torc_2 + M_torc_3;

% if M_th(3) ~= 0 M_th(2) ~= 0
%     debug = 1;
% end


end