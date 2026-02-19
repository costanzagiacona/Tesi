function [lift,drag] = coeffAero(alpha,beta,params)


alpha_abs = abs(alpha);
alpha1 = deg2rad(15);   % Lo stallo vero inizia verso i 15-18 gradi
alpha2 = deg2rad(45);   % Oltre i 45 gradi l'ala è totalmente in stallo

t = (alpha_abs - alpha1) / (alpha2 - alpha1);
t = min(max(t, 0), 1);
n = 1 - (3*t^2 - 2*t^3); % 1 a piccoli angoli, 0 a grandi angoli

lift = n*params.C_l;

n_drag = (3*t^2 - 2*t^3);
drag = (1+0.2*n_drag)*params.C_d;



end