function [F_Cor] = F_Coriolis(Omega_b,V_b,m)

F_Cor = cross(Omega_b,m*V_b);

end