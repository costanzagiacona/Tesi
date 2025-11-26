function [F_wing] = F_aerodyn_wing(C_l,C_d,C_d_z, rho ,v_x, v_z ,s)

F_wing = [-sign(v_x)*(1/2)*rho*((v_x)^2)*s*(C_d +C_d); 0 ;-(1/2)*rho*((v_x)^2)*s*(C_l+C_l)-sign(v_z)*(1/2)*rho*((v_z)^2)*s*(C_d_z+C_d_z)];



end