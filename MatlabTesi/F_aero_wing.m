function Fa = F_aero_wing(C_l,C_d,C_y, rho, s,Va,Rwb)

coeffAero = [-C_d;C_y;-C_l];
Fa = (1/2)*rho*s*(Va^2)*Rwb*coeffAero;

end