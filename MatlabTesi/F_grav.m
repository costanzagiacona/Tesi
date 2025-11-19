function F_g = F_grav(phi , theta , ~ , m, g)

F_g = m*g*[-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)];

end