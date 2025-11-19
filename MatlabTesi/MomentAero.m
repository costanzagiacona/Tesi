function M_aero = MomentAero(r_aerodyn_w_dx,r_aerodyn_w_sx,C_l,C_d, C_d_z,rho ,v_x ,v_z, s)


% ala dx

F_aero_dx = [-sign(v_x)*(1/2)*rho*((v_x)^2)*s*C_d; 0 ;-(1/2)*rho*((v_x)^2)*s*C_l-sign(v_z)*(1/2)*rho*((v_z)^2)*s*C_d_z];
M_dx = cross(r_aerodyn_w_dx,F_aero_dx);

% ala sx

F_aero_sx = [-sign(v_x)*(1/2)*rho*((v_x)^2)*s*C_d; 0 ;-(1/2)*rho*((v_x)^2)*s*C_l-sign(v_z)*(1/2)*rho*((v_z)^2)*s*C_d_z];
M_sx = cross(r_aerodyn_w_sx,F_aero_sx);


M_aero = M_dx + M_sx;

end