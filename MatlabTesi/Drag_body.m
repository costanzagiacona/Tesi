function F_aeroBody = Drag_body(C_d_x,C_d_y,C_d_z,rho,s_body_x,s_body_y,s_body_z,vx,vy,vz)

F_aeroBody = -(1/2)*rho*[sign(vx)*s_body_x*C_d_x*(vx^2);sign(vy)*s_body_y*C_d_y*(vy^2);sign(vz)*s_body_z*C_d_z*(vz^2)];

end