function M_gyro_body = MomentGyroBody(I_body,Omega_body)

M_gyro_body = cross(Omega_body,I_body*Omega_body);

end