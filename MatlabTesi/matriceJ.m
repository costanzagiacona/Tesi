function J = matriceJ(phi,theta,psi)

r11 = 1;
r12 = sin(phi)*tan(theta);
r13 = cos(phi)*tan(theta);

r21 = 0;
r22 = cos(phi);
r23 = -sin(phi);

r31 = 0;
r32 = sin(phi)/cos(theta);
r33 = cos(phi)/cos(theta);

J = [r11 r12 r13; r21 r22 r23; r31 r32 r33];
end