clc
clear all

m = 6;      % massa UAV VTOL (Kg)  % NB -> se si cambia questo parametro cambia la nostra stima di C_d e C_l
g = 9.8;    % costante gravitazionele (m/s^2)

k = 7*(10^-5);      % coeff. thrust rotori
rho = 1.225;    % densità dell'aria (kg/m^3)
ala_y = 0.4;
ala_x = 0.15;
s = ala_x * ala_y;      % superficie alare (m^2) 

v_air = 25;   % velocità relativa alla superficie alare (m/s) lungo x
v_limite = 13.89; % -> 50km/h

C_d = ((m-1)*g)/(rho*s*(v_air)^2); %1.28;  %coeff. di resistenza (drag) aerodinamica lungo asse X

C_l = (m*g)/(rho*s*(v_air)^2); %0.854; %coeff. di portanza (lift) aerodinamica 

C_d_z = (m*g)/(rho*s*(v_limite)^2); % coeff. di resistenza (drag) aerodinamica lungo asse z


x4e = 25;
x5e = 0;
x6e = 0;


x13e = atan((m*g - C_l*rho*s*x4e^2) / (C_d*rho*s*x4e^2)) ;
x13e = pi/6;
x15e = x13e;                                                
x21e = sqrt(C_d*rho*s*x4e^2 / (2*k*cos(x13e)));            
x23e = x21e;      


b11 = (2*k*x21e*sin(x13e))/m;
b12 = (2*k*x23e*sin(x15e))/m;
b21 = (2*k*x21e*cos(x13e))/m;
b22 = (2*k*x23e*cos(x15e))/m;

A = [0 1 0;
     0 0 0;
     0 0 0];

B = [0 0;
     b11 b12;
     b21 b22];

C = [1 0 0;   % y1 = pz
     0 0 1];  % y2 = vx


D = zeros(2,2);


sys = ss(A,B,C,D);
W_2x2 = tf(sys)
W_2x2 = minreal(W_2x2);

disp('Funzione di trasferimento W(s) = ');
W_2x2































