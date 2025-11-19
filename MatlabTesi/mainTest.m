%% SIMULAZIONE VTOL 2

% Locatelli Andrea - Tesi Magistrale Ing. Automazione
% A.A 2024-2025

clear all
clc

% flag print

paramFlag = 0; % se 1 print del valore dei parametri

% parametri VTOL 

% (roll , pitch , yaw) = (phi , theta , psi)

m = 6;      % massa UAV VTOL (Kg)  % NB -> se si cambia questo parametro cambia la nostra stima di C_d e C_l
g = 9.8;    % costante gravitazionele (m/s^2)

k = 7*(10^-5);      % coeff. thrust rotori

%matrice inerzia del corpo (UAV) rispetto al body frame (kg*m^2)
Ixx = 3;
Iyy = 0.5 * Ixx;
Izz = 1.4 * Ixx;

I_body = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];     

%inerzie dei tre rotori : wing_dx , wing_sx , tail (kg*m^2)

I_rotor_xx = 2.5;
I_rotor_yy = 0.5*I_rotor_xx;
I_rotor_zz = 1.4*I_rotor_xx;

I_rotor = (10^-3)*[I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];  

I_rotor_w_dx = I_rotor;
I_rotor_w_sx = I_rotor;
I_rotor_tail = I_rotor;


rho = 1.225;    % densità dell'aria (kg/m^3)
ala_y = 0.4;
ala_x = 0.15;
s = ala_x * ala_y;      % superficie alare (m^2) 

v_air = 25;   % velocità relativa alla superficie alare (m/s) lungo x
v_limite = 13.89; % -> 50km/h

C_d = ((m-1)*g)/(rho*s*(v_air)^2); %1.28;  %coeff. di resistenza (drag) aerodinamica lungo asse X

% scelto in modo tale che se v_x = 90 km/h (25 m/s) la portanza contrasti
% la gravità
C_l = (m*g)/(rho*s*(v_air)^2); %0.854; %coeff. di portanza (lift) aerodinamica 

% scelto in modo tale che sia raggiunta la velocità limite di 50 km/h
% (13.89 m/s) in caduta libera
C_d_z = (m*g)/(rho*s*(v_limite)^2); % coeff. di resistenza (drag) aerodinamica lungo asse z
%C_d_z = 1.2;

%distanza (m) tra centro di massa e rotori (per il calcolo del momento dei thrust)
%lungo i vari assi (X_body , Y_body, Z_body)

d_mx = 0; % assumo il wing rotor posto sull'asse Y_body
d_my = (1/2)*ala_y;
d_mz = 0;

d_tx = -0.3;
d_ty = 0;  % assumo il tail rotor posto sull'asse X_body
d_tz = 0;

b = 0.01; % rapporto tra momento torcente e forza del rotore

%parametri distanza (m) tra centro di massa e forze aerodinamiche (per il calcolo del momento delle forze aerodinamiche)

l_w_dx_x = 0;%(1/3)*ala_x; % distanza per l'ala dx da centro di massa lungo asse X_body
l_w_dx_y = (1/2)*ala_y; % distanza per l'ala dx da centro di massa lungo asse Y_body
l_w_dx_z = 0; % distanza per l'ala dx da centro di massa lungo asse Z_body

l_w_sx_x = 0;%(1/3)*ala_x; % distanza per l'ala sx da centro di massa lungo asse X_body
l_w_sx_y = -(1/2)*ala_y; % distanza per l'ala sx da centro di massa lungo asse Y_body
l_w_sx_z = 0; % distanza per l'ala sx da centro di massa lungo asse Z_body


% Struttura con tutti i parametri

% Parametri generali
parametri.m = m;
parametri.g = g;
parametri.k = k;

% Inerzia corpo
parametri.Ixx = Ixx;
parametri.Iyy = Iyy;
parametri.Izz = Izz;
parametri.I_body = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

% Inerzia rotore con Huygens - Steiner
m_rotor = 0.1;
I_rotor_w_dx = I_rotor + m_rotor*[d_mx^2 0 0;0 d_my^2 0;0 0 d_mz^2];
I_rotor_w_sx = I_rotor + m_rotor*[d_mx^2 0 0;0 d_my^2 0;0 0 d_mz^2];
I_rotor_tail = I_rotor + m_rotor*[d_tx^2 0 0;0 d_ty^2 0;0 0 d_tz^2];

parametri.I_rotor_xx = I_rotor_xx;
parametri.I_rotor_yy = I_rotor_yy;
parametri.I_rotor_zz = I_rotor_zz;
parametri.I_rotor = [I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];

% uso questi
parametri.I_rotor_w_dx = I_rotor_w_dx;
parametri.I_rotor_w_sx = I_rotor_w_sx;
parametri.I_rotor_tail = I_rotor_tail;

% Parametri aerodinamici
parametri.rho = rho;
parametri.s = s;
parametri.C_d = C_d;
parametri.C_d_z = C_d_z;
parametri.C_l = C_l;
parametri.b = b;
parametri.v_air = v_air;

% Parametri ala
parametri.ala_x = ala_x;
parametri.ala_y = ala_y;

% Distanze momento (rotori/forze)
parametri.d_mx = d_mx;
parametri.d_my = d_my;
parametri.d_mz = d_mz;

% Distanze traslazione
parametri.d_tx = d_tx;
parametri.d_ty = d_ty;
parametri.d_tz = d_tz;

% Distanze ala destra
parametri.l_w_dx_x = l_w_dx_x;
parametri.l_w_dx_y = l_w_dx_y;
parametri.l_w_dx_z = l_w_dx_z;

% Distanze ala sinistra
parametri.l_w_sx_x = l_w_sx_x;
parametri.l_w_sx_y = l_w_sx_y;
parametri.l_w_sx_z = l_w_sx_z;
parametri.b = b;

parametri.r_th_w_dx = [d_mx ;  d_my ;  d_mz];
parametri.r_th_w_sx = [d_mx ; -d_my ;  d_mz]; % considero rotore ala dx e sx in posizione simmetrica
parametri.r_th_tail = [d_tx ;  d_ty ;  d_tz];

parametri.l_w_dx = [l_w_dx_x; l_w_dx_y; l_w_dx_z];
parametri.l_w_sx = [l_w_sx_x; l_w_sx_y; l_w_sx_z];

parametri.r_aerodyn_w_dx = parametri.l_w_dx;
parametri.r_aerodyn_w_sx = parametri.l_w_sx;



%% SIMULAZIONE

tspan = [0 20];              % intervallo di simulazione
x0 = zeros(26,1);            % stato iniziale 


x0(4) = 0; 
x4eq = x0(4);

% "controllo all'equilibrio"
omega1_2 = (0.0)*((parametri.rho*parametri.s*parametri.C_d*(x4eq)^2)/parametri.k);
omega2_2 = (0.0)*((parametri.rho*parametri.s*parametri.C_d*(x4eq)^2)/parametri.k);
x0(21) = sqrt(omega1_2);
x0(23) = sqrt(omega2_2);

% posizione iniziale lungo z
x0(3) = 0;

%inclinazione iniziale dei rotori
x0(13)= pi/2;
x0(15)= pi/2;
x0(17)= 0;
x0(19)= 0;



options = odeset('RelTol',1e-3,'AbsTol',1e-6);


[t, x] = ode45( @(t, x) simulazioneVTOL2(x,parametri), tspan, x0, options);

%per plot controllo
global U_values
U_values = zeros(length(t),7);
for k = 1:length(t)
    U_values(k,:) = controlloVTOL_v2(parametri,x(k,:));
end


xp = x(:,1);
yp = x(:,2);
zp = -1*x(:,3); % asse z positivo verso il basso

xv = x(:,4);
yv = x(:,5);
zv = -1* x(:,6); % asse z positivo verso il basso

phi = x(:,7);
theta = x(:,8);
psi = x(:,9);

p = x(:,10);
q = x(:,11);
r = x(:,12);

time = linspace(0,tspan(2),size(x,1));

plotTesi = 1;

if plotTesi == 1
    % grafici che uso nella tesi

    % andamento sottosistema controllato
    figure(12)
    set(gcf, 'Position', [100 100 1000 800])

    % --- vx ---
    subplot(3,1,1);
    h1=plot(time, xv, 'r', 'LineWidth', 2); hold on;
    h2=yline(25,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
    grid on; ylim([-10 40])
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('v_x [m/s]', 'FontSize', 14)
    title('Velocità lungo x','FontSize',16)
    set(gca, 'FontSize', 14)
    legend([h1 h2], {'v_x','vx_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')


    % --- vz ---
    subplot(3,1,2);
    h3=plot(time, zv, 'b', 'LineWidth', 2); hold on;
    h4=yline(0,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
    grid on; ylim([-20 20])
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('v_z [m/s]', 'FontSize', 14)
    title('Velocità lungo z','FontSize',16)
    set(gca, 'FontSize', 14)
    legend([h3 h4], {'v_z','vz_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')


    % --- z (quota) ---
    subplot(3,1,3);
    h5=plot(time, zp, 'g', 'LineWidth', 2); hold on;
    h6=yline(10,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
    grid on; ylim([-16 16])
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Quota z [m]', 'FontSize', 14)
    title('Posizione lungo z','FontSize',16)
    set(gca, 'FontSize', 14)
    legend([h5 h6], {'z','z_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')

    % thrust genearato dai rotori

    omega_1 = x(:,21);
    omega_2 = x(:,23);

    figure(3)
    set(gcf,'Position',[100 100 1000 800])

    subplot(3,1,1);
    h1 = plot(time, parametri.k*omega_1.^2, 'r','LineWidth',2);
    legend('Thrust_{1}','FontSize',14,'Location','best')
    grid on
    ylabel('[N]','FontSize',14)
    set(gca,'FontSize',14)
    title('Thrust generato dai rotori','FontSize',16)

    subplot(3,1,2);
    h2 = plot(time, parametri.k*omega_2.^2, 'r','LineWidth',2);
    legend('Thrust_{2}','FontSize',14,'Location','best')
    grid on
    ylabel('[N]','FontSize',14)
    set(gca,'FontSize',14)


    % dinamica angoli di tilt

    theta1 = x(:,13);
    theta2 = x(:,15);

    figure(2)
    set(gcf,'Position',[100 100 1200 900]) % ingrandisce la finestra

    subplot(4,1,1);
    h1 = plot(time, theta1, 'r','LineWidth',2);
    legend('\theta_1','FontSize',14,'Location','best')
    ylim([-4 4]); grid on
    ylabel('[rad]','FontSize',14)
    title('Andamento angoli di tilt dei rotori','FontSize',16)
    set(gca,'FontSize',14)

    subplot(4,1,2);
    h2 = plot(time, theta2, 'b','LineWidth',2);
    legend('\theta_2','FontSize',14,'Location','best')
    ylim([-4 4]); grid on
    ylabel('[rad]','FontSize',14)
    set(gca,'FontSize',14)

end

% PLOT X1,...,X12

figure(1)
set(gcf, 'Position', [100 100 1200 900])

subplot(4,1,1);
h1 = plot(time, xp, 'r', time, yp, 'b', time, zp, 'g');
set(h1, 'LineWidth', 2)
legend('x_{inertial frame}','y_{inertial frame}','z_{inertial frame}', ...
    'FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-50 50]); grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Posizione [m]', 'FontSize', 14)
title('Andamento stati (x1,...,x12)','FontSize',16)   
set(gca, 'FontSize', 14)


subplot(4,1,2);
h2 = plot(time, xv, 'r', time, yv, 'b', time, zv, 'g');
set(h2, 'LineWidth', 2)
legend('vx_{body frame}','vy_{body frame}','vz_{body frame}', ...
    'FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-50 50]); grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Velocità [m/s]', 'FontSize', 14)
set(gca, 'FontSize', 14)

subplot(4,1,3);
h3 = plot(time, phi, 'r', time, theta, 'b', time, psi, 'g');
set(h3, 'LineWidth', 2)
legend('\phi (roll,x)','\theta (pitch,y)','\psi (yaw,z)', ...
    'FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-10 10]);grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Angoli [rad]', 'FontSize', 14)
set(gca, 'FontSize', 14)

subplot(4,1,4);
h4 = plot(time, p, 'r', time, q, 'b', time, r, 'g');
set(h4, 'LineWidth', 2)
legend('p','q','r', 'FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-10 10]);grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Vel. angolari [rad/s]', 'FontSize', 14)
set(gca, 'FontSize', 14)




%% PLOT angoli di tilt

theta1 = x(:,13);
theta2 = x(:,15);
theta3 = x(:,17);
theta4 = x(:,19);

figure(2)
set(gcf,'Position',[100 100 1200 900]) % ingrandisce la finestra

subplot(4,1,1);
h1 = plot(time, theta1, 'r','LineWidth',2);
legend('\theta_1','FontSize',14,'Location','best')
ylim([-4 4]); grid on
ylabel('[rad]','FontSize',14)
title('Andamento degli angoli di tilt dei rotori','FontSize',16)
set(gca,'FontSize',14)

subplot(4,1,2);
h2 = plot(time, theta2, 'b','LineWidth',2);
legend('\theta_2','FontSize',14,'Location','best')
ylim([-4 4]); grid on
ylabel('[rad]','FontSize',14)
set(gca,'FontSize',14)

subplot(4,1,3);
h3 = plot(time, theta3, 'g','LineWidth',2);
legend('\theta_3','FontSize',14,'Location','best')
ylim([-4 4]); grid on
ylabel('[rad]','FontSize',14)
set(gca,'FontSize',14)

subplot(4,1,4);
h4 = plot(time, theta4, 'k','LineWidth',2);
legend('\theta_4','FontSize',14,'Location','best')
ylim([-4 4]); grid on
xlabel('Time [s]','FontSize',14)
ylabel('[rad]','FontSize',14)
set(gca,'FontSize',14)


%% PLOT dinamica rotori

omega_1 = x(:,21);
omega_2 = x(:,23);
omega_3 = x(:,25);

figure(3)
set(gcf,'Position',[100 100 1000 800]) 

subplot(3,1,1);
h1 = plot(time, omega_1, 'r','LineWidth',2);
legend('\omega_{1}','FontSize',14,'Location','best')
grid on
ylabel('[rad/s]','FontSize',14)
set(gca,'FontSize',14)
title('Andamento velocità angolare dei rotori','FontSize',16) 

subplot(3,1,2);
h2 = plot(time, omega_2, 'r','LineWidth',2);
legend('\omega_{2}','FontSize',14,'Location','best')
grid on
ylabel('[rad/s]','FontSize',14)
set(gca,'FontSize',14)

subplot(3,1,3);
h3 = plot(time, omega_3, 'r','LineWidth',2);
legend('\omega_{3}','FontSize',14,'Location','best')
grid on
xlabel('Time [s]','FontSize',14)
ylabel('[rad/s]','FontSize',14)
set(gca,'FontSize',14)


%% dinamica accelerazione rotori

acc_1 = x(:,22);
acc_2 = x(:,24);
acc_3 = x(:,26);

figure(4) 
set(gcf,'Position',[100 100 1000 800]) % finestra grande

subplot(3,1,1);
h1 = plot(time, acc_1, 'r','LineWidth',2);
legend('acc_{1}','FontSize',14,'Location','best')
grid on
ylabel('[rad/s^2]','FontSize',14)
set(gca,'FontSize',14)
title('Andamento delle accelerazioni angolari dei rotori','FontSize',16) 

subplot(3,1,2);
h2 = plot(time, acc_2, 'r','LineWidth',2);
legend('acc_{2}','FontSize',14,'Location','best')
grid on
ylabel('[rad/s^2]','FontSize',14)
set(gca,'FontSize',14)

subplot(3,1,3);
h3 = plot(time, acc_3, 'r','LineWidth',2);
legend('acc_{3}','FontSize',14,'Location','best')
grid on
xlabel('Time [s]','FontSize',14)
ylabel('[rad/s^2]','FontSize',14)
set(gca,'FontSize',14)

%% posizione e velocità nell'inertial frame (per fare il plot)

vx_i = zeros(size(x,1),1);
vy_i = zeros(size(x,1),1);
vz_i = zeros(size(x,1),1);

for k = 1:size(x,1)
    % Velocità nel body frame
    v_b = [x(k,4); x(k,5); x(k,6)];
    
    % Angoli di Eulero
    phi   = x(k,7);
    theta = x(k,8);
    psi   = x(k,9);
    
    % Matrice di rotazione ZYX
    R_bi = matriceRotazione(phi,theta,psi); % matrice di rotazione                 
    
    % Velocità nel frame inerziale
    v_i = R_bi * v_b;
    
    vx_i(k) = v_i(1);
    vy_i(k) = v_i(2);
    vz_i(k) = -v_i(3); % segno invertito perché z positivo verso il basso
end



figure(5)
set(gcf,'Position',[100 100 1000 800])
h=plot(time, vx_i, 'r', time, vy_i, 'b', time, vz_i, 'g');
set(h, 'LineWidth', 2) 
legend('vx_{inertial frame}','vy_{inertial frame}','vz_{inertial frame}','FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-50 50]); grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Velocità [m/s]', 'FontSize', 14)
set(gca, 'FontSize', 14)
ylim([-30 30]);
title('Velocità lineari nel sistema di riferimento inerziale','FontSize',16) 
grid on

figure(6)
set(gcf,'Position',[100 100 1000 800])
l=plot(time, xp, 'r', time, yp, 'b', time, zp, 'g');
set(l, 'LineWidth', 2) 
legend('x_{inertial frame}','y_{inertial frame}','z_{inertial frame}','FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-50 50]); grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Posizione [m]', 'FontSize', 14)
set(gca, 'FontSize', 14)
ylim([-30 30]);
title('Posizione nel sistema di riferimento inerziale','FontSize',16)
grid on


%% plot controllo

figure(7)
set(gcf,'Position',[100 100 1000 800]) % finestra più grande
for i = 1:3
    subplot(3,1,i)
    h = plot(t, parametri.k*U_values(:,i).^2, 'LineWidth', 2);
    ylabel(sprintf('u_%d [N]', i), 'FontSize',14)
    grid on
    set(gca,'FontSize',14)
    if i==1
        title('Andamento del thrust desiderato dei rotori','FontSize',16) % titolo generale sul primo subplot
    end
    if i==3
        xlabel('Tempo [s]','FontSize',14)
    end
end

figure(8)
set(gcf,'Position',[100 100 1000 900]) % finestra più grande
for i = 4:7
    subplot(4,1,i-3)  
    h = plot(t, U_values(:,i), 'LineWidth', 2);
    ylabel(sprintf('u_%d [rad]', i), 'FontSize',14)
    ylim([-4 4]); grid on
    set(gca,'FontSize',14)
    if i==4
        title('Andamento delle inclinazioni desiderate dei rotori','FontSize',16) % titolo generale sul primo subplot
    end
    if i==7
        xlabel('Tempo [s]','FontSize',14)
    end
end



%% andamento errore
figure(9)
z_des = 10;
vx_des = 25;
vz_des = 0;
set(gcf,'Position',[100 100 1000 800])
l=plot(time, z_des-zp, 'r');
set(l, 'LineWidth', 2) 
legend('e_{z} = z_{des}-z','FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-50 50]); grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Errore ', 'FontSize', 14)
set(gca, 'FontSize', 14)
title('Errore posizione lungo asse Z','FontSize',16)
grid on

figure(10)
set(gcf,'Position',[100 100 1000 800])
l=plot(time, vx_des-vx_i, 'b');
set(l, 'LineWidth', 2) 
legend('e_{vx} = vx_{des}-vx','FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-50 50]); grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Errore ', 'FontSize', 14)
set(gca, 'FontSize', 14)
title('Errore velocità lungo asse X','FontSize',16)
grid on

figure(11)
set(gcf,'Position',[100 100 1000 800])
l=plot(time, vz_des-vz_i, 'g');
set(l, 'LineWidth', 2) 
legend('e_{vz} = vz_{des}-vz','FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-50 50]); grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Errore ', 'FontSize', 14)
set(gca, 'FontSize', 14)
title('Errore velocità lungo asse Z','FontSize',16)
grid on
%%

figure(12)
set(gcf, 'Position', [100 100 1000 800])

% --- vx ---
subplot(3,1,1);
h1=plot(time, xv, 'r', 'LineWidth', 2); hold on;
h2=yline(25,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
grid on; ylim([-10 40])
xlabel('Time [s]', 'FontSize', 14)
ylabel('v_x [m/s]', 'FontSize', 14)
title('Velocità lungo x','FontSize',16)   
set(gca, 'FontSize', 14)
legend([h1 h2], {'v_x','vx_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')


% --- vz ---
subplot(3,1,2);
h3=plot(time, zv, 'b', 'LineWidth', 2); hold on;
h4=yline(0,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
grid on; ylim([-20 20])
xlabel('Time [s]', 'FontSize', 14)
ylabel('v_z [m/s]', 'FontSize', 14)
title('Velocità lungo z','FontSize',16)   
set(gca, 'FontSize', 14)
legend([h3 h4], {'v_z','vz_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')


% --- z (quota) ---
subplot(3,1,3);
h5=plot(time, zp, 'g', 'LineWidth', 2); hold on;
h6=yline(10,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
grid on; ylim([-15 15])
xlabel('Time [s]', 'FontSize', 14)
ylabel('Quota z [m]', 'FontSize', 14)
title('Posizione lungo z','FontSize',16)   
set(gca, 'FontSize', 14)
legend([h5 h6], {'z','z_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')

%%
% thrust genearato dai rotori

    omega_1 = x(:,21);
    omega_2 = x(:,23);

    figure(13)
    set(gcf,'Position',[100 100 1000 800])

    subplot(3,1,1);
    h1 = plot(time, parametri.k*omega_1.^2, 'r','LineWidth',2);
    legend('Thrust_{1}','FontSize',14,'Location','best')
    grid on
    ylabel('[N]','FontSize',14)
    set(gca,'FontSize',14)
    title('Thrust generato dai rotori','FontSize',16)

    subplot(3,1,2);
    h2 = plot(time, parametri.k*omega_2.^2, 'r','LineWidth',2);
    legend('Thrust_{2}','FontSize',14,'Location','best')
    grid on
    ylabel('[N]','FontSize',14)
    set(gca,'FontSize',14)