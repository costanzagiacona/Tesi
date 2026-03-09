%% SIMULAZIONE VTOL 

clear all
clc
close all
clear functions

% flag print
paramFlag = 0; % se 1 print del valore dei parametri
% inizializzazione
test_id = 0;    % flag per cambiare controllo
disturbo = 0;

tspan = [0 30];              % intervallo di simulazione

% flag per fase di volo
% fase = 1 verticale
% fase = 3 orizzontale
fase = 1;       

test_casi = 3;  % flag per cambiare le condizioni di simulazione
% CONTROLLO VERTICALE
% test_casi = 1 => condizioni iniziali ideali
% test_casi = 2 => condizioni iniziali angoli diverse da zero
% test_casi = 3 => disturbo impulsivo
% test_casi = 4 => disturbo a gradino

% CONTROLLO ORIZZONTALE
% test_casi = 1 => condizioni iniziali ideali
% test_casi = 2 => condizioni iniziali velocità inferiori a 25 m/s
% test_casi = 3 => condizioni iniziali angoli diverse da zero 
% test_casi = 4 => disturbo impulsivo


% condizioni desiderate volo orizzontale 
vx_des = 25;
theta_des = 0;
z_des = -10;
target = [vx_des theta_des z_des];

if fase == 1
    test_id = 1;
elseif fase == 3
    test_id = 2;
end

new_model = 1;

% parametri VTOL 
% (roll , pitch , yaw) = (phi , theta , psi)

m = 6.7;      % massa UAV VTOL (Kg)  
g = 9.8;    % costante gravitazionele (m/s^2)
k = 7*(10^-5);      % coeff. thrust rotori
rho = 1.225;    % densità dell'aria (kg/m^3)
ala_y = 0.4;
ala_x = 0.15;
s = ala_x * ala_y;      % superficie alare (m^2) 
v_air = 25;   % velocità relativa alla superficie alare (m/s) lungo x
v_limite = 13.89; % -> 50km/h


b = 0.005*k; % rapporto tra momento torcente e forza del rotore
Ixx = 0.237;
Iyy = 0.244;
Izz = 0.468;

%inerzie dei tre rotori : wing_dx , wing_sx , tail (kg*m^2)

I_rotor_xx = 0.468;
I_rotor_yy = 0.244;
I_rotor_zz = 0.468;

% superfici del tricottero
s_body_x = 0.1;
s_body_y = 0.2*0.7;
s_body_z = 0.15*0.7;

% scelto in modo tale che sia raggiunta la velocità limite di 50 km/h
% (13.89 m/s) in caduta libera
C_d_z = (m*g)/(rho*s_body_z*(v_limite)^2); % coeff. di resistenza (drag) aerodinamica lungo asse z
% C_d_z = 0;
C_d_x = 0.1;
C_d_y = 3;

parametri.s_body_x = s_body_x;
parametri.s_body_y = s_body_y;
parametri.s_body_z = s_body_z;

parametri.C_d_x = C_d_x;
parametri.C_d_y = C_d_y;
parametri.C_d_z = C_d_z;

%distanza (m) tra centro di massa e rotori (per il calcolo del momento dei thrust)
%lungo i vari assi (X_body , Y_body, Z_body)

d_ty = 0;  % assumo il tail rotor posto sull'asse X_body
d_tz = 0;

d_my = (1/2)*ala_y;
d_mz = 0;

d_mx = 0.6;
d_tx = -1.2;



I_body = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];  
I_rotor = (10^-3)*[I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];  

I_rotor_w_dx = I_rotor;
I_rotor_w_sx = I_rotor;
I_rotor_tail = I_rotor;


C_d = ((m-1)*g)/(rho*s*(v_air)^2); %1.28;  %coeff. di resistenza (drag) aerodinamica lungo asse X
C_y = 0; % trascurabile
% C_l = (m*g)/(rho*s*(v_air)^2); %0.854; %coeff. di portanza (lift) aerodinamica 
C_l = (2 * m * g) / (rho * s * v_air^2);

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
parametri.C_y = C_y;
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
if fase == 1
    x0 = zeros(26,1);
else
    x0 = zeros(28,1);            % stato iniziale 
end
if fase == 1

    switch test_casi
        case 1
            x0(4) = 0; % condizione iniziale della velocità lungo X
            x4eq = x0(4);
            x0(3) = 0;
            x0(13)= pi/2;
            x0(15)= pi/2;
            %inclinazione iniziale del rotore di coda (per il volo verticale)
            x0(17) = atan2(-parametri.d_tx*parametri.k, parametri.b);
            x0(19)= -pi/2;

        case 2
            x0(4) = 0; % condizione iniziale della velocità lungo X
            x0(7) = pi/10;
            x0(8) = -pi/8;
            x0(9) = pi/10;
            x0(7) = deg2rad(10);
            x0(8) = deg2rad(10);
            x0(9) = deg2rad(-10);
            x4eq = x0(4);
            x0(3) = 0;
            x0(13)= pi/2;
            x0(15)= pi/2;
            %inclinazione iniziale del rotore di coda (per il volo verticale)
            x0(17) = atan2(-parametri.d_tx*parametri.k, parametri.b);
            x0(19)= -pi/2;

        case 3
            disturbo = 1;
            x0(4) = 0; % condizione iniziale della velocità lungo X
            x4eq = x0(4);
            x0(3) = 0;
            x0(13)= pi/2;
            x0(15)= pi/2;
            %inclinazione iniziale del rotore di coda (per il volo verticale)
            x0(17) = atan2(-parametri.d_tx*parametri.k, parametri.b);
            x0(19)= -pi/2;

        case 4
            disturbo = 2;
            x0(4) = 0; % condizione iniziale della velocità lungo X
            x4eq = x0(4);
            x0(3) = 0;
            x0(13)= pi/2;
            x0(15)= pi/2;
            %inclinazione iniziale del rotore di coda (per il volo verticale)
            x0(17) = atan2(-parametri.d_tx*parametri.k, parametri.b);
            x0(19)= -pi/2;

    end

elseif fase == 3
    
    switch test_casi 
        case 1
            % condizioni ideali
            % posizione iniziale lungo z
            x0(3) = -10;
            x0(4) = 25;
            x4eq = x0(4);

            x0(7) = deg2rad(0);
            x0(8) = deg2rad(0);
            x0(9) = deg2rad(0);

            F_drag = 0.5*parametri.rho*parametri.s_body_x*parametri.C_d_x*sign(x0(4))*x0(4)^2;
            F_drag_ali = parametri.rho*parametri.s*parametri.C_d*sign(x0(4))*x0(4)^2;
            F0_x = F_drag + F_drag_ali;
            T_i = F0_x/2;

            x0(21)= sqrt(T_i/parametri.k);
            x0(23)= x0(21);

        case 2
            % velocità inferiore a quella desiderata
            x0(3) = -10;
            x0(4) = 20;
            x4eq = x0(4);
            F_drag = 0.5*parametri.rho*parametri.s_body_x*parametri.C_d_x*sign(x0(4))*x0(4)^2;
            F_drag_ali = parametri.rho*parametri.s*parametri.C_d*sign(x0(4))*x0(4)^2;
            F0_x = F_drag + F_drag_ali;
            T_i = F0_x/2;
            x0(21)= sqrt(T_i/parametri.k);
            x0(23)= x0(21);

        case 3
            % disturbi
            x0(3) = -10;
            x0(4) = 25;
            x4eq = x0(4);
            x0(7) = deg2rad(10);
            x0(8) = deg2rad(10);
            x0(9) = deg2rad(10);
            F_drag = 0.5*parametri.rho*parametri.s_body_x*parametri.C_d_x*sign(x0(4))*x0(4)^2;
            F_drag_ali = parametri.rho*parametri.s*parametri.C_d*sign(x0(4))*x0(4)^2;
            F0_x = F_drag + F_drag_ali;
            T_i = F0_x/2;
            x0(21)= sqrt(T_i/parametri.k);
            x0(23)= x0(21);

        case 4
            disturbo = 2;
            x0(3) = -10;
            x0(4) = 25;
            x4eq = x0(4);       
            F_drag = 0.5*parametri.rho*parametri.s_body_x*parametri.C_d_x*sign(x0(4))*x0(4)^2;
            F_drag_ali = parametri.rho*parametri.s*parametri.C_d*sign(x0(4))*x0(4)^2;
            F0_x = F_drag + F_drag_ali;
            T_i = F0_x/2;

            x0(21)= sqrt(T_i/parametri.k);
            x0(23)= x0(21);

    end
    
    %inclinazione iniziale dei rotori 
    x0(13) = 0;
    x0(15) = 0;
    x0(17) = 0;
    x0(19) = 0;

end

simbolico = 0;
options = odeset('RelTol',1e-6,'AbsTol',1e-6);
[t, x] = ode45( @(t, x) simulazioneVTOL3(t, x,parametri, test_id, disturbo, target, simbolico), tspan, x0, options);

%per plot controllo
global U_values
U_values = zeros(length(t),7);
for k = 1:length(t)
    U_values(k,:) = controlloVTOL_v3(t(k), parametri,x(k,:), test_id, target);
end

%% GRAFICI
xp = x(:,1);
yp = x(:,2);
zp = -1*x(:,3); % asse z positivo verso il basso

xv = x(:,4);
yv = x(:,5);
zv = -1* x(:,6); % asse z positivo verso il basso
% Inizializza i vettori per i risultati
vz_global = zeros(size(x,1), 1);

for i = 1:size(x,1)
    % Estrai gli angoli all'istante i
    phi_i = x(i, 7); 
    theta_i = x(i, 8); 
    psi_i = x(i, 9);

    % Calcola la matrice per questo istante
    R = matriceRotazione(phi_i, theta_i, psi_i);

    % Velocità body all'istante i
    V_body_i = [x(i,4); x(i,5); x(i,6)];

    % Trasformazione
    V_global_i = R * V_body_i;

    % Salva la componente Z globale
    vz_global(i) = V_global_i(3);
end

zv = vz_global;

phi = rad2deg(x(:,7));
theta = rad2deg(x(:,8));
psi = rad2deg(x(:,9));

p = x(:,10);
q = x(:,11);
r = x(:,12);

time = linspace(0,tspan(2),size(x,1));

flagPlot = 1; % grafici + pallina

plotPallina = 0;% pallina
flagPlot3D = 0; % tricottero 3D

if flagPlot == 1

    % PLOT X1,...,X12

    figure(1)
    set(gcf, 'Position', [100 100 1200 900])

    subplot(4,1,1);
    h1 = plot(time, xp, 'r', time, yp, 'b', time, zp, 'g');
    yline(10,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
    set(h1, 'LineWidth', 2)
    legend('x_{inertial frame}','y_{inertial frame}','z_{inertial frame}', ...
        'FontSize', 14, 'Interpreter','tex', 'Location','best')
    ylim([-20 50]); grid on
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Posizione [m]', 'FontSize', 14)
    title('Andamento stati (x1,...,x12)','FontSize',16)
    set(gca, 'FontSize', 14)


    subplot(4,1,2);
    h2 = plot(time, xv, 'r', time, yv, 'b', time, zv, 'g');
    if fase == 1
        h3 = yline(0,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
    elseif fase == 3
        h3 = yline(25,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
    end
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
    % ylim([-1 1]);
    grid on
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Angoli [grad]', 'FontSize', 14)
    set(gca, 'FontSize', 14)

    subplot(4,1,4);
    h4 = plot(time, p, 'r', time, q, 'b', time, r, 'g');
    set(h4, 'LineWidth', 2)
    legend('p','q','r', 'FontSize', 14, 'Interpreter','tex', 'Location','best')
    ylim([-10 10]);
    grid on
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Vel. angolari [rad/s]', 'FontSize', 14)
    set(gca, 'FontSize', 14)


    figure(2)
    set(gcf, 'Position', [100 100 1200 900])

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
    subplot(3,1,1);
    h5=plot(time, zp, 'g', 'LineWidth', 2); hold on;
    h6=yline(10,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
    grid on; ylim([-16 16])
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Quota z [m]', 'FontSize', 14)
    title('Posizione lungo z','FontSize',16)
    set(gca, 'FontSize', 14)
    legend([h5 h6], {'z','z_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')

    % --- y (posizione lungo y) ---
    subplot(3,1,3);
    h5=plot(time, yp, 'r', 'LineWidth', 2); hold on;
    h6=yline(0,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
    grid on; 
    ylim([-10 10])
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Posizione y [m]', 'FontSize', 14)
    title('Posizione lungo y','FontSize',16)
    set(gca, 'FontSize', 14)
    legend([h5 h6], {'y','y_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')

    % thrust genearato dai rotori

    omega_1 = x(:,21);
    omega_2 = x(:,23);
    omega_3 = x(:,25);

    figure(3)
    set(gcf, 'Position', [100 100 1200 900])

    subplot(3,1,1);
    h1 = plot(time, parametri.k*omega_1.^2, 'r','LineWidth',2);
    legend('Thrust_{1}','FontSize',14,'Location','best')
    ylim([0 100]); grid on
    ylabel('[N]','FontSize',14)
    set(gca,'FontSize',14)
    title('Thrust generato dai rotori','FontSize',16)

    subplot(3,1,2);
    h2 = plot(time, parametri.k*omega_2.^2, 'r','LineWidth',2);
    legend('Thrust_{2}','FontSize',14,'Location','best')
    ylim([0 100]); grid on
    ylabel('[N]','FontSize',14)
    set(gca,'FontSize',14)

    subplot(3,1,3);
    h3 = plot(time, parametri.k*omega_3.^2, 'r','LineWidth',2);
    legend('Thrust_{3}','FontSize',14,'Location','best')
    ylim([0 100]); grid on
    ylabel('[N]','FontSize',14)
    set(gca,'FontSize',14)

    figure(4)
    set(gcf, 'Position', [100 100 1200 900])

    theta1 = rad2deg(x(:,13));
    theta2 = rad2deg(x(:,15));
    theta3 = rad2deg(x(:,17));
    theta4 = rad2deg(x(:,19));

    subplot(4,1,1);
    h1 = plot(time, theta1, 'r','LineWidth',2);
    legend('\theta_1','FontSize',14,'Location','best')
    grid on
    if fase == 1
        ylim([80 100]);
    else
        ylim([-10 10]);
    end
    ylabel('[grad]','FontSize',14)
    title('Andamento angoli di tilt dei rotori','FontSize',16)
    set(gca,'FontSize',14)

    subplot(4,1,2);
    h2 = plot(time, theta2, 'b','LineWidth',2);
    legend('\theta_2','FontSize',14,'Location','best')
    grid on
    if fase == 1
        ylim([80 100]);
    else
        ylim([-10 10]);
    end
    ylabel('[grad]','FontSize',14)
    set(gca,'FontSize',14)

    subplot(4,1,3);
    h3 = plot(time, theta3, 'g','LineWidth',2);
    legend('\theta_3','FontSize',14,'Location','best')
    grid on
    if fase == 1
        ylim([80 100]);
    else
        ylim([-10 10]);
    end
    ylabel('[grad]','FontSize',14)
    title('Andamento angoli di tilt dei rotori','FontSize',16)
    set(gca,'FontSize',14)

    subplot(4,1,4);
    h4 = plot(time, theta4, 'g','LineWidth',2);
    legend('\theta_4','FontSize',14,'Location','best')
    grid on
    if fase == 1
        ylim([-100 -80]);
    else
        ylim([-10 10]);
    end
    ylabel('[grad]','FontSize',14)
    set(gca,'FontSize',14)

end


%%
% close all
% 
% if fase == 1
%     figure(1)
%     set(gcf, 'Position', [100 100 1200 900])
% 
%     subplot(2,1,1);
%     h1 = plot(time, xp, 'r', time, yp, 'b', time, zp, 'g', 'LineWidth',3);
%     yline(10,'--k','LabelHorizontalAlignment','left','FontSize',14,'LineWidth', 3);
%     % set(h1, 'LineWidth', 2)
%     legend('x_{inertial frame}','y_{inertial frame}','z_{inertial frame}', ...
%         'FontSize', 16, 'Interpreter','tex', 'Location','best')
%     ylim([-10 15]);
%     grid on
%     xlabel('Time [s]', 'FontSize', 16)
%     ylabel('Posizione [m]', 'FontSize', 16)
%     title('Posizione','FontSize',17)
%     set(gca, 'FontSize', 16)
% 
%     subplot(2,1,2);
%     h2 = plot(time, xv, 'r', time, yv, 'b', time, zv, 'g', 'LineWidth',3);
%     if fase == 1
%         h3 = yline(0,'--k','LabelHorizontalAlignment','left','FontSize',14,'LineWidth', 3);
%     elseif fase == 3
%         h3 = yline(25,'--k','LabelHorizontalAlignment','left','FontSize',14,'LineWidth', 3);
%     end
%     % set(h2, 'LineWidth', 2)
%     legend('vx_{body frame}','vy_{body frame}','vz_{body frame}', ...
%         'FontSize', 16, 'Interpreter','tex', 'Location','best')
%     ylim([-10 10]); grid on
%     xlabel('Time [s]', 'FontSize', 16)
%     ylabel('Velocità [m/s]', 'FontSize', 16)
%     title('Velocità','FontSize',17)
%     set(gca, 'FontSize', 16)
% end
% 
% 
% if fase == 3
%     figure(1)
%     set(gcf, 'Position', [100 100 1200 900])
% 
%     subplot(3,1,1);
%     plot(time, xp, 'r', 'LineWidth',3);
%     legend('x_{inertial frame}', ...
%         'FontSize', 16, 'Interpreter','tex', 'Location','best')
%     grid on
%     xlabel('Time [s]', 'FontSize', 16)
%     ylabel('Posizione [m]', 'FontSize', 16)
%     title('Posizione','FontSize',17)
%     set(gca, 'FontSize', 16)
% 
%     subplot(3,1,2);
%     plot(time, yp, 'b', 'LineWidth',3);
%     legend('y_{inertial frame}', ...
%         'FontSize', 16, 'Interpreter','tex', 'Location','best')
%     grid on
%     ylim([-10 10])
%     xlabel('Time [s]', 'FontSize', 16)
%     ylabel('Posizione [m]', 'FontSize', 16)
%     % title('Posizione','FontSize',17)
%     set(gca, 'FontSize', 16)
% 
%     subplot(3,1,3);
%     plot( time, zp, 'g', 'LineWidth',3);
%     yline(10,'--k','LabelHorizontalAlignment','left','FontSize',14,'LineWidth', 3);
%     legend('z_{inertial frame}', ...
%         'FontSize', 16, 'Interpreter','tex', 'Location','best')
%     grid on
%     ylim([5 15])
%     xlabel('Time [s]', 'FontSize', 16)
%     ylabel('Posizione [m]', 'FontSize', 16)
%     % title('Posizione','FontSize',17)
%     set(gca, 'FontSize', 16)
% 
%     figure(6)
%     set(gcf, 'Position', [100 100 1200 900])
% 
%     subplot(3,1,1);
%     plot(time, xv, 'r', 'LineWidth',3);
%     yline(25,'--k','LabelHorizontalAlignment','left','FontSize',14,'LineWidth', 3);
%     legend('vx_{body frame}', ...
%         'FontSize', 16, 'Interpreter','tex', 'Location','best')
%     grid on
%     ylim([15 30])
%     xlabel('Time [s]', 'FontSize', 16)
%     ylabel('Velocità [m/s]', 'FontSize', 16)
%     title('Velocità','FontSize',17)
%     set(gca, 'FontSize', 16)
% 
%     subplot(3,1,2);
%     plot(time, yv, 'b', 'LineWidth',3);
%     legend('vy_{body frame}', ...
%         'FontSize', 16, 'Interpreter','tex', 'Location','best')
%     grid on
%     ylim([-10 10])
%     xlabel('Time [s]', 'FontSize', 16)
%     ylabel('Velocità [m/s]', 'FontSize', 16)
%     % title('Velocità','FontSize',17)
%     set(gca, 'FontSize', 16)
% 
%     subplot(3,1,3);
%     plot( time, zv, 'g', 'LineWidth',3);
%     legend('vz_{body frame}', ...
%         'FontSize', 16, 'Interpreter','tex', 'Location','best')
%     grid on
%     ylim([-10 10])
%     xlabel('Time [s]', 'FontSize', 16)
%     ylabel('Velocità [m/s]', 'FontSize', 16)
%     % title('Velocità','FontSize',17)
%     set(gca, 'FontSize', 16)
% 
% end
% 
% 
% 
% figure(2)
% set(gcf,'Position',[100 100 1200 800])
% 
% % --- ANGOLI ---
% subplot(2,1,1)
% plot(time,phi,'r',time,theta,'b',time,psi,'g','LineWidth',3)
% grid on
% xlabel('Time [s]', 'FontSize', 16)
% ylabel('Angoli [rad]')
% title('Angoli di Eulero')
% legend('\phi','\theta','\psi','Location','best')
% set(gca,'FontSize',16)
% if fase == 1
%     ylim([-1 1]);
% else
%     ylim([-10 10]);
% end
% 
% % --- VELOCITÀ ANGOLARI ---
% subplot(2,1,2)
% plot(time,p,'r',time,q,'b',time,r,'g','LineWidth',3)
% grid on
% xlabel('Time [s]')
% ylabel('Vel. angolari [rad/s]')
% title('Velocità angolari')
% legend('p','q','r','Location','best')
% set(gca,'FontSize',16)
% ylim([-1 1])
% 
% 
% figure(3)
% set(gcf,'Position',[100 100 1200 800])
% 
% omega_1 = x(:,21);
% omega_2 = x(:,23);
% omega_3 = x(:,25);
% 
% subplot(3,1,1);
% h1 = plot(time, parametri.k*omega_1.^2, 'r','LineWidth',3);
% legend('Thrust_{1}','FontSize',16,'Location','best')
% ylim([0 100]); grid on
% xlabel('Time [s]', 'FontSize', 16)
% ylabel('[N]','FontSize',16)
% set(gca,'FontSize',16)
% title('Thrust generato dai rotori','FontSize',17)
% 
% subplot(3,1,2);
% h2 = plot(time, parametri.k*omega_2.^2, 'r','LineWidth',3);
% legend('Thrust_{2}','FontSize',16,'Location','best')
% ylim([0 100]); grid on
% xlabel('Time [s]', 'FontSize', 16)
% ylabel('[N]','FontSize',16)
% set(gca,'FontSize',16)
% 
% subplot(3,1,3);
% h3 = plot(time, parametri.k*omega_3.^2, 'r','LineWidth',3);
% legend('Thrust_{3}','FontSize',16,'Location','best')
% ylim([0 100]); grid on
% xlabel('Time [s]', 'FontSize', 16)
% ylabel('[N]','FontSize',16)
% set(gca,'FontSize',16)
% 
% 
% figure(4)
% set(gcf,'Position',[100 100 1200 900])
% 
% theta1 = rad2deg(x(:,13));
% theta2 = rad2deg(x(:,15));
% theta3 = rad2deg(x(:,17));
% theta4 = rad2deg(x(:,19));
% 
% subplot(2,1,1);
% h1 = plot(time, theta1, 'r','LineWidth',3);
% legend('\theta_1','FontSize',16,'Location','best')
% xlabel('Time [s]', 'FontSize', 16)
% grid on
% if fase == 1
%     ylim([80 100]);
% else
%     ylim([-10 10]);
% end
% ylabel('[grad]','FontSize',16)
% title('Andamento angoli di tilt dei rotori anteriori','FontSize',17)
% set(gca,'FontSize',16)
% 
% subplot(2,1,2);
% h2 = plot(time, theta2, 'b','LineWidth',3);
% legend('\theta_2','FontSize',16,'Location','best')
% grid on
% if fase == 1
%     ylim([80 100]);
% else
%     ylim([-10 10]);
% end
% xlabel('Time [s]', 'FontSize', 16)
% ylabel('[grad]','FontSize',16)
% set(gca,'FontSize',16)
% 
% figure(5)
% set(gcf, 'Position', [100 100 1200 900])
% subplot(2,1,1);
% h3 = plot(time, theta3, 'g','LineWidth',3);
% legend('\theta_3','FontSize',16,'Location','best')
% grid on
% xlabel('Time [s]', 'FontSize', 16)
% ylabel('[grad]','FontSize',16)
% title('Andamento angoli di tilt del rotore posteriore','FontSize',17)
% set(gca,'FontSize',16)
% 
% subplot(2,1,2);
% h4 = plot(time, theta4, 'g','LineWidth',3);
% legend('\theta_4','FontSize',16,'Location','best')
% grid on
% xlabel('Time [s]', 'FontSize', 16)
% ylabel('[grad]','FontSize',16)
% set(gca,'FontSize',16)
% 
% xlabel('Time [s]')
% 
% 
