
plotTesi = 0;

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
    omega_3 = x(:,25);

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

    subplot(3,1,3);
    h3 = plot(time, parametri.k*omega_3.^2, 'r','LineWidth',2);
    legend('Thrust_{3}','FontSize',14,'Location','best')
    grid on
    ylabel('[N]','FontSize',14)
    set(gca,'FontSize',14)