%% PLOT PRESENTATION SUITE - Con Legende Professionali
close all;

files = {'Results_SMCRecovery_{Hover}.mat', ...
         'Results_SMCTakeoff_{Ground}.mat', ...
         'Results_SMCCruise_{Flight}.mat'};

set(0, 'DefaultAxesFontSize', 14);       
set(0, 'DefaultLineLineWidth', 1.0);     
set(0, 'DefaultAxesLineWidth', 1.2);     
set(0, 'DefaultLegendFontSize', 12);

for k = 1:length(files)
    filename = files{k};
    if ~isfile(filename); continue; end
    data = load(filename);
    
    if isfield(data, 'risultati') && isfield(data, 'cfg')
        if contains(data.cfg.name, 'Takeoff', 'IgnoreCase', true)
            Plot_Takeoff_Focus(data.risultati, data.cfg);
        elseif contains(data.cfg.name, 'Recovery', 'IgnoreCase', true)
            Plot_Recovery_Focus(data.risultati, data.cfg);
        elseif contains(data.cfg.name, 'Cruise', 'IgnoreCase', true)
            Plot_Cruise_Focus(data.risultati, data.cfg);
        end
    end
end

%% ---------------------------------------------------------
%  FUNZIONE 1: DECOLLO (Quota + Spinta con Legende)
%  ---------------------------------------------------------
function Plot_Takeoff_Focus(risultati, cfg)
    figure(1); clf;
    set(gcf, 'Name', 'Decollo: Quota e Spinta', 'Position', [100 100 800 600]);
    
    N = length(risultati);
    target_z = -cfg.target(3);
    col_z = [0 0.6 0.3 0.15];  
    col_t = [0 0.4 0.8 0.15];  
    
    k_val = 7*(10^-5); m_val = 6.7; g_val = 9.8;
    weight = m_val * g_val;

    % Subplot 1: Quota
    subplot(2,1,1); hold on; grid on; grid minor;
    h_z = plot(NaN, NaN, 'Color', col_z(1:3), 'LineWidth', 2); % Proxy Legenda
    title('Decollo', 'FontSize', 16);
    ylabel('Quota [m]', 'FontWeight', 'bold');
    
    % Subplot 2: Spinta
    subplot(2,1,2); hold on; grid on; grid minor;
    h_t = plot(NaN, NaN, 'Color', col_t(1:3), 'LineWidth', 2); % Proxy Legenda
    ylabel('Spinta Totale [N]', 'FontWeight', 'bold');
    xlabel('Tempo [s]');

    for i = 1:N
        if risultati(i).converged
            t = risultati(i).t;
            z = -risultati(i).x(:,3);
            thrust_tot = k_val * (risultati(i).x(:,21).^2 + risultati(i).x(:,23).^2 + risultati(i).x(:,25).^2);
            subplot(2,1,1); plot(t, z, 'Color', col_z);
            subplot(2,1,2); plot(t, thrust_tot, 'Color', col_t);
        end
    end
    
    subplot(2,1,1); 
    h_tz = yline(target_z, '--k', 'LineWidth', 2);
    legend([h_z, h_tz], {'Traiettorie Quota', 'Target (10m)'}, 'Location', 'southeast');
    ylim([0 target_z + 5]);

    subplot(2,1,2); 
    h_w = yline(weight, '--r', 'LineWidth', 2);
    legend([h_t, h_w], {'Spinta Calcolata', 'Peso (mg)'}, 'Location', 'southeast');
    ylim([0 weight + 40]);
end

%% ---------------------------------------------------------
%  FUNZIONE 2: RECOVERY (Assetto e Attuatori con Legende)
%  ---------------------------------------------------------
function Plot_Recovery_Focus(risultati, cfg)
    figure(2); clf;
    set(gcf, 'Name', 'Recovery: Assetto e Attuatori', 'Position', [100 100 1000 700]);
    
    N = length(risultati);
    col_v = [0 0.45 0.74 0.1]; col_th = [0.85 0.33 0.1 0.1]; col_tilt = [0.93 0.69 0.13 0.1];

    subplot(3,1,1); hold on; grid on;
    h_v = plot(NaN, NaN, 'Color', col_v(1:3), 'LineWidth', 2); % Proxy
    ylabel('V_x [m/s]', 'FontWeight', 'bold');
    title('Recovery: Accoppiamento Velocità-Assetto-Rotori', 'FontSize', 16);
    
    subplot(3,1,2); hold on; grid on;
    h_th = plot(NaN, NaN, 'Color', col_th(1:3), 'LineWidth', 2); % Proxy
    ylabel('Pitch \theta [deg]', 'FontWeight', 'bold');
    
    subplot(3,1,3); hold on; grid on;
    h_tl = plot(NaN, NaN, 'Color', col_tilt(1:3), 'LineWidth', 2); % Proxy
    ylabel('Tilt Servi [deg]', 'FontWeight', 'bold');
    xlabel('Tempo [s]');

    for i = 1:N
        if risultati(i).converged && ~risultati(i).crashed
            t = risultati(i).t;
            vx = risultati(i).x(:,4);
            theta = rad2deg(risultati(i).x(:,8));
            tilt_1 = rad2deg(risultati(i).x(:,13));
            subplot(3,1,1); plot(t, vx, 'Color', col_v);
            subplot(3,1,2); plot(t, theta, 'Color', col_th);
            subplot(3,1,3); plot(t, tilt_1, 'Color', col_tilt);
        end
    end
    
    subplot(3,1,1); legend(h_v, 'Velocità v_x', 'Location', 'northeast');
    subplot(3,1,2); legend(h_th, 'Assetto Pitch', 'Location', 'northeast');
    subplot(3,1,3); legend(h_tl, 'Angolo Tilt Rotori', 'Location', 'northeast');
end

%% ---------------------------------------------------------
%  FUNZIONE 3: CROCIERA (Vx + Pitch con Legende)
%  ---------------------------------------------------------
function Plot_Cruise_Focus(risultati, cfg)
    figure(3); clf;
    set(gcf, 'Name', 'Cruise: Precisione e Assetto', 'Position', [200 200 900 600]);
    
    N = length(risultati);
    col_vx = [0.6350 0.0780 0.1840 0.15]; col_th = [0 0.4470 0.7410 0.15];
    
    subplot(2,1,1); hold on; grid on;
    h_vx = plot(NaN, NaN, 'Color', col_vx(1:3), 'LineWidth', 2); % Proxy
    ylabel('V_x [m/s]', 'FontWeight', 'bold');
    title('Crociera', 'FontSize', 16);
    
    subplot(2,1,2); hold on; grid on;
    h_th = plot(NaN, NaN, 'Color', col_th(1:3), 'LineWidth', 2); % Proxy
    ylabel('Pitch \theta [deg]', 'FontWeight', 'bold');
    xlabel('Tempo [s]');

    for i = 1:N
        if risultati(i).converged
            t = risultati(i).t;
            vx = risultati(i).x(:,4);
            theta = rad2deg(risultati(i).x(:,8));
            subplot(2,1,1); plot(t, vx, 'Color', col_vx);
            subplot(2,1,2); plot(t, theta, 'Color', col_th);
        end
    end
    
    subplot(2,1,1); 
    h_tvx = yline(25, '--k', 'LineWidth', 2);
    legend([h_vx, h_tvx], {'Inseguimento v_x', 'Target (25 m/s)'}, 'Location', 'southeast');
    ylim([20 30]); 

    subplot(2,1,2); 
    legend(h_th, 'Assetto di Crociera', 'Location', 'southeast');
    ylim([-10 10]); 
end