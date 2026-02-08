%% PLOT PRESENTATION SUITE - Solo Grafici Chiave
clc; close all;

% Lista File (Assicurati che i nomi coincidano con quelli nella cartella)
files = {'Results_Recovery_{Hover}.mat', ...
         'Results_Takeoff_{Ground}.mat', ...
         'Results_Cruise_{Flight}.mat'};

% Impostazioni Grafiche "Publication Quality"
set(0, 'DefaultAxesFontSize', 14);       % Font assi leggibile
set(0, 'DefaultLineLineWidth', 1.0);     % Linee marcate
set(0, 'DefaultAxesLineWidth', 1.2);     % Bordi assi marcati

%% CICLO DI GENERAZIONE
for k = 1:length(files)
    filename = files{k};
    
    if ~isfile(filename)
        fprintf('[SKIP] File %s non trovato.\n', filename);
        continue;
    end
    
    fprintf('>> Elaborazione %s ...\n', filename);
    data = load(filename);
    
    if isfield(data, 'risultati') && isfield(data, 'cfg')
        % SELETTORE SCENARIO INTELLIGENTE
        % Sceglie quale grafico fare in base al nome dello scenario
        if contains(data.cfg.name, 'Takeoff', 'IgnoreCase', true)
            Plot_Takeoff_Focus(data.risultati, data.cfg);
            
        elseif contains(data.cfg.name, 'Recovery', 'IgnoreCase', true)
            Plot_Recovery_Focus(data.risultati, data.cfg);
            
        elseif contains(data.cfg.name, 'Cruise', 'IgnoreCase', true)
            Plot_Cruise_Focus(data.risultati, data.cfg);
        end
    end
end
fprintf('>> Grafici pronti.\n');

%% ---------------------------------------------------------
%  FUNZIONE 1: DECOLLO (Solo Quota Z)
%  ---------------------------------------------------------
function Plot_Takeoff_Focus(risultati, cfg)
    figure(1); clf;
    set(gcf, 'Name', 'Slide Decollo - Quota', 'Position', [100 100 800 500]);
    hold on; grid on; grid minor;
    
    N = length(risultati);
    target_z = -cfg.target(3); % Invertiamo segno per plot (Quota Positiva)
    
    % Colore Verde Smeraldo semitrasparente
    col_line = [0 0.6 0.3 0.15]; 
    
    for i = 1:N
        if risultati(i).converged
            t = risultati(i).t;
            z = -risultati(i).x(:,3); % NED negativo -> Quota positiva
            plot(t, z, 'Color', col_line, 'LineWidth', 1.5);
        end
    end
    
    yline(target_z, '--k', 'Target (10m)', 'LineWidth', 2);
    
    title('Scenario Decollo: Tracking di Quota', 'FontSize', 16);
    xlabel('Tempo [s]', 'FontSize', 14);
    ylabel('Quota dal suolo [m]', 'FontSize', 14);
    xlim([0 20]); % Zoom sui primi 20 secondi (il transitorio)
    ylim([0 target_z + 2]);
end

%% ---------------------------------------------------------
%  FUNZIONE 2: RECOVERY (Spaghetti Plot Posizione)
%  ---------------------------------------------------------
function Plot_Recovery_Focus(risultati, cfg)
    figure(2); clf;
    set(gcf, 'Name', 'Slide Recovery - Convergenza', 'Position', [150 150 900 600]);
    
    N = length(risultati);
    % Colori: Rosso (N), Blu (E), Verde (D/Quota)
    col_n = [0.8 0 0 0.15];
    col_e = [0 0 0.8 0.15];
    col_d = [0 0.6 0 0.15];
    
    subplot(3,1,1); hold on; grid on;
    title('Scenario Recovery: Convergenza Posizione', 'FontSize', 16);
    ylabel('X [m]');
    
    subplot(3,1,2); hold on; grid on;
    ylabel('Y [m]');
    
    subplot(3,1,3); hold on; grid on;
    ylabel('Z [m]'); xlabel('Tempo [s]');
    
    for i = 1:N
        if risultati(i).converged && ~risultati(i).crashed
            t = risultati(i).t;
            x = risultati(i).x;
            
            subplot(3,1,1); plot(t, x(:,1), 'Color', col_n);
            subplot(3,1,2); plot(t, x(:,2), 'Color', col_e);
            subplot(3,1,3); plot(t, -x(:,3), 'Color', col_d);
        end
    end
    
    % Linee di target
    subplot(3,1,1); yline(cfg.target(1), '--k');
    subplot(3,1,2); yline(cfg.target(2), '--k');
    subplot(3,1,3); yline(-cfg.target(3), '--k', 'Target Quota');
end

%% ---------------------------------------------------------
%  FUNZIONE 3: CROCIERA (Vx "Tubo" + Quota)
%  ---------------------------------------------------------
function Plot_Cruise_Focus(risultati, cfg)
    figure(3); clf;
    set(gcf, 'Name', 'Slide Cruise - Performance', 'Position', [200 200 800 600]);
    
    N = length(risultati);
    target_vx = 35;
    target_z = -cfg.target(3);
    
    col_vx = [0.8 0.1 0.1 0.2]; % Rosso scuro trasparente
    col_z  = [0.1 0.6 0.1 0.2]; % Verde scuro trasparente
    
    % Subplot 1: Velocità Vx
    subplot(2,1,1); hold on; grid on; grid minor;
    title('Scenario Crociera: Precisione Longitudinale', 'FontSize', 16);
    ylabel('Velocità V_x [m/s]', 'FontSize', 14);
    
    for i = 1:N
        if risultati(i).converged
            t = risultati(i).t;
            vx = risultati(i).x(:,4);
            plot(t, vx, 'Color', col_vx);
        end
    end
    yline(target_vx, '--k', 'Target (35 m/s)', 'LineWidth', 2);
    ylim([target_vx-5, target_vx+5]); % Zoom stretto per mostrare la precisione
    
    % Subplot 2: Quota (Per mostrare assenza ballooning)
    subplot(2,1,2); hold on; grid on; grid minor;
    ylabel('Z [m]', 'FontSize', 14);
    xlabel('Tempo [s]', 'FontSize', 14);
    
    for i = 1:N
        if risultati(i).converged
            t = risultati(i).t;
            z = -risultati(i).x(:,3);
            plot(t, z, 'Color', col_z);
        end
    end
    yline(target_z, '--k', 'Target (10 m)', 'LineWidth', 2);
    ylim([target_z-1, target_z+1]); % Zoom stretto
end