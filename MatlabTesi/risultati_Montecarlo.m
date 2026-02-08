%% PLOT COMPLETE SUITE - Visualizzazione Totale (Recovery + Takeoff + Cruise)
clc; close all;

% Lista dei file generati dalla MonteCarloVTOL
files = {'Results_Recovery_{Hover}.mat', ...
         'Results_Takeoff_{Ground}.mat', ...
         'Results_Cruise_{Flight}.mat'};

% Offset per i numeri di figura per non sovrapporle
% Recovery: Fig 1-4 | Takeoff: Fig 11-14 | Cruise: Fig 21-24
offsets = [0, 10, 20]; 

% Parametri grafici globali
set(0, 'DefaultLineLineWidth', 1.0);
set(0, 'DefaultAxesFontSize', 14);
%%
for k = 1:length(files)
    filename = files{k};
    fig_offset = offsets(k);
    
    if ~isfile(filename)
        fprintf('[AVVISO] File %s non trovato. Salto questa fase.\n', filename);
        continue;
    end
    
    % Caricamento dati
    fprintf('>> Caricamento %s ...\n', filename);
    data = load(filename); 
    
    % Estrazione variabili
    if isfield(data, 'risultati') && isfield(data, 'cfg')
        risultati = data.risultati;
        cfg = data.cfg;
        
        % Generazione Grafici per questa fase
        GeneratePlots(risultati, cfg, fig_offset);
    else
        fprintf('[ERRORE] Il file %s non contiene le strutture "risultati" o "cfg".\n', filename);
    end
end
fprintf('>> Generazione grafici completata.\n');

%% FUNZIONE DI PLOTTING LOCALE
function GeneratePlots(risultati, cfg, fig_offset)
    
    N_sim = length(risultati);
    target = cfg.target; % [Vx_ned, Pitch, Z_ned] (Attenzione: target è in NED!)
    name = cfg.name;
    
    % Colori per spaghetti plot (con trasparenza alpha)
    % Nota: MATLAB standard non supporta alpha nelle linee facilmente senza patch,
    % usiamo colori pastello chiari per simulare la trasparenza se sono tanti.
    col_x = [1 0.4 0.4 0.3]; % Rosso chiaro
    col_y = [0.4 0.4 1 0.3]; % Blu chiaro
    col_z = [0.4 0.8 0.4 0.3]; % Verde chiaro
    
    % Parametro Thrust (approssimato per il plot)
    k_thrust = 1.5e-5; % Valore tipico se non presente in cfg
    
    %% FIGURE 1 (+offset): PANORAMICA STATI (X, V, Angoli, Ratei)
    hFig1 = figure(1 + fig_offset);
    set(hFig1, 'Name', [name ' - Stati Completi'], 'Position', [50 50 1000 800]);
    
    % 1. Posizione
    subplot(4,1,1); hold on; grid on;
    title(['[' name '] Traiettorie Posizione'], 'FontSize', 12, 'FontWeight', 'bold');
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            t = risultati(i).t; x = risultati(i).x;
            plot(t, x(:,1), 'Color', [0.8 0 0 0.2]); % X (Rosso)
            plot(t, x(:,2), 'Color', [0 0 0.8 0.2]); % Y (Blu)
            plot(t, -x(:,3), 'Color', [0 0.6 0 0.2]); % Quota (-Z) (Verde)
        end
    end
    % Linea Target Quota (target(3) è negativo in NED, quindi plottiamo -target(3))
    yline(-target(3), '--k', 'Target Quota', 'LineWidth', 1.5);
    ylabel('Pos [m]'); legend('X', 'Y', 'Quota', 'Location', 'best');
    
    % 2. Velocità
    subplot(4,1,2); hold on; grid on;
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            t = risultati(i).t; x = risultati(i).x;
            plot(t, x(:,4), 'Color', [0.8 0 0 0.2]); % Vx
            plot(t, x(:,5), 'Color', [0 0 0.8 0.2]); % 
            plot(t, -x(:,6), 'Color', [0 0.6 0 0.2]); % 
        end
    end
    % Target Velocità (solo per Cruise o se diverso da 0)
    if cfg.test_id == 2 % Cruise
        yline(target(1), '--r', 'Target Vx', 'LineWidth', 1.5);
    else
        yline(0, '--r', 'Ref Vx', 'LineWidth', 1.0);
    end
    ylabel('Vx [m/s]');
    
    % 3. Angoli (Assetto)
    subplot(4,1,3); hold on; grid on;
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            t = risultati(i).t; x = risultati(i).x;
            plot(t, rad2deg(x(:,7)), 'Color', [0.8 0 0 0.2]); % Roll
            plot(t, rad2deg(x(:,8)), 'Color', [0 0 0.8 0.2]); % Pitch
        end
    end
    ylabel('Ang [deg]'); legend('Roll', 'Pitch');
    % Target Pitch (se rilevante)
    if abs(target(2)) > 0.01
        yline(rad2deg(target(2)), '--b', 'Target Pitch');
    end

    % 4. Ratei
    subplot(4,1,4); hold on; grid on;
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            t = risultati(i).t; x = risultati(i).x;
            plot(t, rad2deg(x(:,12)), 'Color', [0 0.6 0 0.2]); % Yaw Rate (r)
        end
    end
    ylabel('Yaw Rate [deg/s]'); xlabel('Time [s]');
    
    %% FIGURE 2 (+offset): FOCUS CONTROLLO (Z, Vz, Y)
    hFig2 = figure(2 + fig_offset);
    set(hFig2, 'Name', [name ' - Performance Controllo'], 'Position', [100 100 800 600]);
    
    % 1. Quota Z (Zoom)
    subplot(2,1,1); hold on; grid on;
    title(['[' name '] Tracking Quota'], 'FontSize', 12, 'FontWeight', 'bold');
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            plot(risultati(i).t, -risultati(i).x(:,3), 'Color', [0 0.6 0 0.3]);
        end
    end
    yline(-target(3), '--k', 'Target', 'LineWidth', 2);
    ylabel('Quota [m]'); 
    
    % 2. Velocità Verticale (Vz Inerziale)
    subplot(2,1,2); hold on; grid on;
    ylabel('Vz Inerziale [m/s]');
    xlabel('Time [s]');
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            t = risultati(i).t; x = risultati(i).x;
            % Calcoliamo Vz inerziale approssimata (w_earth)
            % w_earth approx = w_body * cos(phi)cos(theta) - u*sin(theta) + ...
            % Per semplicità usiamo solo Vz body ruotata se gli angoli sono piccoli
            % O meglio: ricalcoliamola bene
            vz_inertial = zeros(length(t),1);
            for k=1:length(t)
                % Matrice rotazione Body -> NED
                phi = x(k,7); theta = x(k,8); psi = x(k,9);
                R_row3 = [-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
                v_body = x(k,4:6)';
                vz_ned = R_row3 * v_body; 
                vz_inertial(k) = -vz_ned; % Positiva a salire
            end
            plot(t, vz_inertial, 'Color', [0 0 1 0.2]);
        end
    end
    yline(0, '--k');
    
    %% FIGURE 3 (+offset): ATTUATORI (RPM Motori)
    hFig3 = figure(3 + fig_offset);
    set(hFig3, 'Name', [name ' - Motori'], 'Position', [150 150 800 600]);
    
    subplot(3,1,1); hold on; grid on; title(['[' name '] RPM Motori'], 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Motore 1 (DX) [rad/s]');
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            plot(risultati(i).t, risultati(i).x(:,21), 'Color', [1 0 0 0.1]);
        end
    end
    
    subplot(3,1,2); hold on; grid on; ylabel('Motore 2 (SX) [rad/s]');
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            plot(risultati(i).t, risultati(i).x(:,23), 'Color', [0 0 1 0.1]);
        end
    end
    
    subplot(3,1,3); hold on; grid on; ylabel('Motore 3 (Coda) [rad/s]');
    xlabel('Time [s]');
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            plot(risultati(i).x(:,25), 'Color', [0 1 0 0.1]); % Errore nel plot originale corretto qui
            % Nota: plotto vs t
            plot(risultati(i).t, risultati(i).x(:,25), 'Color', [0 0.6 0 0.1]);
        end
    end

    %% FIGURE 4 (+offset): SERVI TILT
    hFig4 = figure(4 + fig_offset);
    set(hFig4, 'Name', [name ' - Tilt'], 'Position', [200 200 800 600]);
    
    subplot(1,1,1); hold on; grid on; title(['[' name '] Angoli Tilt'], 'FontSize', 12, 'FontWeight', 'bold');
    for i=1:N_sim
        if risultati(i).converged && ~risultati(i).crashed
            % Plot solo Tilt 1 (DX) per pulizia
            plot(risultati(i).t, rad2deg(risultati(i).x(:,13)), 'Color', [0 0 0 0.2]); 
        end
    end
    ylabel('Tilt Anteriore [deg]'); xlabel('Time [s]');
    if cfg.test_id == 1 || strcmp(cfg.x0_type, 'takeoff')
        yline(90, '--r', 'Hover (90°)', 'LineWidth', 2);
    else
        yline(0, '--r', 'Cruise (0°)', 'LineWidth', 2);
    end
    
end