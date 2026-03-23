%% PLOT PRESENTATION SUITE - Kinematic, Dynamic & Actuator Analysis
close all; clc;
files = {'Results1_Recovery Hover.mat', ...
         'Results1_Takeoff Ground.mat',...
         'Results1_Cruise Flight.mat'};

files = {'Results1_Cruise Flight.mat'};
% files = {'Results1_Takeoff Ground.mat'};
% files = {'Results1_Recovery Hover.mat'};

% Impostazioni grafiche per la leggibilità accademica
set(0, 'DefaultAxesFontSize', 20);       
set(0, 'DefaultLineLineWidth', 1.0);     
set(0, 'DefaultAxesLineWidth', 1.4);     
set(0, 'DefaultLegendFontSize', 12);

for k = 1:length(files)
    filename = files{k};
    if ~isfile(filename); continue; end
    data = load(filename);
    
    if isfield(data, 'risultati') && isfield(data, 'cfg')
        scenarioName = extractBetween(data.cfg.name, '_', '_');
        if isempty(scenarioName), scenarioName = {data.cfg.name}; end
        
        fprintf('Analisi in corso: %s...\n', char(scenarioName));
        Plot_State_Overview(data.cfg, data.risultati, char(scenarioName));
    end
end

%% ---------------------------------------------------------
%  FUNZIONE UNICA: VISUALIZZAZIONE COMPLETA (STATI + ATTUATORI)
%  ---------------------------------------------------------
function Plot_State_Overview(cfg, risultati, label)
    % --- CONFIGURAZIONE INDICI E COSTANTI (DA VERIFICARE) ---
    idx_tilt = 13;          % Indice stato Tilt nel vettore x
    idx_rotors = [21, 23, 25]; % Indici velocità rotori (Omega). Aggiungi altri se necessario
    k_thrust = 7e-5;        % Coefficiente di spinta (N/(rad/s)^2)
    m_val = 6.7; g_val = 9.81; % Per riferimento peso
    weight = m_val * g_val;
    % --------------------------------------------------------

    % Colori semi-trasparenti per Monte Carlo
    c_pos = [0 0.4470 0.7410 0.2]; 
    c_vel = [0.8500 0.3250 0.0980 0.2]; 
    c_att = [0.4660 0.6740 0.1880 0.2]; 
    c_rat = [0.4940 0.1840 0.5560 0.2]; 
    c_act = [0.6350 0.0780 0.1840 0.2]; % Rosso scuro per attuatori

    N = length(risultati);
    
    % --- CREAZIONE FIGURE ---
    figs(1) = figure('Name', [label, ': Posizione'], 'Position', [10 50 500 700]);
    figs(2) = figure('Name', [label, ': Velocita'], 'Position', [520 50 500 700]);
    figs(3) = figure('Name', [label, ': Assetto'], 'Position', [1030 50 500 700]);
    figs(4) = figure('Name', [label, ': Ratei'], 'Position', [10 50 500 700]); 
    figs(5) = figure('Name', [label, ': Attuatori (Tilt)'], 'Position', [520 50 500 700]); 
    figs(6) = figure('Name', [label, ': Attuatori (Thrust)'], 'Position', [1030 50 500 700]);
    
    % --- PARAMETRI ROTORI ---
    num_rotors = length(idx_rotors);
    % Mappatura corretta per estrarre solo gli angoli (saltando le derivate)
    idx_tilt_vec = [13, 15, 17];
    rotor_names_tilt = {'Tilt Rotore 2', 'Tilt Rotore 1', 'Tilt Rotore 3'}; 
    rotor_names_thrust = {'Thrust Rotore 2', 'Thrust Rotore 1', 'Thrust Rotore 3'}; 
    rotor_colors = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880];

    % =====================================================================
    % 1. CICLO DI TRACCIAMENTO DATI (Solo plot delle curve, massima velocità)
    % =====================================================================
    for i = 1:N
        if risultati(i).converged
            t = risultati(i).t;
            t = 25;
            X = risultati(i).x; 
            
            % Posizione
            set(0, 'CurrentFigure', figs(1));
            subplot(3,1,1); hold on; plot(t, X(:,1), 'Color', c_pos);
            subplot(3,1,2); hold on; plot(t, X(:,2), 'Color', c_pos);
            subplot(3,1,3); hold on; plot(t, -X(:,3), 'Color', c_pos);
            
            % Velocità
            set(0, 'CurrentFigure', figs(2));
            subplot(3,1,1); hold on; plot(t, X(:,4), 'Color', c_vel);
            subplot(3,1,2); hold on; plot(t, X(:,5), 'Color', c_vel);
            subplot(3,1,3); hold on; plot(t, X(:,6), 'Color', c_vel);
            
            % Assetto
            set(0, 'CurrentFigure', figs(3));
            subplot(3,1,1); hold on; plot(t, rad2deg(X(:,7)), 'Color', c_att);
            subplot(3,1,2); hold on; plot(t, rad2deg(X(:,8)), 'Color', c_att);
            subplot(3,1,3); hold on; plot(t, rad2deg(X(:,9)), 'Color', c_att);
            
            % Ratei
            set(0, 'CurrentFigure', figs(4));
            subplot(3,1,1); hold on; plot(t, rad2deg(X(:,10)), 'Color', c_rat);
            subplot(3,1,2); hold on; plot(t, rad2deg(X(:,11)), 'Color', c_rat);
            subplot(3,1,3); hold on; plot(t, rad2deg(X(:,12)), 'Color', c_rat);
            
            % Attuatori: Tilt
            set(0, 'CurrentFigure', figs(5));
            for r = 1:2
                subplot(2, 1, r); hold on;
                plot(t, rad2deg(X(:, idx_tilt_vec(r))), 'Color', [rotor_colors(r,:), 0.25], 'LineWidth', 1.0);
            end
            
            % Attuatori: Thrust Singoli
            set(0, 'CurrentFigure', figs(6));
            num_rotors = 2;
            for r = 1:num_rotors
                subplot(num_rotors, 1, r); hold on;
                thrust_r = k_thrust * (X(:, idx_rotors(r)).^2);
                ylim([-5, 110]);
                plot(t, thrust_r, 'Color', [rotor_colors(r,:), 0.25], 'LineWidth', 1.0);
            end
        end
    end
    
    % =====================================================================
    % 2. FORMATTAZIONE GRAFICA (Eseguita una sola volta per subplot)
    % =====================================================================
    % Fig 1: Posizione
    set(0, 'CurrentFigure', figs(1));
    subplot(3,1,1); grid on; ylabel('X [m]'); title([label, ' - Cinematica']);
    subplot(3,1,2); grid on; ylabel('Y [m]');
    subplot(3,1,3); grid on; ylabel('Z [m]'); xlabel('Tempo [s]');
    
    % Fig 2: Velocità
    set(0, 'CurrentFigure', figs(2));
    subplot(3,1,1); grid on; ylabel('V_x [m/s]'); title([label, ' - Velocita']);
    subplot(3,1,2); grid on; ylabel('V_y [m/s]');
    subplot(3,1,3); grid on; ylabel('V_z [m/s]'); xlabel('Tempo [s]');
    
    % Fig 3: Assetto
    set(0, 'CurrentFigure', figs(3));
    subplot(3,1,1); grid on; ylabel('\phi [deg]'); title([label, ' - Assetto']);
    subplot(3,1,2); grid on; ylabel('\theta [deg]');
    subplot(3,1,3); grid on; ylabel('\psi [deg]'); xlabel('Tempo [s]');
    
    % Fig 4: Ratei
    set(0, 'CurrentFigure', figs(4));
    subplot(3,1,1); grid on; ylabel('p [deg/s]'); ylim([-100, 100]); title([label, ' - Ratei']);
    subplot(3,1,2); grid on; ylabel('q [deg/s]'); ylim([-100, 100]); 
    subplot(3,1,3); grid on; ylabel('r [deg/s]'); ylim([-100, 100]); xlabel('Tempo [s]');
    
    % Fig 5: Tilt
    set(0, 'CurrentFigure', figs(5));
    for r = 1:2
        subplot(2, 1, r); grid on;
        ylabel(['\theta_{', num2str(r), '} [deg]']); title(rotor_names_tilt{r});
        yline(90, '--k', 'Alpha', 0.2); yline(0, '--k', 'Alpha', 0.2); ylim([-5, 95]);
        if cfg.test_id == 1
            ylim([40 140]);
        elseif cfg.test_id == 2
            ylim([-30 60]);
        end
        if r == 2; xlabel('Tempo [s]'); end
    end
    linkaxes(findobj(figs(5), 'Type', 'axes'), 'xy');
    
    % Fig 6: Thrust
    set(0, 'CurrentFigure', figs(6));
    for r = 1:num_rotors
        subplot(num_rotors, 1, r); grid on;
        ylabel(['T_{', num2str(r), '} [N]']); title(rotor_names_thrust{r});
        if r == num_rotors; xlabel('Tempo [s]'); end
    end
    linkaxes(findobj(figs(6), 'Type', 'axes'), 'xy');

    % Titoli Generali
    for f = 1:length(figs)
        if isvalid(figs(f))
            figure(figs(f));
            sgtitle([label, ' Analysis Unit'], 'FontWeight', 'bold', 'FontSize', 14);
        end
    end
end