%% PLOT PRESENTATION SUITE - Kinematic, Dynamic & Actuator Analysis
close all; clc;
files = {'Results_SMC_Recovery Hover.mat', ...
         'Results_SMC_Takeoff Ground.mat', ...
         'Results_PID_Cruise Flight.mat'};

files = {'Results_PID_Cruise Flight.mat'};
% files = {'Results_SMC_Takeoff Ground.mat'};

% Impostazioni grafiche per la leggibilità accademica
set(0, 'DefaultAxesFontSize', 15);       
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
        Plot_State_Overview(data.risultati, char(scenarioName));
    end
end

%% ---------------------------------------------------------
%  FUNZIONE UNICA: VISUALIZZAZIONE COMPLETA (STATI + ATTUATORI)
%  ---------------------------------------------------------
function Plot_State_Overview(risultati, label)
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
    
    % Creazione Figure (Posizionate per non sovrapporsi completamente)
    figs(1) = figure('Name', [label, ': Posizione'], 'Position', [10 50 500 700]);
    figs(2) = figure('Name', [label, ': Velocità'], 'Position', [520 50 500 700]);
    figs(3) = figure('Name', [label, ': Assetto'], 'Position', [1030 50 500 700]);
    figs(4) = figure('Name', [label, ': Ratei'], 'Position', [10 50 500 700]); % Sovrapposta a 1
    figs(5) = figure('Name', [label, ': Attuatori (Thrust)'], 'Position', [520 50 500 700]); % Sovrapposta a 2
    figs(6) = figure('Name', [label, ': Attuatori (Tilt)'], 'Position', [520 50 500 700]); % Sovrapposta a 2

    for i = 1:N
        if risultati(i).converged
            t = risultati(i).t;
            X = risultati(i).x; 
            
            % --- FIG 1: POSIZIONE ---
            set(0, 'CurrentFigure', figs(1));
            subplot(3,1,1); hold on; grid on; plot(t, X(:,1), 'Color', c_pos); ylabel('X [m]'); title([label, ' - Cinematica']);
            subplot(3,1,2); hold on; grid on; plot(t, X(:,2), 'Color', c_pos); ylabel('Y [m]');
            subplot(3,1,3); hold on; grid on; plot(t, -X(:,3), 'Color', c_pos); ylabel('Z [m]'); xlabel('Tempo [s]');

            % --- FIG 2: VELOCITÀ ---
            set(0, 'CurrentFigure', figs(2));
            subplot(3,1,1); hold on; grid on; plot(t, X(:,4), 'Color', c_vel); ylabel('V_x [m/s]'); title([label, ' - Velocità']);
            subplot(3,1,2); hold on; grid on; plot(t, X(:,5), 'Color', c_vel); ylabel('V_y [m/s]');
            subplot(3,1,3); hold on; grid on; plot(t, X(:,6), 'Color', c_vel); ylabel('V_z [m/s]'); xlabel('Tempo [s]');

            % --- FIG 3: ASSETTO (Gradi) ---
            set(0, 'CurrentFigure', figs(3));
            subplot(3,1,1); hold on; grid on; plot(t, rad2deg(X(:,7)), 'Color', c_att); ylabel('\phi (Roll) [deg]'); title([label, ' - Assetto']);
            subplot(3,1,2); hold on; grid on; plot(t, rad2deg(X(:,8)), 'Color', c_att); ylabel('\theta (Pitch) [deg]');
            subplot(3,1,3); hold on; grid on; plot(t, rad2deg(X(:,9)), 'Color', c_att); ylabel('\psi (Yaw) [deg]'); xlabel('Tempo [s]');

            % --- FIG 4: RATEI (Gradi/s) ---
            set(0, 'CurrentFigure', figs(4));
            subplot(3,1,1); hold on; grid on; plot(t, rad2deg(X(:,10)), 'Color', c_rat); ylabel('p [deg/s]'); ylim([-100, 100]); title([label, ' - Ratei']);
            subplot(3,1,2); hold on; grid on; plot(t, rad2deg(X(:,11)), 'Color', c_rat); ylabel('q [deg/s]'); ylim([-100, 100]); 
            subplot(3,1,3); hold on; grid on; plot(t, rad2deg(X(:,12)), 'Color', c_rat); ylabel('r [deg/s]'); ylim([-100, 100]);  xlabel('Tempo [s]');

            % --- FIG 5: ATTUATORI (TILT & THRUST) ---
            % --- FIG 6: ANALISI ATTUATORI (TILT SINGOLI ROTORI) ---
            set(0, 'CurrentFigure', figs(5));
            num_rotors = length(idx_rotors);
            
            % Mappatura indici Tilt (Assicurati che idx_tilt sia un vettore di 3 elementi, es: [13, 14, 15])
            % Se hai un solo indice per tutti, usa: idx_tilt_vec = repmat(idx_tilt, 1, 3);
            idx_tilt_vec = [13, 14, 15]; % Esempio: modifica in base al tuo vettore di stato
            
            % Nomi e colori coerenti con la Fig. 5 per facilitare la cross-analisi
            rotor_names = {'Tilt Front-Left', 'Tilt Front-Right', 'Tilt Rear'}; 
            rotor_colors = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880];
            
            for r = 1:num_rotors
                subplot(num_rotors, 1, r); hold on; grid on;
                
                % Estrazione e conversione in gradi
                % X(:, idx_tilt_vec(r)) preleva lo stato specifico per il rotore r
                tilt_deg = rad2deg(X(:, idx_tilt_vec(r)));
                
                % Plot con trasparenza per analisi Monte Carlo
                plot(t, tilt_deg, 'Color', [rotor_colors(r,:), 0.25], 'LineWidth', 1.2);
                
                % Etichettatura tecnica
                ylabel(['\delta_{', num2str(r), '} [deg]']);
                title(rotor_names{r});
                
                % Riferimenti di assetto standard (Hover = 90°, Cruise = 0°)
                yline(90, '--k', 'Alpha', 0.2);
                yline(0, '--k', 'Alpha', 0.2);
                
                % Limiti asse Y per confronto immediato
                ylim([-5, 95]); 
            end
            xlabel('Tempo [s]');
            
            % Sincronizzazione degli assi per evidenziare asincronie temporali
            linkaxes(findobj(figs(5), 'Type', 'axes'), 'xy');

            % --- FIG 6: ANALISI ATTUATORI (SINGOLI ROTORI) ---
            set(0, 'CurrentFigure', figs(6));
            num_rotors = length(idx_rotors);
            
            % Definiamo i nomi dei rotori per le etichette (Personalizzali se sai l'ordine)
            rotor_names = {'Rotore Front-Left', 'Rotore Front-Right', 'Rotore Rear'}; 
            % Palette colori accademica (Blue, Red, Green)
            rotor_colors = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880];

            for r = 1:num_rotors
                subplot(num_rotors, 1, r); hold on; grid on;
                
                % Calcolo della spinta: T = k * omega^2
                thrust_r = k_thrust * (X(:, idx_rotors(r)).^2);
                
                % Plot con trasparenza per visualizzare la dispersione Monte Carlo
                plot(t, thrust_r, 'Color', [rotor_colors(r,:), 0.25], 'LineWidth', 1);
                
                % Etichettatura rigorosa
                ylabel(['T_{', num2str(r), '} [N]']);
                title(rotor_names{r});
                
                % Aggiunta del limite di saturazione se noto (Esempio: 25N)
                % yline(25, '--r', 'Saturazione', 'Alpha', 0.5); 
            end
            xlabel('Tempo [s]');
        end
    end
    
    % Aggiunta riferimenti (Weight) alla Figura 5 solo alla fine del loop
    if isvalid(figs(5))
        set(0, 'CurrentFigure', figs(5));
        subplot(3,1,3); 
        yline(weight, '--k', 'Peso (mg)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    end

    % Pulizia Titoli Figure
    for f = 1:length(figs)
        figure(figs(f));
        sgtitle([label, ' Analysis Unit'], 'FontWeight', 'bold', 'FontSize', 14);
    end
end