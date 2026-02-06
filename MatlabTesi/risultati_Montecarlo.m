%% PLOT COMPLETE SUITE - Visualizzazione Totale (Hover + Crociera)
clear all; clc; close all;

% Lista delle fasi da analizzare
files = {'Risultati_Hover.mat', 'Risultati_Crociera.mat'};
offsets = [0, 10]; % Offset per i numeri di figura (Hover: 1-4, Crociera: 11-14)

for k = 1:length(files)
    filename = files{k};
    fig_offset = offsets(k);
    
    if ~isfile(filename)
        fprintf('Attenzione: File %s non trovato. Salto questa fase.\n', filename);
        continue;
    end
    
    % Caricamento dati
    fprintf('Caricamento %s ...\n', filename);
    data = load(filename); 
    risultati = data.risultati;
    cfg = data.cfg;
    
    % Generazione Grafici per questa fase
    GeneratePlots(risultati, cfg, fig_offset);
end

fprintf('Generazione grafici completata.\n');


%% FUNZIONE DI PLOTTING LOCALE
function GeneratePlots(risultati, cfg, fig_offset)
    
    N_sim = length(risultati);
    target = cfg.target; % [Vx, Theta, Z]
    name = cfg.name;
    
    % Colori per spaghetti plot (con trasparenza)
    col_x = [1 0 0 0.15]; % Rosso trasparente
    col_y = [0 0 1 0.15]; % Blu trasparente
    col_z = [0 0.8 0 0.15]; % Verde trasparente
    
    % Parametro Thrust (dal tuo main)
    k_thrust = 7*(10^-5); 

    %% FIGURE 1 (+offset): PANORAMICA STATI (X, V, Angoli, Ratei)
    figure(1 + fig_offset)
    set(gcf, 'Name', [name ' - Stati Completi'], 'Position', [100 100 1200 800]);
    
    % 1. Posizione
    subplot(4,1,1); hold on; grid on;
    title(['[' name '] Traiettorie Posizione'], 'FontSize', 14);
    for i=1:N_sim
        if risultati(i).converged
            t = risultati(i).t; x = risultati(i).x;
            plot(t, x(:,1), 'Color', col_x); % X
            plot(t, x(:,2), 'Color', col_y); % Y
            plot(t, -x(:,3), 'Color', col_z); % Z (quota positiva)
        end
    end
    yline(abs(target(3)), '--g', 'Target Z', 'LineWidth', 1.5);
    ylabel('Pos [m]'); legend('X', 'Y', 'Z', 'Location', 'best');
    
    % 2. Velocità
    subplot(4,1,2); hold on; grid on;
    for i=1:N_sim
        if risultati(i).converged
            t = risultati(i).t; x = risultati(i).x;
            plot(t, x(:,4), 'Color', col_x); % Vx
            plot(t, x(:,5), 'Color', col_y); % Vy
            plot(t, -x(:,6), 'Color', col_z); % Vz
        end
    end
    if cfg.test_id == 2
        yline(target(1), '--r', 'Target Vx');
    end
    ylabel('Vel [m/s]'); legend('Vx', 'Vy', 'Vz');
    
    % 3. Angoli
    subplot(4,1,3); hold on; grid on;
    for i=1:N_sim
        if risultati(i).converged
            t = risultati(i).t; x = risultati(i).x;
            plot(t, rad2deg(x(:,7)), 'Color', col_x); % Phi
            plot(t, rad2deg(x(:,8)), 'Color', col_y); % Theta
            plot(t, rad2deg(x(:,9)), 'Color', col_z); % Psi
        end
    end
    ylabel('Ang [deg]'); legend('Roll', 'Pitch', 'Yaw');
    
    % 4. Ratei
    subplot(4,1,4); hold on; grid on;
    for i=1:N_sim
        if risultati(i).converged
            t = risultati(i).t; x = risultati(i).x;
            plot(t, rad2deg(x(:,10)), 'Color', col_x); 
            plot(t, rad2deg(x(:,11)), 'Color', col_y); 
            plot(t, rad2deg(x(:,12)), 'Color', col_z); 
        end
    end
    ylabel('Rate [deg/s]'); xlabel('Time [s]');


    %% FIGURE 2 (+offset): FOCUS CONTROLLO (Z, Vz, Y)
    figure(2 + fig_offset)
    set(gcf, 'Name', [name ' - Performance Controllo'], 'Position', [150 150 1000 800]);
    
    % 1. Quota Z
    subplot(3,1,1); hold on; grid on;
    title(['[' name '] Tenuta Quota Z'], 'FontSize', 14);
    for i=1:N_sim
        if risultati(i).converged
            plot(risultati(i).t, -risultati(i).x(:,3), 'Color', col_z);
        end
    end
    yline(abs(target(3)), '--k', 'Target', 'LineWidth', 2);
    ylabel('Quota [m]'); ylim([0 20]);
    
    % 2. Velocità Verticale Inerziale (Vz)
    subplot(3,1,2); hold on; grid on;
    ylabel('Vz Inerziale [m/s]');
    % Calcolo Vz globale on-the-fly per correttezza
    for i=1:N_sim
        if risultati(i).converged
            t = risultati(i).t; x = risultati(i).x;
            vz_glob = zeros(length(t),1);
            for k=1:length(t)
               % Rotazione vettore velocità body -> world
               R = matriceRotazione(x(k,7), x(k,8), x(k,9));
               v_body = [x(k,4); x(k,5); x(k,6)];
               v_world = R * v_body;
               vz_glob(k) = v_world(3);
            end
            plot(t, -vz_glob, 'Color', col_y); % -Vz per avere velocità salita positiva
        end
    end
    yline(0, '--k'); 
    
    % 3. Posizione Laterale Y
    subplot(3,1,3); hold on; grid on;
    title(['[' name '] Deriva Laterale Y'], 'FontSize', 14);
    for i=1:N_sim
        if risultati(i).converged
            plot(risultati(i).t, risultati(i).x(:,2), 'Color', col_x);
        end
    end
    yline(0, '--k', 'Centerline', 'LineWidth', 2);
    ylabel('Y [m]'); xlabel('Time [s]');
    

    %% FIGURE 3 (+offset): MOTORI (Thrust)
    figure(3 + fig_offset)
    set(gcf, 'Name', [name ' - Attuatori Motori'], 'Position', [200 200 1000 800]);
    
    subplot(3,1,1); hold on; grid on; title(['[' name '] Spinta Motori (Thrust)'], 'FontSize', 14);
    ylabel('Motore DX (1) [N]');
    for i=1:N_sim
        if risultati(i).converged
            plot(risultati(i).t, k_thrust * risultati(i).x(:,21).^2, 'Color', col_x);
        end
    end
    
    subplot(3,1,2); hold on; grid on; ylabel('Motore SX (2) [N]');
    for i=1:N_sim
        if risultati(i).converged
            plot(risultati(i).t, k_thrust * risultati(i).x(:,23).^2, 'Color', col_y);
        end
    end
    
    subplot(3,1,3); hold on; grid on; ylabel('Motore Coda (3) [N]');
    for i=1:N_sim
        if risultati(i).converged
            plot(risultati(i).t, k_thrust * risultati(i).x(:,25).^2, 'Color', col_z);
        end
    end
    xlabel('Time [s]');


    %% FIGURE 4 (+offset): SERVI (Tilt)
    figure(4 + fig_offset)
    set(gcf, 'Name', [name ' - Attuatori Servi'], 'Position', [250 250 1000 800]);
    
    subplot(2,1,1); hold on; grid on; title(['[' name '] Angoli Tilt Anteriori'], 'FontSize', 14);
    for i=1:N_sim
        if risultati(i).converged
            plot(risultati(i).t, rad2deg(risultati(i).x(:,13)), 'Color', col_x); % Tilt 1
            plot(risultati(i).t, rad2deg(risultati(i).x(:,15)), 'Color', col_y); % Tilt 2
        end
    end
    ylabel('Theta 1,2 [deg]'); legend('DX', 'SX');
    if cfg.test_id == 1, yline(90, '--k'); else, yline(0, '--k'); end
    
    subplot(2,1,2); hold on; grid on; title('Angoli Tilt Coda');
    for i=1:N_sim
        if risultati(i).converged
            plot(risultati(i).t, rad2deg(risultati(i).x(:,17)), 'Color', col_z); % Tilt 3
            plot(risultati(i).t, rad2deg(risultati(i).x(:,19)), 'Color', [0 0 0 0.15]); % Tilt 4
        end
    end
    ylabel('Theta 3,4 [deg]'); legend('Coda Pitch', 'Coda Yaw'); xlabel('Time [s]');

end

% Funzione ausiliaria necessaria per il calcolo Vz Inerziale
function R = matriceRotazione(phi, theta, psi)
    R_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R = R_z * R_y * R_x;
end