function PlotMonteCarloStats(csv_filename)
    % =========================================================================
    % ANALISI STATISTICA MONTE CARLO - ADVANCED (PROFESSOR EDITION)
    % =========================================================================
    clc; close all;
    if ~isfile(csv_filename)
        error('File %s non trovato. Eseguire prima la simulazione.', csv_filename);
    end
    
    data = readtable(csv_filename);
    N_total = height(data);
    idx_conv = data.Success == true;
    data_conv = data(idx_conv, :);
    data_crash = data(~idx_conv, :);
    N_conv = height(data_conv);
    
    fprintf('Analisi Statistica: %d simulazioni convergenti su %d totali (Tasso di successo: %.1f%%).\n', ...
        N_conv, N_total, (N_conv/N_total)*100);
    
    if N_conv == 0
        warning('Nessuna simulazione convergente. Analisi interrotta.');
        return;
    end

    % =========================================================================
    % FIGURA 1: Distribuzioni di Probabilità dell'Errore (Istogrammi + PDF)
    % =========================================================================
    % Sostituiamo i boxplot con istogrammi per osservare la reale forma
    % della distribuzione statistica degli errori di posizione.
    
    figure('Name', 'PDF Errore di Posizione', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 400]);
    
    vars = {'RMSE_X', 'RMSE_Y', 'RMSE_Z'};
    titles = {'Errore X', 'Errore Y', 'Errore Z'};
    
    for i = 1:3
        subplot(1, 3, i);
        % Utilizziamo la normalizzazione PDF affinché l'area sottesa sia 1
        h = histogram(data_conv.(vars{i}), 'Normalization', 'pdf', 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'w');
        hold on;
        
        % Sovrapponiamo una stima della densità di probabilità (Kernel Density)
        % Se non possiede lo Statistics Toolbox, commenti le due righe seguenti
        [f, xi] = ksdensity(data_conv.(vars{i}));
        plot(xi, f, 'r-', 'LineWidth', 2);
        
        title(titles{i});
        xlabel('RMSE [m]');
        ylabel('Densità di Probabilità f(x)');
        grid on;
    end

    % =========================================================================
    % FIGURA 2: Matrice di Correlazione (Heatmap)
    % =========================================================================
    % Calcoliamo la correlazione incrociata tra le perturbazioni iniziali
    % (cause) e gli errori a regime (effetti). Questo rivela le vulnerabilità.
    
    figure('Name', 'Matrice di Correlazione Input-Output', 'NumberTitle', 'off', 'Position', [150, 150, 800, 600]);
    
    % Selezioniamo subset di input (Posizione e Assetto iniziali) e output (RMSE)
    input_data = [data_conv.START_X, data_conv.START_Y, data_conv.START_Z, ...
                  data_conv.START_Phi, data_conv.START_Theta, data_conv.START_Psi];
    output_data = [data_conv.RMSE_X, data_conv.RMSE_Y, data_conv.RMSE_Z];
    
    % Matrice completa [Input, Output]
    combined_data = [input_data, output_data];
    R = corrcoef(combined_data);
    
    % Estraiamo solo il blocco di cross-correlazione (Input vs Output)
    R_cross = R(1:6, 7:9); 
    
    x_labels = {'RMSE X', 'RMSE Y', 'RMSE Z'};
    y_labels = {'X_0', 'Y_0', 'Z_0', '\phi_0', '\theta_0', '\psi_0'};
    
    heatmap(x_labels, y_labels, R_cross, 'Colormap', jet, 'ColorLimits', [-1 1]);
    title('Correlazione: Condizione Iniziale vs Errore Finale');

    % =========================================================================
    % FIGURA 3: Bacino di Attrazione Operativo (Scatter 3D)
    % =========================================================================
    % Visualizziamo nello spazio degli stati quali condizioni iniziali
    % portano alla convergenza e quali al crash.
    
    figure('Name', 'Bacino di Attrazione', 'NumberTitle', 'off', 'Position', [200, 200, 800, 600]);
    
    % Esempio: Tracciamo Roll iniziale, Pitch iniziale e Vz iniziale
    start_phi_conv = rad2deg(data_conv.START_Phi);
    start_theta_conv = rad2deg(data_conv.START_Theta);
    start_vz_conv = data_conv.START_Vz;
    
    scatter3(start_phi_conv, start_theta_conv, start_vz_conv, 30, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
    hold on;
    
    if height(data_crash) > 0
        start_phi_crash = rad2deg(data_crash.START_Phi);
        start_theta_crash = rad2deg(data_crash.START_Theta);
        start_vz_crash = data_crash.START_Vz;
        scatter3(start_phi_crash, start_theta_crash, start_vz_crash, 50, 'r', 'x', 'LineWidth', 2);
        legend('Convergenza', 'Crash / Instabilità', 'Location', 'best');
    else
        legend('Convergenza', 'Location', 'best');
    end
    
    title('Bacino di Convergenza Pratico del Controllore');
    xlabel('Rollio Iniziale \phi_0 [deg]');
    ylabel('Beccheggio Iniziale \theta_0 [deg]');
    zlabel('Vel. Verticale Iniziale V_{z,0} [m/s]');
    grid on;
    view(45, 30);
end