function MonteCarloVTOL(params)
    % MonteCarloSuite - Versione Rigorosa per Modello a 30 Stati
    fprintf('================================================\n');
    fprintf('     VTOL MONTE CARLO SUITE (PRO)               \n');
    fprintf('================================================\n');
    
    % --- CONFIGURAZIONE 1: HOVERING ---
    cfg_hover.name = 'Hover';
    cfg_hover.test_id = 1; 
    cfg_hover.target = [0; 0; -10]; % Vettore colonna per evitare errori
    cfg_hover.tspan = [0 50]; 
    cfg_hover.sigma = struct('pos', 3.0, 'vel', 1.0, 'att', deg2rad(10), 'omega', deg2rad(5));
    cfg_hover.x0_type = 'static';
    
    % RunCampaign(cfg_hover, params);
    
    fprintf('\n------------------------------------------------\n');
    
    % --- CONFIGURAZIONE 2: CROCIERA ---
    cfg_cruise.name = 'Crociera';
    cfg_cruise.test_id = 2; 
    cfg_cruise.target = [25; 0; -10]; % [Vx_target; Pitch_target; Z_target]
    cfg_cruise.tspan = [0 80]; 
    cfg_cruise.sigma = struct('pos', 2.0, 'vel', 5.0, 'att', deg2rad(5), 'omega', deg2rad(2));
    cfg_cruise.x0_type = 'cruise'; 
    
    RunCampaign(cfg_cruise, params);
    
    fprintf('\n================================================\n');
    fprintf(' SUITE COMPLETATA.\n');
end


function x0 = GenerateX0(type, target, sigma, params)
    % Inizializzazione Vettore a 30 Stati (come da simulazioneVTOL3.m)
    x0 = zeros(30,1);
    
    % Forza target a colonna per evitare il "Dimension Mismatch"
    t_target = target(:); 

    if strcmp(type, 'static')
        % --- TRIM HOVERING ---
        % Posizione: Target + Rumore (distribuzione normale)
        x0(1:3) = t_target + randn(3,1) * (sigma.pos / 3);
        if x0(3) > -0.5, x0(3) = -0.5; end % Sicurezza suolo (NED)
        
        % Velocità e Assetto
        x0(4:6) = randn(3,1) * (sigma.vel / 3);
        x0(7:9) = randn(3,1) * (sigma.att / 3);
        
        % Motori: Trim per contrastare il peso
        % Dalla tua funzione controlloVTOL_v3:
        % Motori anteriori: 1 e 2. Motore coda: 3.
        T_hover_total = params.m * params.g;
        omega_trim = sqrt((T_hover_total/3) / params.k);
        x0(21) = omega_trim; % Motore 1
        x0(23) = omega_trim; % Motore 2
        x0(25) = omega_trim; % Motore 3
        
        % Tilt: Verticale (90 gradi = pi/2)
        x0([13, 15, 17, 19]) = pi/2;
        
    elseif strcmp(type, 'cruise')
        % --- TRIM CROCIERA ---
        % Posizione: Iniziamo a X=0, quota target
        x0(1:3) = [0; 0; t_target(3)] + randn(3,1) * (sigma.pos / 3);
        
        % Velocità: Iniziamo vicino alla velocità di crociera target
        x0(4) = t_target(1) + randn() * (sigma.vel / 3);
        x0(5:6) = randn(2,1) * 0.5;
        
        % Assetto: Quasi livellato
        x0(7:9) = randn(3,1) * (sigma.att / 3);
        
        % Motori: Tilt orizzontale (0 rad)
        x0([13, 15, 17, 19]) = 0;
        
        % Spinta stimata per vincere il drag a quella velocità
        F_drag_est = 0.5 * params.rho * params.s * params.C_d * t_target(1)^2;
        omega_cruise = sqrt((F_drag_est/2) / params.k);
        x0(21) = omega_cruise;
        x0(23) = omega_cruise;
        x0(25) = 0; % Coda spenta in crociera
    end
    
    % Ratei angolari (comuni)
    x0(10:12) = randn(3,1) * sigma.omega;
    
    % Nota: Gli integrali (27:30) rimangono a 0 (giusto così).
end

function RunCampaign(cfg, params)
    N_sim = 50; 
    fprintf('>> Avvio Campagna: %s (Test ID %d)\n', cfg.name, cfg.test_id);
    
    % Inizializziamo la struct risultati per il salvataggio .mat
    % Deve contenere t, x e converged per essere letta dalla tua funzione di plot
    risultati = struct('t', {}, 'x', {}, 'converged', {}, 'RMSE_u', {}, 'RMSE_Pe', {});
    
    % Inizializziamo la struct per il report statistico Full_Analysis
    stats = struct();
    
    h = waitbar(0, ['Simulazione ' cfg.name '...']);
    
    for i = 1:N_sim
        if ishandle(h), waitbar(i/N_sim, h); end
        
        x0 = GenerateX0(cfg.x0_type, cfg.target, cfg.sigma, params);
        
        try
            [t, x] = ode45(@(t,x) simulazioneVTOL3(t, x, params, cfg.test_id, 0, cfg.target, 0), cfg.tspan, x0);
            
            % Check convergenza
            ok = (t(end) == cfg.tspan(2)) && ~any(isnan(x(:)));
            
            % --- DATI PER IL PLOTTING (.mat) ---
            risultati(i).t = t;
            risultati(i).x = x;
            risultati(i).converged = ok;
            
            % --- DATI PER LE STATISTICHE (Full_Analysis) ---
            stats(i).conv = ok;
            if ok
                idx = t > (cfg.tspan(2)*0.6);
                err = x(idx, 1:12);
                % Target vector per RMSE (Pn, Pe, Pd, u, v, w, phi, theta, psi, p, q, r)
                if cfg.test_id == 1
                    target_vec = [cfg.target(1), 0, cfg.target(3), 0, 0, 0, 0, 0, 0, 0, 0, 0];
                else
                    target_vec = [0, 0, cfg.target(3), cfg.target(1), 0, 0, 0, 0, 0, 0, 0, 0];
                    err(:,1) = 0; % Ignora errore X in crociera
                end
                stats(i).rmse = sqrt(mean((err - target_vec).^2, 1));
            else
                stats(i).rmse = NaN(1,12);
            end
            
        catch
            risultati(i).converged = false;
            stats(i).conv = false;
            stats(i).rmse = NaN(1,12);
        end
    end
    
    if ishandle(h), close(h); end
    
    % --- SALVATAGGIO FILE .MAT PER GRAFICI ---
    % Nota: La tua funzione di plot cerca file come 'Results_Hover_Test.mat'
    fileMat = ['Results_' cfg.name '_Test.mat'];
    save(fileMat, 'risultati', 'cfg');
    fprintf('   -> File .mat salvato: %s\n', fileMat);
    
    % --- SALVATAGGIO CSV ANALISI ---
    GenerateFullCSV(stats, ['Full_Analysis_' cfg.name '.csv']);
end

function GenerateFullCSV(stats, filename)
    N = length(stats);
    conv = [stats.conv]';
    rmses = reshape([stats.rmse], 12, N)';
    
    T = table((1:N)', conv, rmses(:,1), rmses(:,2), rmses(:,3), ...
              rmses(:,4), rmses(:,5), rmses(:,6), ...
              rmses(:,7), rmses(:,8), rmses(:,9), ...
              rmses(:,10), rmses(:,11), rmses(:,12), ...
        'VariableNames', {'Run', 'OK', 'RMSE_Pn', 'RMSE_Pe', 'RMSE_Pd', ...
                          'RMSE_u', 'RMSE_v', 'RMSE_w', ...
                          'RMSE_phi', 'RMSE_theta', 'RMSE_psi', ...
                          'RMSE_p', 'RMSE_q', 'RMSE_r'});
    writetable(T, filename);
    fprintf('   -> Report Completo Generato: %s\n', filename);
end