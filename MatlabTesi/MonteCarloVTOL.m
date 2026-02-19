function MonteCarloVTOL(params)
    % =========================================================================
    % VTOL MONTE CARLO SUITE (PROFESSOR EDITION)
    % =========================================================================
    % Include:
    % 1. Gestione rigorosa delle condizioni iniziali (Decollo vs Hover).
    % 2. Event Detection: Interruzione immediata in caso di crash o flip.
    % 3. Statistiche separate per scenario.
    % =========================================================================

    fprintf('================================================\n');
    fprintf('     AVVIO SUITE DI VALIDAZIONE ROBUSTA         \n');
    fprintf('================================================\n');
    
    % --- SCENARIO 1: RECOVERY (Stabilizzazione da perturbazione in aria) ---
    cfg_rec.name = 'Recovery Hover';
    cfg_rec.test_id = 1; 
    cfg_rec.target = [0; 0; -10]; 
    cfg_rec.tspan = [0 100]; 
    cfg_rec.sigma = struct('pos', 2.0, 'vel', 1.5, 'att', deg2rad(15), 'omega', deg2rad(5));
    cfg_rec.x0_type = 'static'; % Parte già in quota
    
    % RunCampaign(cfg_rec, params);
    
    fprintf('\n------------------------------------------------\n');
    
    % --- SCENARIO 2: TAKEOFF (Decollo da terra con motori al minimo) ---
    cfg_take.name = 'Takeoff Ground';
    cfg_take.test_id = 1; 
    cfg_take.target = [0; 0; -10]; 
    cfg_take.tspan = [0 50]; 
    % Incertezza pos bassa (sappiamo da dove partiamo), incertezza sensori alta
    cfg_take.sigma = struct('pos', 0.1, 'vel', 0.05, 'att', deg2rad(2), 'omega', deg2rad(1));
    cfg_take.x0_type = 'takeoff'; % Parte da terra
    
    % RunCampaign(cfg_take, params);

    fprintf('\n------------------------------------------------\n');
    
    % --- SCENARIO 3: CRUISE (Volo avanzato) ---
    cfg_cru.name = 'Cruise Flight';
    cfg_cru.test_id = 2; 
    cfg_cru.target = [25; 0; -10]; 
    cfg_cru.tspan = [0 60]; 
    % CORREZIONE: Varianze ridotte per testare perturbazioni realistiche
    cfg_cru.sigma = struct('pos', 5.0, 'vel', 5.0, 'att', deg2rad(10), 'omega', deg2rad(0));
    cfg_cru.x0_type = 'cruise'; 
    
    RunCampaign(cfg_cru, params);
    
    fprintf('\n================================================\n');
    fprintf(' SUITE COMPLETATA.\n');
end

%% --- FUNZIONE CORE: Esecuzione Campagna ---
function RunCampaign(cfg, params)
    N_sim = 100; % Numero di simulazioni
    fprintf('>> Scenario: %s (Test ID %d) | %d Simulazioni\n', cfg.name, cfg.test_id, N_sim);
    
    % Inizializzazione strutture (Aggiunto campo 'x0')
    risultati(N_sim) = struct('t', [], 'x', [], 'converged', [], 'crashed', [], 'crash_reason', '');
    stats(N_sim) = struct('conv', [], 'crashed', [], 'rmse', [], 'x0', []); % <--- NUOVO CAMPO
    
    % Setup Event Detector
    opts = odeset('Events', @CrashDetector, 'RelTol', 1e-6, 'AbsTol', 1e-6);
    
    h = waitbar(0, ['Simulazione ' cfg.name '...']);
    
    for i = 1:N_sim
        if ishandle(h), waitbar(i/N_sim, h); end
        
        % 1. Generazione Condizione Iniziale
        x0_full = GenerateX0(cfg.x0_type, cfg.target, cfg.sigma, params);
        
        % --- REGISTRAZIONE CONDIZIONE INIZIALE (CRITICO) ---
        % Salviamo solo i primi 12 stati (Pos, Vel, Att, Omega)
        % Non ci interessano gli stati interni dei motori o integratori per l'analisi statistica
        stats(i).x0 = x0_full(1:12); 
        
        try
            % 2. Integrazione ODE
            [t, x, te, xe, ie] = ode45(@(t,x) simulazioneVTOL3(t, x, params, cfg.test_id, 0, cfg.target, 0), ...
                                       cfg.tspan, x0_full, opts);
            
            % 3. Analisi Esito
            has_crashed = ~isempty(te);
            if has_crashed
                reasons = {'Impatto Suolo', 'Divergenza Quota', 'Ribaltamento Assetto'};
                if ie(end) <= length(reasons)
                    current_reason = reasons{ie(end)};
                else
                    current_reason = 'Sconosciuto';
                end
            else
                current_reason = 'Nessuno';
            end
            
            % Verifica convergenza
            ok = (t(end) >= cfg.tspan(2)) && ~has_crashed && ~any(isnan(x(:)));
            
            % 4. Calcolo Statistiche RMSE
            if t(end) > (cfg.tspan(2) * 0.5)
                idx_settle = t > (t(end) * 0.6); 
                
                if cfg.test_id == 1 
                    target_vec = [cfg.target(1), cfg.target(2), cfg.target(3), 0, 0, 0, 0, 0, 0, 0, 0, 0];
                elseif cfg.test_id == 2
                    target_vec = [0, 0, cfg.target(3), cfg.target(1), 0, 0, 0, cfg.target(2), 0, 0, 0, 0];
                end
                
                err_data = x(idx_settle, 1:12);
                if cfg.test_id == 2, err_data(:,1) = 0; end 
                
                rmse_vettore = sqrt(mean((err_data - target_vec).^2, 1));
            else
                rmse_vettore = NaN(1,12);
            end
            
            % 5. Archiviazione
            risultati(i).t = t;
            risultati(i).x = x;
            risultati(i).converged = ok;
            risultati(i).crashed = has_crashed;
            risultati(i).crash_reason = current_reason;
            
            stats(i).conv = ok;
            stats(i).crashed = has_crashed;
            stats(i).rmse = rmse_vettore;
            
        catch ME
            fprintf('   [!] Errore numerico Run %d: %s\n', i, ME.message);
            stats(i).conv = false;
            stats(i).crashed = true;
            stats(i).rmse = NaN(1,12);
            stats(i).x0 = x0_full(1:12); % Salviamo x0 anche in caso di errore
            risultati(i).crash_reason = 'Errore Software';
        end
    end
    
    if ishandle(h), close(h); end
    
    n_crash = sum([stats.crashed]);
    fprintf('   -> Completato. Successi: %d/%d | Crash: %d\n', (N_sim - n_crash), N_sim, n_crash);
    
    save(['Results_PID_' cfg.name '.mat'], 'risultati', 'cfg');
    GenerateFullCSV(stats, ['Analysis_PID_' cfg.name '.csv']);
end

%% --- FUNZIONE AUSILIARIA: Generazione Condizioni Iniziali ---
function x0 = GenerateX0(type, target, sigma, params)
    x0 = zeros(30,1);
    t_target = target(:); 
    
    % Calcolo regime di hovering teorico
    % T = m*g -> 3*k*w^2 = m*g
    omega_hover = sqrt((params.m * params.g / 3) / params.k);

    if strcmp(type, 'static')
        % --- RECOVERY (Già in volo) ---
        x0(1:3) = t_target + randn(3,1) * sigma.pos;
        if x0(3) > -2.0, x0(3) = -2.0; end % Sicurezza: non sotto i 2m
        
        x0(4:6) = randn(3,1) * sigma.vel;
        x0(7:9) = randn(3,1) * sigma.att;
        
        % Motori attivi (Trim + rumore)
        x0(21) = omega_hover + randn()*10;
        x0(23) = omega_hover + randn()*10;
        x0(25) = omega_hover + randn()*10;
        x0([13, 15, 17, 19]) = pi/2; % Tilt verticale
        
    elseif strcmp(type, 'takeoff')
        % --- TAKEOFF (Partenza da terra) ---
        % Posizione: Z quasi 0 (NED), XY nota
        x0(1:2) = [0;0] + randn(2,1) * sigma.pos;
        x0(3) = -0.05; % 5 cm dal suolo per evitare singolarità matematiche
        
        x0(4:6) = randn(3,1) * sigma.vel; % Quasi fermo
        x0(7:9) = randn(3,1) * sigma.att; % Quasi piatto
        
        % Motori: IDLE (Minimo Tecnico)
        % Impostiamo il minimo al 20% della velocità di hovering.
        % Questo genera una spinta trascurabile (4% del peso), il drone non decolla da solo.
        omega_idle = 0.20 * omega_hover;
        
        x0(21) = omega_idle;
        x0(23) = omega_idle;
        x0(25) = omega_idle;
        x0([13, 15, 17, 19]) = pi/2; % Tilt verticale

    elseif strcmp(type, 'cruise')
        % --- CRUISE (Volo ala fissa) ---
        x0(1:3) = [0; 0; t_target(3)] + randn(3,1) * sigma.pos;
        
        % 1. PROTEZIONE QUOTA (Z NED è positivo verso il basso)
        % Impediamo al drone di nascere a meno di 2 metri dal suolo o sottoterra
        if x0(3) > -2.0
            x0(3) = -2.0; 
        end
        
        x0(4) = t_target(1) + randn() * sigma.vel; % Vx
        
        % 2. PROTEZIONE STALLO AERODINAMICO
        % Il drone ha bisogno di portanza per mantenere il volo orizzontale.
        % Fissiamo una velocità minima di sicurezza (es. 18 m/s).
        if x0(4) < 18.0
            x0(4) = 18.0; 
        end
        
        x0(5:6) = randn(2,1) * 0.5;
        x0(7:9) = randn(3,1) * sigma.att;
        
        % Tilt orizzontale
        x0([13, 15, 17, 19]) = 0; 
        
        % ... [resto del codice esistente per le spinte iniziali]
        
        % Stima spinta per drag
        F_drag = 0.5 * params.rho * params.s * params.C_d * t_target(1)^2;
        omega_cruise = sqrt((F_drag/2) / params.k);

        F_drag = 0.5*params.rho*params.s_body_x*params.C_d_x*sign(x0(4))*x0(4)^2;
        F_drag_ali = params.rho*params.s*params.C_d*sign(x0(4))*x0(4)^2;
        F0_x = F_drag + F_drag_ali;
        T_i = F0_x/2;
        x0(21)= sqrt(T_i/params.k);
        x0(23)= x0(21);
        % x0(21) = omega_cruise;
        % x0(23) = omega_cruise;
        % x0(25) = 0; % Coda spenta
    end
    
    % Ratei angolari (comuni)
    % x0(10:12) = randn(3,1) * sigma.omega;
end

%% --- FUNZIONE EVENTI: Watchdog Anti-Crash ---
function [value, isterminal, direction] = CrashDetector(~, x)
    % 1. Impatto Suolo: z > 0.1m (NED positivo è giù)
    ground_impact = x(3) - 0.1; 
    
    % 2. Divergenza Quota: z < -200m (volato via troppo in alto)
    fly_away = x(3) + 200; 
    
    % 3. Ribaltamento: Roll o Pitch > 80 gradi
    unsafe_att = max(abs(x(7:8))) - deg2rad(80);
    
    value = [ground_impact; fly_away; unsafe_att];
    isterminal = [1; 1; 1]; % Ferma tutto se succede
    direction = [1; -1; 0]; % Direzione dell'attraversamento
end

%% --- FUNZIONE REPORT: CSV ---
function GenerateFullCSV(stats, filename)
    N = length(stats);
    conv = [stats.conv]';
    crashed = [stats.crashed]';
    
    % --- ESTRAZIONE DATI ---
    
    % 1. Matrice Condizioni Iniziali (N x 12)
    % Trasformiamo l'array di struct in una matrice ordinata
    x0_mat = reshape([stats.x0], 12, N)';
    
    % 2. Matrice RMSE Finali (N x 12)
    rmses = reshape([stats.rmse], 12, N)';
    
    % --- CREAZIONE TABELLA ---
    % La tabella ora contiene: ID, Esito, X0 (12 col), RMSE (12 col)
    
    T = table((1:N)', conv, crashed, ...
        ... % Condizioni Iniziali (Cause)
        x0_mat(:,1), x0_mat(:,2), x0_mat(:,3), ...      % Pos X, Y, Z start
        x0_mat(:,4), x0_mat(:,5), x0_mat(:,6), ...      % Vel u, v, w start
        x0_mat(:,7), x0_mat(:,8), x0_mat(:,9), ...      % Att Phi, Theta, Psi start
        x0_mat(:,10), x0_mat(:,11), x0_mat(:,12), ...   % Rates p, q, r start
        ... % Risultati (Effetti)
        rmses(:,1), rmses(:,2), rmses(:,3), ...
        rmses(:,4), rmses(:,5), rmses(:,6), ...
        rmses(:,7), rmses(:,8), rmses(:,9), ...
        rmses(:,10), rmses(:,11), rmses(:,12), ...
        'VariableNames', { ...
            'Run', 'Success', 'Crashed', ...
            ... % Nomi colonne Input
            'START_X', 'START_Y', 'START_Z', ...
            'START_Vx', 'START_Vy', 'START_Vz', ...
            'START_Phi', 'START_Theta', 'START_Psi', ...
            'START_p', 'START_q', 'START_r', ...
            ... % Nomi colonne Output
            'RMSE_X', 'RMSE_Y', 'RMSE_Z', ...
            'RMSE_Vx', 'RMSE_Vy', 'RMSE_Vz', ...
            'RMSE_Phi', 'RMSE_Theta', 'RMSE_Psi', ...
            'RMSE_p', 'RMSE_q', 'RMSE_r'
        });
    
    writetable(T, filename);
    fprintf('   -> Analisi Completa (Input X0 + Output RMSE) generata: %s\n', filename);
end