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
    cfg_rec.name = 'Recovery_{Hover}';
    cfg_rec.test_id = 1; 
    cfg_rec.target = [0; 0; -10]; 
    cfg_rec.tspan = [0 100]; 
    cfg_rec.sigma = struct('pos', 2.0, 'vel', 1.5, 'att', deg2rad(15), 'omega', deg2rad(5));
    cfg_rec.x0_type = 'static'; % Parte già in quota
    
    % RunCampaign(cfg_rec, params);
    
    fprintf('\n------------------------------------------------\n');
    
    % --- SCENARIO 2: TAKEOFF (Decollo da terra con motori al minimo) ---
    cfg_take.name = 'Takeoff_{Ground}';
    cfg_take.test_id = 1; 
    cfg_take.target = [0; 0; -10]; 
    cfg_take.tspan = [0 50]; 
    % Incertezza pos bassa (sappiamo da dove partiamo), incertezza sensori alta
    cfg_take.sigma = struct('pos', 0.1, 'vel', 0.05, 'att', deg2rad(2), 'omega', deg2rad(1));
    cfg_take.x0_type = 'takeoff'; % Parte da terra
    
    % RunCampaign(cfg_take, params);

    fprintf('\n------------------------------------------------\n');
    
    % --- SCENARIO 3: CRUISE (Volo avanzato) ---
    cfg_cru.name = 'Cruise_{Flight}';
    cfg_cru.test_id = 2; 
    cfg_cru.target = [25; 0; -10]; 
    cfg_cru.tspan = [0 60]; 
    cfg_cru.sigma = struct('pos', 2.0, 'vel', 3.0, 'att', deg2rad(5), 'omega', deg2rad(2));
    cfg_cru.x0_type = 'cruise'; 
    
    RunCampaign(cfg_cru, params);
    
    fprintf('\n================================================\n');
    fprintf(' SUITE COMPLETATA CON SUCCESSO.\n');
end

%% --- FUNZIONE CORE: Esecuzione Campagna ---
function RunCampaign(cfg, params)
    N_sim = 100; 
    fprintf('>> Scenario: %s (Test ID %d) | %d Simulazioni\n', cfg.name, cfg.test_id, N_sim);
    
    % Preallocazione strutture dati
    risultati = struct('t', {}, 'x', {}, 'converged', {}, 'crashed', {}, 'crash_reason', {});
    stats = struct(); % Per CSV
    
    % Setup Event Detector per interrompere i crash
    opts = odeset('Events', @CrashDetector, 'RelTol', 1e-3, 'AbsTol', 1e-6);
    
    h = waitbar(0, ['Simulazione ' cfg.name '...']);
    
    for i = 1:N_sim
        if ishandle(h), waitbar(i/N_sim, h); end
        
        % Generazione Condizione Iniziale
        x0 = GenerateX0(cfg.x0_type, cfg.target, cfg.sigma, params);
        
        try
            % Integrazione ODE con Event Detection
            [t, x, te, xe, ie] = ode45(@(t,x) simulazioneVTOL3(t, x, params, cfg.test_id, 0, cfg.target, 0), cfg.tspan, x0, opts);
            
            % Analisi Esito
            has_crashed = ~isempty(te); % Se te non è vuoto, ode45 è stato interrotto
            crash_reason = 'None';
            
            if has_crashed
                % Decodifica motivo del crash
                reasons = {'Impatto Suolo', 'Divergenza Quota', 'Ribaltamento Assetto'};
                if ie(end) <= length(reasons)
                    crash_reason = reasons{ie(end)};
                else
                    crash_reason = 'Unknown';
                end
            end
            
            % Convergenza: Tempo finito raggiunto E nessun crash E niente NaN
            ok = (t(end) >= cfg.tspan(2)) && ~has_crashed && ~any(isnan(x(:)));
            
            % --- SALVATAGGIO DATI ---
            risultati(i).t = t;
            risultati(i).x = x;
            risultati(i).converged = ok;
            risultati(i).crashed = has_crashed;
            risultati(i).crash_reason = crash_reason;
            
            % --- CALCOLO STATISTICHE (Solo se non crashato subito) ---
            stats(i).conv = ok;
            stats(i).crashed = has_crashed;
            
            % Calcoliamo RMSE solo se abbiamo volato almeno il 60% del tempo
            if t(end) > (cfg.tspan(2) * 0.6)
                idx_settle = t > (cfg.tspan(2) * 0.6);
                
                % Definisci vettore target esteso (12 stati)
                if cfg.test_id == 1
                    % Hovering: Tutto a 0 tranne Z
                    target_vec = [cfg.target(1), 0, cfg.target(3), 0, 0, 0, 0, 0, 0, 0, 0, 0];
                else
                    % Cruise: Vx target, Z target, il resto 0
                    target_vec = [0, 0, cfg.target(3), cfg.target(1), 0, 0, 0, 0, 0, 0, 0, 0];
                end
                
                err = x(idx_settle, 1:12);
                % Rimuovi offset X per cruise (non ci interessa la posizione assoluta X)
                if cfg.test_id == 2, err(:,1) = 0; end 
                
                stats(i).rmse = sqrt(mean((err - target_vec).^2, 1));
            else
                stats(i).rmse = NaN(1,12);
            end
            
        catch ME
            fprintf('   [!] Errore numerico Run %d: %s\n', i, ME.message);
            risultati(i).converged = false;
            stats(i).conv = false;
            stats(i).rmse = NaN(1,12);
        end
    end
    
    if ishandle(h), close(h); end
    
    % Report sommario in console
    n_crash = sum([risultati.crashed]);
    fprintf('   -> Completato. Successi: %d/%d | Crash: %d\n', (N_sim - n_crash), N_sim, n_crash);
    
    % Salvataggio
    fileMat = ['Results_' cfg.name '.mat'];
    save(fileMat, 'risultati', 'cfg');
    GenerateFullCSV(stats, ['Analysis_' cfg.name '.csv']);
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
        x0(4) = t_target(1) + randn() * sigma.vel; % Vx
        x0(5:6) = randn(2,1) * 0.5;
        x0(7:9) = randn(3,1) * sigma.att;
        
        % Tilt orizzontale
        x0([13, 15, 17, 19]) = 0; 
        
        % Stima spinta per drag
        F_drag = 0.5 * params.rho * params.s * params.C_d * t_target(1)^2;
        omega_cruise = sqrt((F_drag/2) / params.k);
        x0(21) = omega_cruise;
        x0(23) = omega_cruise;
        x0(25) = 0; % Coda spenta
    end
    
    % Ratei angolari (comuni)
    x0(10:12) = randn(3,1) * sigma.omega;
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
    rmses = reshape([stats.rmse], 12, N)';
    
    T = table((1:N)', conv, crashed, rmses(:,3), rmses(:,4), rmses(:,7), rmses(:,8), ...
        'VariableNames', {'Run', 'Success', 'Crashed', 'RMSE_Alt', 'RMSE_Vx', 'RMSE_Phi', 'RMSE_Theta'});
    
    writetable(T, filename);
    fprintf('   -> CSV Analisi generato: %s\n', filename);
end