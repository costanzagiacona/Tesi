function MonteCarloVTOL(params)
    clc; close all;
    % =========================================================================
    % VTOL MONTE CARLO SUITE (PROFESSOR EDITION - REVISED)
    % =========================================================================
    fprintf('================================================\n');
    fprintf('     AVVIO SUITE DI VALIDAZIONE ROBUSTA         \n');
    fprintf('================================================\n');
    
    % --- SCENARIO 1: RECOVERY ---
    cfg_rec.name = 'Recovery Hover';
    cfg_rec.test_id = 1; 
    cfg_rec.target = [0; 0; -10]; 
    cfg_rec.tspan = [0 30]; 
    cfg_rec.sigma = struct('pos', 5.0, 'vel', 3, 'att', deg2rad(10), 'omega', deg2rad(5));
    cfg_rec.x0_type = 'static'; 
    cfg_rec.n_states = 26; % Esplicito la dimensione dello stato
    
    % RunCampaign(cfg_rec, params);
    fprintf('\n------------------------------------------------\n');
    
    % --- SCENARIO 2: TAKEOFF ---
    cfg_take.name = 'Takeoff Ground';
    cfg_take.test_id = 1; 
    cfg_take.target = [0; 0; -10]; 
    cfg_take.tspan = [0 15]; 
    cfg_take.sigma = struct('pos', 0.1, 'vel', 0.05, 'att', deg2rad(3), 'omega', deg2rad(10));
    cfg_take.x0_type = 'takeoff'; 
    cfg_take.n_states = 26;
    
    % RunCampaign(cfg_take, params);
    fprintf('\n------------------------------------------------\n');
    
    % --- SCENARIO 3: CRUISE ---
    cfg_cru.name = 'Cruise Flight';
    cfg_cru.test_id = 2; 
    cfg_cru.target = [25; 0; -100]; 
    cfg_cru.tspan = [0 40]; 
    cfg_cru.sigma = struct('pos', 5.0, 'vel', 7.0, 'att', deg2rad(10), 'omega', deg2rad(8)); % Ripristinata incertezza omega
    cfg_cru.x0_type = 'cruise'; 
    cfg_cru.n_states = 28; % Modello con dinamica estesa per il cruise
    
    RunCampaign(cfg_cru, params);
    
    fprintf('\n================================================\n');
    fprintf(' SUITE COMPLETATA.\n');
end

%% --- FUNZIONE CORE: Esecuzione Campagna ---
function RunCampaign(cfg, params)
    N_sim = 200;
    fprintf('>> Scenario: %s (Test ID %d) | %d Simulazioni\n', cfg.name, cfg.test_id, N_sim);
    
    % CORREZIONE: Preallocazione rigorosa
    vuoto_ris = struct('t', [], 'x', [], 'converged', false, 'crashed', false, 'crash_reason', '');
    risultati = repmat(vuoto_ris, N_sim, 1);
    
    vuoto_stats = struct('conv', false, 'crashed', false, 'rmse', NaN(1,12), 'x0', NaN(1,12));
    stats = repmat(vuoto_stats, N_sim, 1);
    
    opts = odeset('Events', @CrashDetector, 'RelTol', 1e-6, 'AbsTol', 1e-6);
    h = waitbar(0, ['Simulazione ' cfg.name '...']);
    
    for i = 1:N_sim
        if ishandle(h), waitbar(i/N_sim, h); end
        
        % Passiamo esplicitamente la dimensione desiderata
        x0_full = GenerateX0(cfg, params);
        stats(i).x0 = x0_full(1:12)'; % Salvataggio coerente in riga
        
        try
            [t, x, te, xe, ie] = ode45(@(t,x) simulazioneVTOL3(t, x, params, cfg.test_id, 0, cfg.target, 0), ...
                                       cfg.tspan, x0_full, opts);
            
            has_crashed = ~isempty(te);
            if has_crashed
                reasons = {'Impatto Suolo', 'Divergenza Quota', 'Eccesso Rollio +', 'Eccesso Rollio -', 'Eccesso Beccheggio +', 'Eccesso Beccheggio -'};
                current_reason = reasons{ie(end)};
            else
                current_reason = 'Nessuno';
            end
            
            if has_crashed
                fprintf('Run %d fallita per: %s al tempo %.2f\n', i, current_reason, te(end));
            end

            ok = (t(end) >= cfg.tspan(2)) && ~has_crashed && ~any(isnan(x(:)));
            
            if t(end) > (cfg.tspan(2) * 0.5)
                idx_settle = t > (t(end) * 0.6); 
                if cfg.test_id == 1 
                    target_vec = [cfg.target(1), cfg.target(2), cfg.target(3), 0, 0, 0, 0, 0, 0, 0, 0, 0];
                else
                    target_vec = [0, 0, cfg.target(3), cfg.target(1), 0, 0, 0, cfg.target(2), 0, 0, 0, 0];
                end
                
                err_data = x(idx_settle, 1:12);
                if cfg.test_id == 2, err_data(:,1) = 0; end 
                
                err_raw = err_data - target_vec;
                for ang_idx = 7:9
                    err_raw(:, ang_idx) = atan2(sin(err_raw(:, ang_idx)), cos(err_raw(:, ang_idx)));
                end
                rmse_vettore = sqrt(mean(err_raw.^2, 1));
            else
                rmse_vettore = NaN(1,12);
            end
            
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
            risultati(i).crash_reason = 'Errore Software';
        end
    end
    
    if ishandle(h), close(h); end
    
    n_crash = sum([stats.crashed]);
    fprintf('   -> Completato. Successi: %d/%d | Crash: %d\n', (N_sim - n_crash), N_sim, n_crash);
    
    save(['Results4_' cfg.name '.mat'], 'risultati', 'cfg');
    GenerateFullCSV(stats, ['Analysis4_' cfg.name '.csv']);
end

%% --- FUNZIONE AUSILIARIA: Generazione Condizioni Iniziali ---
function x0 = GenerateX0(cfg, params)
    % Dimensione di stato definita a priori, non dedotta!
    x0 = zeros(cfg.n_states, 1);
    t_target = cfg.target(:); 
    
    % Equilibrio statico $T = mg \implies 3 k \omega^2 = mg$
    omega_hover = sqrt((params.m * params.g / 3) / params.k);
    
    if strcmp(cfg.x0_type, 'static')
        x0(1:3) = t_target + randn(3,1) * cfg.sigma.pos;
        if x0(3) > -2.0, x0(3) = -2.0; end 
        
        x0(4:6) = randn(3,1) * cfg.sigma.vel;
        x0(7) = randn * cfg.sigma.att;
        x0(8) = randn * cfg.sigma.att;
        x0(9) = randn * cfg.sigma.att;
        
        x0(21) = omega_hover + randn()*10;
        x0(23) = omega_hover + randn()*10;
        x0(25) = omega_hover + randn()*10;
        x0([13, 15, 17, 19]) = pi/2;
        
    elseif strcmp(cfg.x0_type, 'takeoff')
        x0(1:2) = [0;0] + randn(2,1) * cfg.sigma.pos;
        x0(3) = -0.05; 
        
        x0(4:6) = randn(3,1) * cfg.sigma.vel; 
        x0(7:9) = randn(3,1) * cfg.sigma.att; 
        
        omega_idle = 0.20 * omega_hover;
        x0(21) = omega_idle; x0(23) = omega_idle; x0(25) = omega_idle;
        x0([13, 15, 17, 19]) = pi/2;
        
    elseif strcmp(cfg.x0_type, 'cruise')
        x0(1:3) = [0; 0; t_target(3)] + randn(3,1) * cfg.sigma.pos;
        if x0(3) > -2.0, x0(3) = -2.0; end
        
        x0(4) = t_target(1) + randn() * cfg.sigma.vel;
        if x0(4) < 18.0, x0(4) = 18.0; end
        
        x0(5:6) = randn(2,1) * 0.5;
        x0(7:9) = randn(3,1) * cfg.sigma.att;
        x0([13, 15, 17, 19]) = 0; 
        
        % CORREZIONE: Resistenza calcolata coerentemente con la velocità perturbata
        F_drag_body = 0.5 * params.rho * params.s_body_x * params.C_d_x * sign(x0(4)) * x0(4)^2;
        F_drag_ali = params.rho * params.s * params.C_d * sign(x0(4)) * x0(4)^2;
        F0_x = F_drag_body + F_drag_ali;
        
        T_i = F0_x / 2; % Assumendo 2 rotori in trazione
        omega_cruise = sqrt(T_i / params.k);
        x0(21) = omega_cruise;
        x0(23) = omega_cruise;
    end
    
    % CORREZIONE: Applicazione effettiva delle perturbazioni ai ratei angolari
    x0(10:12) = randn(3,1) * cfg.sigma.omega;
end

%% --- FUNZIONE EVENTI: Watchdog Anti-Crash ---
function [value, isterminal, direction] = CrashDetector(~, x)
    ground_impact = x(3) - 0.1; 
    fly_away = x(3) + 200; 
    
    
    % CORREZIONE: Eliminata la discontinuità causata da abs() e max()
    limit_rad = deg2rad(80);
    roll_pos = x(7) - limit_rad;
    roll_neg = -x(7) - limit_rad;
    pitch_pos = x(8) - limit_rad;
    pitch_neg = -x(8) - limit_rad;
    
    value = [ground_impact; fly_away; roll_pos; roll_neg; pitch_pos; pitch_neg];
    isterminal = ones(6,1);
    direction = [1; -1; 1; 1; 1; 1]; % Direzioni corrette per gli attraversamenti
end

%% --- FUNZIONE REPORT: CSV ---
function GenerateFullCSV(stats, filename)
    N = length(stats);
    conv = [stats.conv]';
    crashed = [stats.crashed]';
    
    x0_mat = reshape([stats.x0], 12, N)';
    rmses = reshape([stats.rmse], 12, N)';
    
    T = table((1:N)', conv, crashed, ...
        x0_mat(:,1), x0_mat(:,2), x0_mat(:,3), ...
        x0_mat(:,4), x0_mat(:,5), x0_mat(:,6), ...
        x0_mat(:,7), x0_mat(:,8), x0_mat(:,9), ...
        x0_mat(:,10), x0_mat(:,11), x0_mat(:,12), ...
        rmses(:,1), rmses(:,2), rmses(:,3), ...
        rmses(:,4), rmses(:,5), rmses(:,6), ...
        rmses(:,7), rmses(:,8), rmses(:,9), ...
        rmses(:,10), rmses(:,11), rmses(:,12), ...
        'VariableNames', { ...
            'Run', 'Success', 'Crashed', ...
            'START_X', 'START_Y', 'START_Z', ...
            'START_Vx', 'START_Vy', 'START_Vz', ...
            'START_Phi', 'START_Theta', 'START_Psi', ...
            'START_p', 'START_q', 'START_r', ...
            'RMSE_X', 'RMSE_Y', 'RMSE_Z', ...
            'RMSE_Vx', 'RMSE_Vy', 'RMSE_Vz', ...
            'RMSE_Phi', 'RMSE_Theta', 'RMSE_Psi', ...
            'RMSE_p', 'RMSE_q', 'RMSE_r'
        });
    writetable(T, filename);
    fprintf('   -> Analisi CSV generata: %s\n', filename);
end