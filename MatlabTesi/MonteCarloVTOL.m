function MonteCarloVTOL(params)
% MonteCarloSuite - Validazione Completa (Hovering + Crociera)
    fprintf('================================================\n');
    fprintf('     VTOL MONTE CARLO SUITE                 \n');
    fprintf('================================================\n');
    
    %% 2. ESECUZIONE CAMPAGNE DI TEST
    
    % --- CONFIGURAZIONE 1: HOVERING ---
    cfg_hover.name = 'Hover';
    cfg_hover.test_id = 1; 
    cfg_hover.target = [0, 0, -10]; 
    cfg_hover.tspan = [0 100]; 
    cfg_hover.sigma = struct('pos', 5.0, 'vel', 1.0, 'att', deg2rad(20), 'omega', deg2rad(5));
    cfg_hover.x0_type = 'static';
    
    % RunCampaign(cfg_hover, params);
    
    fprintf('\n------------------------------------------------\n');
    
    % --- CONFIGURAZIONE 2: CROCIERA ---
    cfg_cruise.name = 'Crociera';
    cfg_cruise.test_id = 3; 
    cfg_cruise.target = [25, 0, -10]; 
    cfg_cruise.tspan = [0 100]; 
    cfg_cruise.sigma = struct('pos', 5.0, 'vel', 7.0, 'att', deg2rad(10), 'omega', deg2rad(5));
    cfg_cruise.x0_type = 'cruise'; 
    
    RunCampaign(cfg_cruise, params);
    
    fprintf('\n================================================\n');
    fprintf(' SUITE COMPLETATA. FILE .MAT E .CSV GENERATI.\n');
    fprintf('================================================\n');
end

function RunCampaign(cfg, params)
    N_sim = 100; 
    fprintf('>> Avvio Campagna: %s (Test ID %d)\n', cfg.name, cfg.test_id);
    
    risultati = struct('t', {}, 'x', {}, 'x0', {}, 'RMSE_x', {}, 'RMSE_z', {}, 'RMSE_y', {}, 'converged', {});
    
    h = waitbar(0, ['Simulazione ' cfg.name '...']);
    
    for i = 1:N_sim
        waitbar(i/N_sim, h);
        
        % Generazione X0
        x0 = GenerateX0(cfg.x0_type, cfg.target, cfg.sigma, params);
        
        % Esecuzione ODE
        options = odeset('RelTol',1e-3,'AbsTol',1e-3);
        try
            [t, x] = ode45(@(t,x) simulazioneVTOL3(t, x, params, cfg.test_id, 0, cfg.target, 0), cfg.tspan, x0, options);
            
            % Check convergenza
            if any(isnan(x(:))) || max(abs(x(:,3))) > 500
                ok = false;
            else
                ok = true;
            end
            
            % --- CALCOLO ERRORI ---
            idx = t > (cfg.tspan(2)/2);
            if sum(idx)==0, idx=1:length(t); end
            
            % 1. Errore X (Distinzione Hover vs Cruise)
            if cfg.test_id == 1 % HOVER
                % In Hover voglio stare a X=0. L'errore posizionale ha senso.
                x_err = x(idx, 1) - cfg.target(1);
                vx_err = x(idx, 4) - 0;
            else % CROCIERA
                % In Crociera la X cresce indefinitamente. 
                % L'errore posizionale non ha senso (metrica NaN).
                x_err = NaN; 
                vx_err = x(idx, 4) - cfg.target(1); % Target(1) è 25 m/s
            end
            
            % 2. Altri Errori Posizione (Comuni)
            z_err = x(idx, 3) - cfg.target(3);
            y_err = x(idx, 2) - 0; 
            
            % 3. Errori Ratei Angolari
            p_err = x(idx, 10); 
            q_err = x(idx, 11); 
            r_err = x(idx, 12);
            
            risultati(i).t = t;
            risultati(i).x = x;
            risultati(i).RMSE_vx = sqrt(mean(vx_err.^2));
            risultati(i).x0 = x0;
            risultati(i).converged = ok;
            
            % Salvataggio RMSE
            risultati(i).RMSE_x = sqrt(mean(x_err.^2));
            risultati(i).RMSE_z = sqrt(mean(z_err.^2));
            risultati(i).RMSE_y = sqrt(mean(y_err.^2));
            risultati(i).RMSE_p = sqrt(mean(p_err.^2)); 
            risultati(i).RMSE_q = sqrt(mean(q_err.^2)); 
            risultati(i).RMSE_r = sqrt(mean(r_err.^2)); 
            
        catch
            risultati(i).converged = false;
        end
    end
    close(h);
    
    fileMat = ['Risultati_case3_WF_' cfg.name '.mat'];
    save(fileMat, 'risultati', 'cfg');
    fileCsv = ['Report_case3_noWF_' cfg.name '.csv'];
    GenerateCSV(risultati, fileCsv);
end

function x0 = GenerateX0(type, target, sigma, params)
    x0 = zeros(30,1);
    
    % Perturbazioni comuni (Gaussiano per Omega)
    omg_noise = randn(3,1) * sigma.omega;
    
    if strcmp(type, 'static')
        % --- HOVER ---
        % Posizione Uniforme
        lim_p = sigma.pos;
        pos_noise = -lim_p + (2 * lim_p) * rand(3,1);
        
        x0(1) = target(1) + pos_noise(1);
        x0(2) = target(2) + pos_noise(2);
        x0(3) = target(3) + pos_noise(3);
        if x0(3) > 0, x0(3) = -0.5; end 
        
        % Velocità Uniforme
        lim_v = sigma.vel;
        vel_noise = -lim_v + (2 * lim_v) * rand(3,1);
        x0(4:6) = [0;0;0] + vel_noise;
        
        % Assetto Uniforme
        lim_att = sigma.att; 
        att_noise = -lim_att + (2 * lim_att) * rand(3,1);
        x0(7:9) = att_noise;
        
        % Motori
        x0(13) = pi/2; x0(15) = pi/2; x0(17) = pi/2; x0(19) = -pi/2;
        T_hover = params.m * params.g;
        omega_hover = sqrt((T_hover/2)/params.k);
        x0(21) = omega_hover; x0(23) = omega_hover; x0(25) = omega_hover;
        
    elseif strcmp(type, 'cruise')
        % --- CROCIERA ---
        pos_noise = randn(3,1) * sigma.pos; 
        
        x0(1:3) = [0; 0; target(3)] + pos_noise;
        x0(1) = 0; 
        
        lim_v = sigma.vel; 
        vx_noise = -lim_v + (2 * lim_v) * rand(); 
        
        std_lat = 2.0; 
        vyz_noise = randn(2,1) * std_lat;
        
        x0(4) = target(1) + vx_noise; 
        x0(5) = 0 + vyz_noise(1);     
        x0(6) = 0 + vyz_noise(2);     
        
        % Assetto Uniforme
        lim_att = sigma.att; 
        % att_noise = -lim_att + (2 * lim_att) * rand(3,1);
        att_noise = randn(3,1) * sigma.att;
        % NOTA: Ho rimosso la riga che sovrascriveva att_noise qui!
        
        x0(7) = att_noise(1);                 
        x0(8) = 0 + att_noise(2);             
        x0(9) = att_noise(3);                 
        
        % Motori
        x0(13) = 0; x0(15) = 0; x0(17) = 0; x0(19) = 0;
        T_cruise = 15; 
        omega_cruise = sqrt(T_cruise/params.k);
        x0(21) = omega_cruise; x0(23) = omega_cruise; x0(25) = 0; 
    end
    
    x0(10:12) = omg_noise;
end

function GenerateCSV(risultati, filename)
    % Estrazione ID e Convergenza
    ids = (1:length(risultati))';
    conv = [risultati.converged]';
    
    % --- Estrazione Metriche con controllo esistenza campo ---
    % Serve per evitare errori se usi vecchi file .mat senza il campo vx
    
    % 1. Posizione X (Importante per Hover)
    if isfield(risultati, 'RMSE_x')
        rx = [risultati.RMSE_x]';
    else
        rx = NaN(length(ids), 1);
    end
    
    % 2. Velocità X (Importante per Cruise) <--- NUOVO
    if isfield(risultati, 'RMSE_vx')
        rvx = [risultati.RMSE_vx]';
    else
        rvx = NaN(length(ids), 1);
    end
    
    % 3. Altri assi (Z, Y)
    rz = [risultati.RMSE_z]';
    ry = [risultati.RMSE_y]';
    
    % 4. Ratei (Assetto)
    rp = [risultati.RMSE_p]';
    rq = [risultati.RMSE_q]';
    rr = [risultati.RMSE_r]';
    
    % --- Gestione NaN per i non convergenti ---
    % Pulisce i dati: se la sim è crashata, i valori numerici non hanno senso
    rx(~conv) = NaN;
    rvx(~conv) = NaN;
    rz(~conv) = NaN;
    ry(~conv) = NaN;
    rp(~conv) = NaN; 
    rq(~conv) = NaN; 
    rr(~conv) = NaN;
    
    % --- Creazione Tabella Completa ---
    T = table(ids, conv, rx, rvx, rz, ry, rp, rq, rr, ...
              'VariableNames', {'Run', 'Converged', ...
                                'RMSE_X_Pos', 'RMSE_Vx_Vel', ... % Distinzione Posizione/Velocità
                                'RMSE_Z_Pos', 'RMSE_Y_Pos', ...
                                'RMSE_p_RollRate', 'RMSE_q_PitchRate', 'RMSE_r_YawRate'});
    
    % Scrittura su file
    writetable(T, filename);
    fprintf('   CSV Generato: %s\n', filename);
    fprintf('   -> Colonne aggiunte: X Position (Hover) e Vx Velocity (Cruise)\n');
end