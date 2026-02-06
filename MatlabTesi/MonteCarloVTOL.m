function MonteCarloVTOL(params)
% MonteCarloSuite - Validazione Completa (Hovering + Crociera)
%
% Esegue due campagne di test Monte Carlo distinte:
% 1. Fase Hovering (Test ID 1): Partenza da fermo, mantenimento posizione.
% 2. Fase Crociera (Test ID 2): Partenza in velocità, mantenimento rotta.
%
% Output:
% - 2 file .mat con le traiettorie complete
% - 2 file .csv con le statistiche di errore

    % clear all; clc; close all;

    fprintf('================================================\n');
    fprintf('     VTOL MONTE CARLO SUITE                 \n');
    fprintf('================================================\n');

    %% 1. DEFINIZIONE PARAMETRI FISICI (Comuni)
    % % Parametri del drone (copiati dal tuo modello)
    % m = 6.7; g = 9.8; k = 7*(10^-5); rho = 1.225;
    % ala_y = 0.4; ala_x = 0.15; s = ala_x * ala_y;
    % 
    % params.m = m; params.g = g; params.k = k; params.rho = rho;
    % params.s = s; params.ala_x = ala_x; params.ala_y = ala_y;
    % params.v_air = 25; 
    % params.C_d = 0.05; params.C_l = 0.854; 
    % params.C_d_x = 0.1; params.C_d_y = 3; params.C_d_z = 1.0;
    % params.b = 0.005*k;
    % 
    % % Inerzie
    % Ixx = 0.237; Iyy = 0.244; Izz = 0.468;
    % params.I_body = diag([Ixx, Iyy, Izz]);
    % 
    % % Inerzie rotori (semplificate per il calcolo)
    % I_rotor_base = 1e-4 * eye(3);
    % params.I_rotor_w_dx = I_rotor_base; 
    % params.I_rotor_w_sx = I_rotor_base; 
    % params.I_rotor_tail = I_rotor_base;
    % 
    % % Geometria (necessaria per bracci di leva)
    % params.d_mx = 0.6; params.d_my = 0.2; params.d_mz = 0;
    % params.d_tx = -1.2; params.d_ty = 0; params.d_tz = 0;
    % params.r_th_w_dx = [0.6; 0.2; 0]; 
    % params.r_th_w_sx = [0.6; -0.2; 0];
    % params.r_th_tail = [-1.2; 0; 0];
    % params.r_aerodyn_w_dx = [0;0;0]; params.r_aerodyn_w_sx = [0;0;0];

    %% 2. ESECUZIONE CAMPAGNE DI TEST
    
    % --- CONFIGURAZIONE 1: HOVERING ---
    cfg_hover.name = 'Hover';
    cfg_hover.test_id = 1; % Controllo Verticale
    cfg_hover.target = [0, 0, -10]; % Vx=0, Theta=0, Z=-10
    cfg_hover.tspan = [0 100]; % Durata
    cfg_hover.sigma = struct('pos', 2.0, 'vel', 0.5, 'att', deg2rad(5), 'omega', deg2rad(2));
    cfg_hover.x0_type = 'static'; % Partenza quasi ferma
    
    RunCampaign(cfg_hover, params);
    
    fprintf('\n------------------------------------------------\n');
    
    % --- CONFIGURAZIONE 2: CROCIERA ---
    cfg_cruise.name = 'Crociera';
    cfg_cruise.test_id = 2; % Controllo Orizzontale
    cfg_cruise.target = [25, 0, -10]; % Vx=25, Theta=0, Z=-10
    cfg_cruise.tspan = [0 100]; 
    cfg_cruise.sigma = struct('pos', 5.0, 'vel', 2.0, 'att', deg2rad(10), 'omega', deg2rad(5));
    cfg_cruise.x0_type = 'cruise'; % Partenza in velocità
    
    RunCampaign(cfg_cruise, params);

    fprintf('\n================================================\n');
    fprintf(' SUITE COMPLETATA. FILE .MAT E .CSV GENERATI.\n');
    fprintf('================================================\n');
end

function RunCampaign(cfg, params)
    N_sim = 100; % Numero simulazioni per campagna
    fprintf('>> Avvio Campagna: %s (Test ID %d)\n', cfg.name, cfg.test_id);
    
    risultati = struct('t', {}, 'x', {}, 'x0', {}, 'RMSE_z', {}, 'RMSE_y', {}, 'converged', {});
    
    h = waitbar(0, ['Simulazione ' cfg.name '...']);
    
    for i = 1:N_sim
        waitbar(i/N_sim, h);
        
        % Generazione X0 specifica per la fase
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
            
            % Calcolo Errori (Regime: ultimi 50% dei dati)
            idx = t > (cfg.tspan(2)/2);
            if sum(idx)==0, idx=1:length(t); end
            
            z_err = x(idx, 3) - cfg.target(3);
            % Per hover controlliamo la posizione assoluta Y, per crociera la deviazione
            y_err = x(idx, 2) - 0; 
            
            risultati(i).t = t;
            risultati(i).x = x;
            risultati(i).x0 = x0;
            risultati(i).converged = ok;
            risultati(i).RMSE_z = sqrt(mean(z_err.^2));
            risultati(i).RMSE_y = sqrt(mean(y_err.^2));
            
        catch
            risultati(i).converged = false;
        end
    end
    close(h);
    
    % % Salvataggio .MAT
    % fileMat = ['Risultati_' cfg.name '.mat'];
    % save(fileMat, 'risultati', 'cfg');
    % fprintf('   Dati salvati in: %s\n', fileMat);
    % 
    % % Generazione Report .CSV
    % fileCsv = ['Report_' cfg.name '.csv'];
    % GenerateCSV(risultati, fileCsv);
    % fprintf('   Statistiche salvate in: %s\n', fileCsv);
end

function x0 = GenerateX0(type, target, sigma, params)
    x0 = zeros(30,1);
    
    % Perturbazioni comuni
    pos_noise = randn(3,1) * sigma.pos;
    vel_noise = randn(3,1) * sigma.vel;
    att_noise = randn(3,1) * sigma.att;
    omg_noise = randn(3,1) * sigma.omega;
    
    if strcmp(type, 'static')
        % HOVER: Parto da 0 m/s, quota target perturbata
        x0(1:3) = [0; 0; target(3)] + pos_noise; 
        x0(4:6) = [0; 0; 0] + vel_noise;
        
        % Motori: configurazione Hover (Tilt 90 gradi = pi/2)
        % Nota: Nel tuo codice theta tilt è x(13,15,17,19)
        x0(13) = pi/2; x0(15) = pi/2; x0(17) = pi/2; x0(19) = -pi/2;
        
        % Velocità rotori (hovering approssimato)
        T_hover = params.m * params.g;
        omega_hover = sqrt((T_hover/2)/params.k); % Un po' meno perché c'è il rotore di coda
        x0(21) = omega_hover; x0(23) = omega_hover; x0(25) = omega_hover;

    elseif strcmp(type, 'cruise')
        % CROCIERA: Parto da Vx target
        x0(1:3) = [0; 0; target(3)] + pos_noise;
        % Solo la y è perturbata nella posizione, la x parte da 0
        x0(1) = 0; x0(2) = pos_noise(2); x0(3) = target(3) + pos_noise(3);
        
        x0(4:6) = [target(1); 0; 0] + vel_noise;
        
        % Motori: configurazione Aereo (Tilt 0)
        x0(13) = 0; x0(15) = 0; x0(17) = 0; x0(19) = 0;
        
        % Velocità rotori (Cruise thrust stima)
        T_cruise = 15; % Newton a caso per partire
        omega_cruise = sqrt(T_cruise/params.k);
        x0(21) = omega_cruise; x0(23) = omega_cruise; x0(25) = 0; 
    end
    
    % Applico assetto e ratei perturbati
    x0(7:9) = att_noise;
    x0(10:12) = omg_noise;
end

function GenerateCSV(risultati, filename)
    % Crea tabella e salva
    ids = (1:length(risultati))';
    conv = [risultati.converged]';
    rz = [risultati.RMSE_z]';
    ry = [risultati.RMSE_y]';
    
    % Gestione NaN per i non convergeti
    rz(~conv) = NaN; ry(~conv) = NaN;
    
    T = table(ids, conv, rz, ry, 'VariableNames', {'Run', 'Converged', 'RMSE_Z', 'RMSE_Y'});
    writetable(T, filename);
end