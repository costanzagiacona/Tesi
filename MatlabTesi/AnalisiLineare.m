%% 1. STUDIO HOVERING (Punto Fisso)
clear variables; close all; clc;

% Carica parametri (assicurati che mainTest2 o un file di config li carichi)
run('mainTest2.m'); % Eseguiamo il main per caricare la struct 'parametri'
close all; % Chiudiamo i plot del main
%%
% Impostazioni per Hovering
target_hover = [0, 0, -10]; % Vx=0, Theta=0, Z=-10
test_id_hover = 1;          % Controllo verticale

% Simuliamo per abbastanza tempo da stabilizzare il drone
tspan_trim = [0 20]; 
x0_hover = zeros(30,1);
x0_hover(3) = -10; % Partiamo già vicini alla quota
x0_hover(13:20) = pi/2; % Rotori verticali (approssimazione iniziale)

[t_sim, x_sim] = ode45(@(t,x) simulazioneVTOL3(t, x, parametri, test_id_hover, 0, target_hover, 0), tspan_trim, x0_hover);

% ESTRIAMO IL PUNTO DI EQUILIBRIO (L'ultimo stato della simulazione)
x_eq_hover = x_sim(end, :)';

fprintf('\n--- ANALISI HOVERING ---\n');
fprintf('Equilibrio raggiunto a Vx = %.4f m/s, Z = %.4f m\n', x_eq_hover(4), x_eq_hover(3));

% LINEARIZZAZIONE
[A_hover, eig_hover] = calcola_linearizzazione(x_eq_hover, parametri, test_id_hover, target_hover);

% Analisi Modale Rapida (Periodo e Smorzamento dei modi oscillatori)
fprintf('\nModi Oscillatori Dominanti (Hover):\n');
for i=1:length(eig_hover)
    lam = eig_hover(i);
    if imag(lam) > 0.1 % Solo parte positiva complessa
        wn = abs(lam);
        zeta = -real(lam)/wn;
        fprintf('Freq: %.2f rad/s | Smorzamento: %.3f\n', wn, zeta);
    end
end

%% 2. STUDIO CROCIERA (Forward Flight)

% Impostazioni per Crociera
target_cruise = [25, 0, -10]; % Vx=25, Theta=0 (target), Z=-10
test_id_cruise = 2;          % Controllo orizzontale (usiamo il case 12 che è l'ultimo tuning)

% Condizioni iniziali "lanciate" per favorire la convergenza
x0_cruise = zeros(30,1);
x0_cruise(3) = -10;
x0_cruise(4) = 25;   % Partiamo già a 25 m/s
x0_cruise(13:20) = 0; % Rotori orizzontali

% Simulazione lunga per stabilizzare
tspan_trim = [0 40]; 
[t_sim_c, x_sim_c] = ode45(@(t,x) simulazioneVTOL3(t, x, parametri, test_id_cruise, 0, target_cruise, 0), tspan_trim, x0_cruise);

% ESTRIAMO IL PUNTO DI EQUILIBRIO
x_eq_cruise = x_sim_c(end, :)';

fprintf('\n--- ANALISI CROCIERA ---\n');
fprintf('Equilibrio raggiunto a Vx = %.4f m/s, Theta = %.4f deg\n', x_eq_cruise(4), rad2deg(x_eq_cruise(8)));

% LINEARIZZAZIONE
[A_cruise, eig_cruise] = calcola_linearizzazione(x_eq_cruise, parametri, test_id_cruise, target_cruise);

% Analisi
fprintf('\nModi Oscillatori Dominanti (Crociera):\n');
for i=1:length(eig_cruise)
    lam = eig_cruise(i);
    if imag(lam) > 0.1
        wn = abs(lam);
        zeta = -real(lam)/wn;
        fprintf('Freq: %.2f rad/s | Smorzamento: %.3f\n', wn, zeta);
    end
end

% Trova gli indici degli autovalori instabili
idx_unstable = find(real(eig_cruise) > 1e-6);

if ~isempty(idx_unstable)
    fprintf('\n--- DETTAGLIO INSTABILITÀ ---\n');
    [V, D] = eig(A_cruise); % Calcola autovettori (V)
    
    state_names = {'x','y','z','vx','vy','vz','phi','theta','psi',...
                   'p','q','r', 'motori...', 'integratori...'}; 
    % (Nota: completa la lista dei nomi se vuoi, o usa gli indici)
    
    for i = 1:length(idx_unstable)
        k = idx_unstable(i);
        val = eig_cruise(k);
        fprintf('Autovalore Instabile #%d: %.4f + %.4fi\n', i, real(val), imag(val));
        
        % Analisi dei fattori di partecipazione (Magnitudo dell'autovettore)
        eigenvector = abs(V(:, k));
        [max_val, max_idx] = max(eigenvector);
        
        fprintf('  -> Stato dominante: x(%d) (Magnitudo: %.4f)\n', max_idx, max_val);
        
        % Mostra i top 3 stati coinvolti
        [sorted_vals, sorted_idx] = sort(eigenvector, 'descend');
        fprintf('  -> Top 3 stati coinvolti: %d, %d, %d\n', sorted_idx(1), sorted_idx(2), sorted_idx(3));
    end
else
    fprintf('Nessuna instabilità trovata con la soglia 1e-6.\n');
end