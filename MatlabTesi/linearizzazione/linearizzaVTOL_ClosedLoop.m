function A_cl = linearizzaVTOL_ClosedLoop(x_eq, params, test_id, target)
    % linearizzaVTOL_ClosedLoop calcola la matrice Jacobiana A_cl (30x30)
    % del sistema VTOL ad anello chiuso usando differenze finite centrali.
    
    n_stati = length(x_eq);
    A_cl = zeros(n_stati, n_stati);
    
    % Passo di perturbazione infinitesima. 
    % Attenzione: se h è troppo piccolo e le pendenze del controllore sono 
    % elevate, potresti incorrere in errori di round-off numerico.
    h = 1e-6; 
    
    % Condizioni fittizie per la valutazione (sistema tempo-invariante)
    t = 0; 
    disturbo = 0;  % Nessun disturbo impulsivo durante la linearizzazione
    simbolico = 0; % Uso numerico
    
    %% Calcolo della Matrice A_cl (Jacobiano rispetto allo stato)
    for j = 1:n_stati
        % Vettore perturbazione
        dx = zeros(n_stati, 1);
        dx(j) = h;
        
        % Perturbazione in avanti (x_eq + h)
        x_plus = x_eq + dx;
        f_plus = simulazioneVTOL3_linearizzabile(t, x_plus, params, test_id, disturbo, target, simbolico);
        
        % Perturbazione all'indietro (x_eq - h)
        x_minus = x_eq - dx;
        f_minus = simulazioneVTOL3_linearizzabile(t, x_minus, params, test_id, disturbo, target, simbolico);
        
        % Differenza centrale per la j-esima colonna di A_cl
        A_cl(:, j) = (f_plus - f_minus) / (2 * h);
    end
    
    % Pulizia numerica: azzera i valori spuri dovuti ad approssimazione
    A_cl(abs(A_cl) < 1e-10) = 0;
end