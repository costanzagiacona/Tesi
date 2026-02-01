function [A, autovalori] = calcola_linearizzazione(x_eq, params, test_id, target)
% CALCOLA_LINEARIZZAZIONE
% Calcola la matrice dinamica A del sistema linearizzato attorno a x_eq.
%
% INPUT:
%   x_eq:    Vettore di stato all'equilibrio (30x1)
%   params:  Struttura parametri del drone
%   test_id: Identificativo del controllore attivo (1 per Hover, 12 per Cruise)
%   target:  Vettore dei target [vx, theta, z, ...]
%
% OUTPUT:
%   A:          Matrice Jacobiana 30x30
%   autovalori: Autovalori del sistema

    % Dimensione dello stato
    n = length(x_eq);
    
    % Perturbazione piccola per il calcolo numerico
    % Deve essere abbastanza piccola da approssimare la derivata, 
    % ma abbastanza grande da evitare errori di arrotondamento macchina.
    delta = 1e-5; 
    
    % Preallocazione
    A = zeros(n, n);
    
    % Poiché simulazioneVTOL3 richiede (t,x,...), fissiamo t=0 
    % (assumiamo il sistema tempo-invariante localmente)
    t0 = 0;
    simbolico = 0; % Vogliamo i valori numerici, non simbolici
    disturbo = 0;  % Nessun disturbo esterno durante la linearizzazione
    
    fprintf('Avvio linearizzazione con test_id = %d...\n', test_id);
    
    % Calcolo delle derivate perturbando ogni stato uno alla volta
    for i = 1:n
        % Perturbazione positiva
        x_plus = x_eq;
        x_plus(i) = x_plus(i) + delta;
        dx_plus = simulazioneVTOL3(t0, x_plus, params, test_id, disturbo, target, simbolico);
        
        % Perturbazione negativa
        x_minus = x_eq;
        x_minus(i) = x_minus(i) - delta;
        dx_minus = simulazioneVTOL3(t0, x_minus, params, test_id, disturbo, target, simbolico);
        
        % Differenza finita centrale
        % A(:, i) è la colonna i-esima della matrice A
        A(:, i) = (dx_plus - dx_minus) / (2 * delta);
    end
    
    % Calcolo autovalori
    autovalori = eig(A);
    
    % Plot rapido degli autovalori nel piano complesso
    figure;
    plot(real(autovalori), imag(autovalori), 'rx', 'LineWidth', 2);
    grid on;
    xline(0, 'k--'); yline(0, 'k--');
    title(['Luogo delle radici (Autovalori) - ID: ' num2str(test_id)]);
    xlabel('Reale (\sigma)'); ylabel('Immaginario (j\omega)');
    
    % Controllo stabilità
    if all(real(autovalori) < 1e-6)
        fprintf('RISULTATO: Il sistema linearizzato è STABILE (o marginalmente stabile).\n');
    else
        fprintf('RISULTATO: Il sistema linearizzato è INSTABILE.\n');
        num_inst = sum(real(autovalori) > 1e-6);
        fprintf('          Trovati %d autovalori instabili.\n', num_inst);
    end
end