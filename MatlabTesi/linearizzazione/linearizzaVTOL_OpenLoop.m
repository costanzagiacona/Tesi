function [A, B] = linearizzaVTOL_OpenLoop(x_eq, u_eq, params)
    % linearizzaVTOL_OpenLoop
    % Calcola le matrici Jacobiane A e B del plant non lineare.
    
    n_stati = length(x_eq);
    m_ingressi = length(u_eq);
    A = zeros(n_stati, n_stati);
    B = zeros(n_stati, m_ingressi);
    
    % Passo di perturbazione infinitesima ottimale per precisione double
    h = 1e-5; 
    t = 0; % Sistema tempo-invariante
    
    % --- Calcolo della Matrice A (Perturbazione degli stati x) ---
    for j = 1:n_stati
        dx = zeros(n_stati, 1);
        dx(j) = h;
        
        f_plus  = simulazioneVTOL_OpenLoop(t, x_eq + dx, u_eq, params);
        f_minus = simulazioneVTOL_OpenLoop(t, x_eq - dx, u_eq, params);
        
        A(:, j) = (f_plus - f_minus) / (2 * h);
    end
    
    % --- Calcolo della Matrice B (Perturbazione degli ingressi u) ---
    for k = 1:m_ingressi
        du = zeros(m_ingressi, 1);
        du(k) = h;
        
        f_plus  = simulazioneVTOL_OpenLoop(t, x_eq, u_eq + du, params);
        f_minus = simulazioneVTOL_OpenLoop(t, x_eq, u_eq - du, params);
        
        B(:, k) = (f_plus - f_minus) / (2 * h);
    end
    
    % Pulizia numerica del round-off
    A(abs(A) < 1e-9) = 0;
    B(abs(B) < 1e-9) = 0;
end