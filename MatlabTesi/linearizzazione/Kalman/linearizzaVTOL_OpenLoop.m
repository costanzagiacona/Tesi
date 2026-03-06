function [A, B] = linearizzaVTOL_OpenLoop(x_eq, u_eq, params)
    n_stati = length(x_eq);
    m_ingressi = length(u_eq);
    A = zeros(n_stati, n_stati);
    B = zeros(n_stati, m_ingressi);
    h = 1e-6; 
    
    % 1. Calcolo della Matrice A (Perturbazione degli stati)
    for j = 1:n_stati
        dx = zeros(n_stati, 1);
        dx(j) = h;
        
        f_plus = simulazioneVTOL_OpenLoop(0, x_eq + dx, u_eq, params);
        f_minus = simulazioneVTOL_OpenLoop(0, x_eq - dx, u_eq, params);
        
        A(:, j) = (f_plus - f_minus) / (2 * h);
    end
    
    % 2. Calcolo della Matrice B (Perturbazione degli ingressi)
    for k = 1:m_ingressi
        du = zeros(m_ingressi, 1);
        du(k) = h;
        
        f_plus = simulazioneVTOL_OpenLoop(0, x_eq, u_eq + du, params);
        f_minus = simulazioneVTOL_OpenLoop(0, x_eq, u_eq - du, params);
        
        B(:, k) = (f_plus - f_minus) / (2 * h);
    end
    
    % Pulizia numerica
    A(abs(A) < 1e-10) = 0;
    B(abs(B) < 1e-10) = 0;
end