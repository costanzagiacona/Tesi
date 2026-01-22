function [w1, w2, theta1, theta2] = risolvi_sistema_rotori(Mx_req, theta_pitch_cmd, Mz_req, Fx_req, Fz_req, params)
    % --- 1. DEFINIZIONE DEI PARAMETRI DEL SISTEMA (Costanti) ---
    k = params.k;      % Costante aerodinamica/spinta (valore esempio)
    dmx = params.d_mx;     % Braccio di momento x (metri - valore esempio)
    dmy = params.d_my;     % Braccio di momento y (metri - valore esempio)


    A = [ 1,    1,     0,     0; ...
          0,    0,    -1,    -1; ...
          0,    0,   -dmy,   dmy; ...
         -dmy, dmy,    0,     0 ];
         
    b = [Fx_req; Fz_req; Mx_req; Mz_req];
    
    x_sol = A \ b; % Soluzione esatta (se A non è singolare)
    
    V1 = x_sol(1); V2 = x_sol(2);
    H1 = x_sol(3); H2 = x_sol(4);
    
    % Ricostruzione comandi locali
    % Motore 1
    T1 = sqrt(V1^2 + H1^2);
    theta1 = atan2(H1, V1); % Angolo del vettore forza
    
    % Motore 2
    T2 = sqrt(V2^2 + H2^2);
    theta2 = atan2(H2, V2);
    
    % --- INTEGRAZIONE PITCH ---
    % Il pitch non è gestito dalle forze differenziali, ma ruotando
    % il vettore "medio" o il corpo. 
    % Nel tuo caso 9, il pitch si controlla sommando un angolo ai servi.
    
    % Saturazioni e conversioni
    w1 = sqrt(max(0, T1)/params.k);
    w2 = sqrt(max(0, T2)/params.k);
    
    % L'angolo finale del servo è l'angolo richiesto dal vettore forza
    % % PIÙ il comando del PID di Pitch
    % theta1 = angle1 + theta_pitch_cmd; 
    % theta2 = angle2 + theta_pitch_cmd;
end