%% ANALISI DI STABILITÀ LINEARE VTOL - REPORT FINALE
clc; clear; close all;
addpath 'C:\Users\costa\Documents\GitHub\Tesi\MatlabTesi\'
addpath 'C:\Users\costa\Documents\GitHub\Tesi\MatlabTesi\linearizzazione'
mainTest2; % Carica i parametri e la struct 'parametri'
close all;

% Tolleranza numerica per gli autovalori (per distinguere lo zero reale dal rumore)
tol = 1e-8;

%% =========================================================================
% CASE 1: HOVERING (VOLO VERTICALE) - 26 STATI - OPEN LOOP
% =========================================================================
fprintf('--- ANALISI CASO 1: HOVERING (OPEN LOOP) ---\n');

% 1. Definizione Punto di Equilibrio x_eq
x_eq_hover = zeros(26,1);
x_eq_hover(3) = -10; % Quota desiderata
x_eq_hover(13) = pi/2; % Tilt servi anteriori (Verticale)
x_eq_hover(15) = pi/2;
theta3_ideal = atan2(((-parametri.d_tx * parametri.k) / parametri.b), 1);
x_eq_hover(17) = theta3_ideal; % Tilt coda
x_eq_hover(19) = -pi/2;

% Calcolo spinte di Trim
Thrust_tot_req = parametri.m * parametri.g;
F_tail_z = (parametri.d_mx * Thrust_tot_req) / (parametri.d_mx - parametri.d_tx);
F_front_tot_z = Thrust_tot_req - F_tail_z;

x_eq_hover(21) = sqrt(F_front_tot_z / (2 * parametri.k)); % omega_1
x_eq_hover(23) = x_eq_hover(21);                          % omega_2
x_eq_hover(25) = sqrt(F_tail_z / (parametri.k * sin(theta3_ideal))); % omega_3

% --- NOVITÀ: ESTRAZIONE DELL'INGRESSO DI EQUILIBRIO ---
% 2. Calcolo di u_eq
% Valutiamo il controllore una sola volta nel punto di trim. 
% Questo ci fornisce il vettore u_eq (7x1) costante che mantiene l'equilibrio.
t_eval = 0;
u_eq_hover = controlloVTOL_lineare(t_eval, parametri, x_eq_hover, 1, target);

% 3. Linearizzazione Open-Loop
% Usiamo la nuova funzione che perturba in modo indipendente x e u.
[A_hover, B_hover] = linearizzaVTOL_OpenLoop(x_eq_hover, u_eq_hover, parametri);


% 4. Verifica Stabilità del Plant (Anello Aperto)
autov_hover = eig(A_hover);
re_hover = real(autov_hover);

if any(re_hover > tol)
    status_hover = 'PLANT INSTABILE (Comportamento fisico atteso)';
elseif all(re_hover < -tol)
    status_hover = 'PLANT ASINTOTICAMENTE STABILE';
else
    status_hover = 'PLANT STABILE SEMPLICEMENTE / MARGINALMENTE STABILE';
end

fprintf('Stato del Plant: %s\n', status_hover);
fprintf('Polo più critico (Max Re): %.4f\n\n', max(re_hover));

% --- NOVITÀ: VERIFICA DI RAGGIUNGIBILITÀ (KALMAN) ---
% 5. Matrice di Controllabilità
% R = [B, AB, A^2B, ..., A^(n-1)B]
n_stati = size(A_hover, 1);
R_kalman = ctrb(A_hover, B_hover);
rango_R = rank(R_kalman);

fprintf('--- ANALISI DI CONTROLLABILITÀ DI KALMAN ---\n');
fprintf('Rango della matrice di controllabilità: %d / %d\n', rango_R, n_stati);

if rango_R == n_stati
    fprintf('Il sistema in Hovering è COMPLETAMENTE RAGGIUNGIBILE.\n\n');
else
    fprintf('ATTENZIONE: Il sistema NON è completamente raggiungibile.\n');
    fprintf('Ci sono %d stati (o combinazioni di stati) non governabili dagli ingressi.\n\n', n_stati - rango_R);
end


% --- ANALISI RIGOROSA CON TEST PBH (Popov-Belevitch-Hautus) ---
fprintf('--- TEST PBH DI RAGGIUNGIBILITÀ ---\n');
n_stati = size(A_hover, 1);
I_n = eye(n_stati);
autovalori = eig(A_hover);
tol_pbh = 1e-6; % Tolleranza per il rango SVD

stati_non_raggiungibili = 0;

for i = 1:length(autovalori)
    lambda = autovalori(i);
    % Costruzione della matrice PBH
    Matrice_PBH = [lambda * I_n - A_hover, B_hover];
    
    % Calcolo del rango
    rango_pbh = rank(Matrice_PBH, tol_pbh);
    
    if rango_pbh < n_stati
        stati_non_raggiungibili = stati_non_raggiungibili + 1;
        fprintf('MODO NON RAGGIUNGIBILE RILEVATO!\n');
        fprintf('Autovalore critico: %.4f + %.4fi\n', real(lambda), imag(lambda));
    end
end

if stati_non_raggiungibili == 0
    fprintf('Test PBH SUPERATO: Il sistema e'' COMPLETAMENTE RAGGIUNGIBILE.\n');
    fprintf('La perdita di rango nella matrice di Kalman era un falso negativo numerico.\n\n');
else
    fprintf('Il sistema presenta %d modi fisicamente NON governabili dagli ingressi.\n\n', stati_non_raggiungibili);
end

% Visualizzazione dei valori singolari di Kalman per conferma didattica
S = svd(R_kalman);
figure('Name', 'Valori Singolari Controllabilità');
semilogy(S, 'o-', 'LineWidth', 2);
grid on;
title('Valori Singolari della Matrice di Kalman (Hovering)');
xlabel('Indice del valore singolare');
ylabel('Ampiezza (Scala Logaritmica)');


%% =========================================================================
% CONFRONTO PARTECIPAZIONE: SISTEMA FISICO vs SISTEMA TRASFORMATO (MODALE)
% =========================================================================
fprintf('\n--- CONFRONTO MATRICI DI PARTECIPAZIONE ---\n');

%% --- 1. SISTEMA FISICO ORIGINALE (A_hover) ---
% Estraiamo V (autovettori destri), D (autovalori), e W (autovettori sinistri)
[V_orig, D_orig, W_orig] = eig(A_hover); 
autovalori_orig = diag(D_orig);

% Calcolo rigoroso della matrice di partecipazione senza inv(V)
% Nota: si usa il complesso coniugato di W_orig
P_orig = abs(V_orig .* conj(W_orig)); 
P_norm_orig = (P_orig ./ sum(P_orig, 1)) * 100;

% Raggruppamento Fisico Sensato
P_macro_orig = zeros(5, 26);
P_macro_orig(1, :) = sum(P_norm_orig(1:3, :), 1);   % Posizione
P_macro_orig(2, :) = sum(P_norm_orig(4:6, :), 1);   % Vel. Lineare
P_macro_orig(3, :) = sum(P_norm_orig(7:9, :), 1);   % Assetto
P_macro_orig(4, :) = sum(P_norm_orig(10:12, :), 1); % Vel. Angolare
P_macro_orig(5, :) = sum(P_norm_orig(13:26, :), 1); % Attuatori

% Ordinamento per velocità del modo (Parte Reale decrescente)
[~, sort_idx] = sort(real(autovalori_orig), 'descend');
autov_sorted = autovalori_orig(sort_idx);
P_macro_orig_sorted = P_macro_orig(:, sort_idx);

%% --- 2. SISTEMA TRASFORMATO ---
% NOTA CRITICA: Poiché A_hover è difettiva, la vera trasformazione modale pura
% (D = inv(V)*A*V) non è fisicamente realizzabile. MATLAB farà un'approssimazione 
% con i numeri in virgola mobile, ma per un grafico a scopo didattico possiamo 
% mostrare il sistema pseudo-diagonalizzato o usare la forma di Schur.

% Evitiamo l'inversione esplicita usando l'operatore \ (mldivide) che è ai minimi quadrati
A_mod = V_orig \ A_hover * V_orig; 

[V_mod, D_mod, W_mod] = eig(A_mod);
P_mod = abs(V_mod .* conj(W_mod));
P_norm_mod = (P_mod ./ sum(P_mod, 1)) * 100;
P_mod_sorted = P_norm_mod(:, sort_idx);


% --- 3. RAPPRESENTAZIONE GRAFICA A CONFRONTO ---
figure('Name', 'Confronto Partecipazione: Base Fisica vs Base Modale', 'Position', [100, 100, 1400, 700]);

% ETICHETTE ASSE X
labels_sorted = cell(26, 1);
for i = 1:26
    labels_sorted{i} = sprintf('%.2f', real(autov_sorted(i)));
end

% GRAFICO 1: SISTEMA FISICO
subplot(1, 2, 1);
b1 = bar(P_macro_orig_sorted', 'stacked', 'EdgeColor', 'black', 'LineWidth', 0.5);
b1(1).FaceColor = [0.2 0.6 0.8]; % Posizione
b1(2).FaceColor = [0.4 0.8 0.9]; % Vel. Lineare
b1(3).FaceColor = [0.9 0.4 0.1]; % Assetto
b1(4).FaceColor = [0.9 0.7 0.2]; % Vel. Angolare
b1(5).FaceColor = [0.5 0.5 0.5]; % Attuatori

title('Sistema Originale $X$ (Accoppiato)', 'Interpreter', 'latex', 'FontSize', 15);
xlabel('Autovalori (Reale)', 'FontSize', 12);
ylabel('Partecipazione (%)', 'FontSize', 12);
xticks(1:26); xticklabels(labels_sorted); xtickangle(90);
ylim([0 100]); grid on;
legend({'Posizione', 'Vel. Lineare', 'Assetto', 'Vel. Angolare', 'Attuatori'}, ...
    'Location', 'north', 'Orientation', 'horizontal', 'FontSize', 10);

% GRAFICO 2: SISTEMA TRASFORMATO (MODALE)
subplot(1, 2, 2);
% Uso un colormap di default per mostrare i 26 stati astratti
b2 = bar(P_mod_sorted', 'stacked', 'EdgeColor', 'none'); 
colormap(subplot(1,2,2), lines(26)); 

title('Sistema Trasformato $Z = T X$ (Disaccoppiato)', 'Interpreter', 'latex', 'FontSize', 15);
xlabel('Autovalori (Reale)', 'FontSize', 12);
xticks(1:26); xticklabels(labels_sorted); xtickangle(90);
ylim([0 100]); grid on;

% Aggiungiamo un testo esplicativo nel secondo grafico
text(13, 50, {'Matrice Identita''', '(Ogni stato Z_i partecipa', 'al 100% in un solo modo)'}, ...
    'HorizontalAlignment', 'center', 'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 12);