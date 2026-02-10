%% CALCOLO MATRICE A LINEARIZZATA (Open Loop)
clear all; clc;

% 1. Caricamento Parametri
mainTest2; 
close all; % Chiude le figure del main

% 2. Definizione del Punto di Trim (Crociera - Case 3)
% Sostituisci questi valori con quelli esatti trovati dal tuo solver 'fminsearch'
% Esempio (valori ipotetici ma realistici per il tuo drone):
phi_trim = deg2rad(0);
theta_trim = deg2rad(0);   % Pitch per sostenere il naso
psi_trim = deg2rad(0.0305);
omega_trim = 652.95;            % Rad/s motori anteriori (~50% gas)
tilt_trim  = deg2rad(0.0);   % Tilt per vincere il drag
omega_tail = 0;              % Motore coda spento in crociera (o basso)

V_target = 25; 
H_target = -10;

% 3. Costruzione Vettore di Stato Completo (x_trim) - 30 Stati
x_eq = zeros(30, 1);

% Posizione
x_eq(3) = H_target;

% Velocità Body (Coerenti con V_target e theta_trim)
x_eq(4) = V_target * cos(theta_trim); % u
x_eq(6) = V_target * sin(theta_trim); % w

% Assetto
x_eq(7) = phi_trim;
x_eq(8) = theta_trim;
x_eq(9) = psi_trim;

% Attuatori (Servi Tilt) - Assumiamo siano fermi nella posizione di trim
x_eq(13) = tilt_trim; % Tilt 1
x_eq(15) = tilt_trim; % Tilt 2
x_eq(17) = 0;         % Tilt Coda
x_eq(19) = 0;         % Tilt 4

% Attuatori (Motori) - Assumiamo girino a velocità costante
x_eq(21) = omega_trim; % Motore 1
x_eq(23) = omega_trim; % Motore 2
x_eq(25) = omega_tail; % Motore Coda

% Integratori (Li lasciamo a 0 per linearizzazione Open Loop)
x_eq(27:30) = 0;

% 4. Definizione Ingresso di Equilibrio (u_eq)
% Questo serve per chiamare la funzione di simulazione
u_eq = zeros(7,1);
u_eq(1) = omega_trim; 
u_eq(2) = omega_trim;
u_eq(3) = omega_tail;
u_eq(4) = tilt_trim;
u_eq(5) = tilt_trim;
u_eq(6) = 0;
u_eq(7) = 0;

% TRUCCO: Per linearizzare il "Plant" (il drone fisico) e non il controllore,
% dobbiamo bypassare 'controlloVTOL_v3' dentro 'simulazioneVTOL3'.
% Usiamo una variabile globale temporanea per forzare l'input.
global U_FORCE_LIN
U_FORCE_LIN = u_eq; 

% NOTA: Devi modificare 'simulazioneVTOL3.m' aggiungendo queste righe all'inizio:
% global U_FORCE_LIN
% if ~isempty(U_FORCE_LIN)
%     u = U_FORCE_LIN;
% else
%     u = controlloVTOL_v3(...);
% end

% 5. Calcolo Numerico Jacobiano (Differenze Finite Centrali)
n = 30;
epsilon = 1e-5; % Perturbazione piccola
A = zeros(n, n);

fprintf('Calcolo Jacobiano A in corso...\n');

% Calcolo f(x_eq, u_eq) centrale
% Nota: passiamo t=0 e test_id=0 (o qualsiasi, tanto usiamo U_FORCE_LIN)
f0 = simulazioneVTOL3(0, x_eq, parametri, 0, 0, [0 0 0], 0);

for i = 1:n
    % Perturbazione Positiva
    x_plus = x_eq;
    x_plus(i) = x_eq(i) + epsilon;
    f_plus = simulazioneVTOL3(0, x_plus, parametri, 0, 0, [0 0 0], 0);
    
    % Perturbazione Negativa
    x_minus = x_eq;
    x_minus(i) = x_eq(i) - epsilon;
    f_minus = simulazioneVTOL3(0, x_minus, parametri, 0, 0, [0 0 0], 0);
    
    % Derivata Centrale: (f(x+h) - f(x-h)) / 2h -> Più precisa!
    A(:, i) = (f_plus - f_minus) / (2 * epsilon);
    
    if mod(i,5)==0; fprintf('Stato %d completato...\n', i); end
end

% Pulisci variabile globale
clear global U_FORCE_LIN

% 6. Analisi Autovalori
eigs_sys = eig(A);

% Plot Autovalori
figure('Name', 'Mappa Poli Open Loop');
plot(real(eigs_sys), imag(eigs_sys), 'rx', 'LineWidth', 2, 'MarkerSize', 8);
grid on; xline(0, 'k--');
xlabel('Reale (\sigma)'); ylabel('Immaginario (j\omega)');
title(['Autovalori Linearizzati @ ' num2str(V_target) ' m/s']);

% Stampa dei modi instabili
instabili = eigs_sys(real(eigs_sys) > 1e-6);
if ~isempty(instabili)
    fprintf('\nATTENZIONE: Il sistema OPEN LOOP è INSTABILE (Normale per un multicottero)\n');
    disp('Autovalori instabili:');
    disp(instabili);
else
    fprintf('\nIl sistema Open Loop risulta stabile (Incredibile ma possibile in crociera)\n');
end