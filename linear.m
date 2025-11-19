clc;
clear all;
close all;

%% ========================================================================
%  PARTE 0: INIZIALIZZAZIONE
% =========================================================================
% Pulisce l'ambiente di lavoro, le variabili e chiude le figure.

%% ========================================================================
%  PARTE 1: DEFINIZIONE DEI PARAMETRI DI ANALISI E DEL DRONE
% =========================================================================
% In questa sezione si definiscono i range dei parametri da analizzare e le
% costanti fisiche del drone. È l'unica sezione da modificare per
% eseguire nuove analisi.

fprintf('--- 1. Impostazione dei parametri di analisi ---\n');

% --- Range dei parametri da analizzare ---
% Vettore delle posizioni del baricentro da testare [metri]
% (Negativo = posteriore, Positivo = anteriore, 0 = centro aerodinamico)
delta_cg_x_range = [-0.08, -0.05, -0.02, 0, 0.02];

% Vettore degli angoli di inclinazione motori da testare [gradi]
theta_tilt_range_deg = [10, 15, 20, 25];
theta_tilt_range_rad = deg2rad(theta_tilt_range_deg);

% --- Parametri Numerici Fissi del Drone ---
m_val         = 6;          % Massa totale (kg)
g_val         = 9.8;        % Accelerazione di gravità (m/s^2)
rho_val       = 1.225;      % Densità dell'aria (kg/m^3)
s_val         = 0.4 * 0.15; % Superficie alare (m^2)
b_kappa_val   = 0.01;       % Rapporto coppia/spinta del rotore posteriore
Ixx_val       = 3;          % Momento d'inerzia di rollio (kg*m^2)
Iyy_val       = 0.5 * Ixx_val; % Momento d'inerzia di beccheggio (kg*m^2)
Izz_val       = 1.4 * Ixx_val; % Momento d'inerzia di imbardata (kg*m^2)
x4_target_val = 25;         % Velocità di volo target (m/s)

% Calcolo dei coefficienti aerodinamici basati sui parametri
Cd_val = ((m_val-1)*g_val)/(rho_val*s_val*(x4_target_val)^2);
Cl_val = (m_val*g_val)/(rho_val*s_val*(x4_target_val)^2);


%% ========================================================================
%  PARTE 2: SETUP SIMBOLICO DEL MODELLO DINAMICO
% =========================================================================
% Questa sezione costruisce il modello matematico completo del drone
% usando variabili simboliche. Viene eseguita una sola volta.

fprintf('--- 2. Setup Simbolico del Modello (potrebbe richiedere tempo) ---\n');

% Definizione delle variabili simboliche per parametri e stati
syms m g rho s C_d C_l b_kappa Ixx_s Iyy_s Izz_s real;
syms delta_cg_x_sym theta_tilt_sym real;
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 real;
syms T_ant_sx T_ant_dx T_post delta_servo real;

% Creazione dei vettori e matrici simboliche
I_sym   = diag([Ixx_s, Iyy_s, Izz_s]);
x_sym   = [x1;x2;x3;x4;x5;x6;x7;x8;x9;x10;x11;x12];
u_sym   = [T_ant_sx; T_ant_dx; T_post; delta_servo];
v_body  = [x4; x5; x6];
omega   = [x10; x11; x12];

% Geometria simbolica (dipende dai parametri di analisi)
d_mx_old = 0; d_tx_old = -0.3; % Posizioni fisse dei componenti
l_ant_sym  = d_mx_old - delta_cg_x_sym;
l_post_sym = abs(d_tx_old - delta_cg_x_sym);
dx_cg_sym  = d_mx_old - delta_cg_x_sym;

% Matrici di rotazione (cinematica)
c7=cos(x7);s7=sin(x7);c8=cos(x8);s8=sin(x8);c9=cos(x9);s9=sin(x9);
R_bg = [c9*c8, c9*s8*s7-s9*c7, c9*s8*c7+s9*s7; s9*c8, s9*s8*s7+c9*c7, s9*s8*c7-c9*s7; -s8, c8*s7, c8*c7];
J_bg = [1, s7*tan(x8), c7*tan(x8); 0, c7, -s7; 0, s7/c8, c7/c8];

% Forze e Momenti (dinamica)
F_grav = R_bg'*[0;0;m*g];
F_aero = -0.5*rho*s*[C_d*x4^2; 0; C_l*x4^2];
F_thrust_ant = [sin(theta_tilt_sym);0;-cos(theta_tilt_sym)]*(T_ant_sx+T_ant_dx);
F_thrust_post = [sin(theta_tilt_sym)*cos(delta_servo);sin(theta_tilt_sym)*sin(delta_servo);-cos(theta_tilt_sym)]*T_post;
F_TOT = F_grav + F_aero + F_thrust_ant + F_thrust_post;
M_aero = cross([dx_cg_sym;0;0], F_aero);
M_thrust = [0; (T_ant_sx+T_ant_dx)*l_ant_sym*cos(theta_tilt_sym)-T_post*l_post_sym*cos(theta_tilt_sym); -T_post*l_post_sym*sin(delta_servo)];
M_reaction_yaw = [0;0;b_kappa*T_post];
M_TOT = M_aero + M_thrust + M_reaction_yaw;

% Equazioni di stato (x_dot = f(x,u))
x_dot = sym(zeros(12,1));
x_dot(1:3)  = R_bg * v_body;
x_dot(4:6)  = (1/m)*F_TOT - cross(omega, v_body);
x_dot(7:9)  = J_bg * omega;
x_dot(10:12)= inv(I_sym)*(M_TOT - cross(omega, I_sym*omega));

% Calcolo della Jacobiana Simbolica (la matrice A in forma simbolica)
A_sym = jacobian(x_dot, x_sym);


%% ========================================================================
%  PARTE 3: ANALISI DI STABILITÀ PARAMETRICA (LOOP PRINCIPALE)
% =========================================================================
% Questa è la sezione computazionale. Itera su ogni combinazione dei
% parametri di analisi, calcola la stabilità e salva i risultati.

fprintf('--- 3. Inizio Analisi Parametrica ---\n');

% Inizializzazione della matrice dei risultati
stability_results = zeros(length(delta_cg_x_range), length(theta_tilt_range_rad));

% Ciclo su ogni posizione del baricentro
for i = 1:length(delta_cg_x_range)
    % Ciclo su ogni angolo di inclinazione motori
    for j = 1:length(theta_tilt_range_rad)
        
        % A. Prende i valori correnti per questa iterazione
        delta_cg_x_val = delta_cg_x_range(i);
        theta_tilt_val = theta_tilt_range_rad(j);
        fprintf('Analisi per CG a %.3f m e Tilt a %.1f deg...\n', delta_cg_x_val, rad2deg(theta_tilt_val));
        
        % B. Calcola il punto di equilibrio per questa specifica configurazione
        l_ant_val=d_mx_old-delta_cg_x_val; l_post_val=abs(d_tx_old-delta_cg_x_val); dx_cg_val=d_mx_old-delta_cg_x_val;
        eqs=@(v)[(2*v(2))*sin(theta_tilt_val)+v(3)*cos(v(4))*sin(theta_tilt_val)-m_val*g_val*sin(v(1))-0.5*rho_val*x4_target_val^2*s_val*Cd_val;...
            -(2*v(2))*cos(theta_tilt_val)-v(3)*cos(v(4))*cos(theta_tilt_val)+m_val*g_val*cos(v(1))-0.5*rho_val*x4_target_val^2*s_val*Cl_val;...
            (2*v(2)*l_ant_val-v(3)*l_post_val)*cos(theta_tilt_val)+(m_val*g_val*cos(v(1))-0.5*rho_val*x4_target_val^2*s_val*Cl_val)*dx_cg_val;...
            b_kappa_val*v(3)-(v(3)*sin(v(4)))*l_post_val];
        initial_guess=[0.1,m_val*g_val/3,m_val*g_val/3,0.05]; options=optimoptions('fsolve','Display','off');
        
        try
            % C. Risolve il sistema e costruisce i vettori di equilibrio
            solution=fsolve(eqs,initial_guess,options);
            x8e_val=solution(1); T_ant_e_val=solution(2); T_post_e_val=solution(3); delta_e_val=solution(4);
            x_eq_val=[0;0;0;x4_target_val;0;0;0;x8e_val;0;0;0;0];
            u_eq_val=[T_ant_e_val;T_ant_e_val;T_post_e_val;delta_e_val];

            % D. Sostituisce tutti i valori numerici nella matrice A simbolica
            A_num=subs(A_sym,{m,g,rho,s,C_d,C_l,b_kappa,Ixx_s,Iyy_s,Izz_s,delta_cg_x_sym,theta_tilt_sym},{m_val,g_val,rho_val,s_val,Cd_val,Cl_val,b_kappa_val,Ixx_val,Iyy_val,Izz_val,delta_cg_x_val,theta_tilt_val});
            A_num=subs(A_num,x_sym,x_eq_val);
            A_num=subs(A_num,u_sym,u_eq_val);
            A_num=double(A_num);
            
            % E. Calcola gli autovalori e salva il numero di poli instabili
            poli = eig(A_num);
            num_unstable_poles = sum(real(poli) > 1e-9);
            stability_results(i, j) = num_unstable_poles;
            
            % F. [CASO SPECIALE] Stampa i dettagli per il caso CG = 0
            if delta_cg_x_val == 0
                fprintf('   -> CASO SPECIALE (CG = 0) RILEVATO. Autovalori:\n');
                [~,sort_idx]=sort(real(poli),'descend'); poli_sorted=poli(sort_idx);
                for k=1:length(poli_sorted), fprintf('   %2d: Real=%12.4f, Imag=%+12.4fj\n',k,real(poli_sorted(k)),imag(poli_sorted(k))); end
            end
            
        catch
            % Gestisce i casi in cui non esiste un punto di equilibrio fisico
            fprintf('   -> fsolve non ha trovato una soluzione. Contrassegno come invalido.\n');
            stability_results(i, j) = NaN;
        end
    end
end

%% ========================================================================
%  PARTE 4: VISUALIZZAZIONE DEI RISULTATI
% =========================================================================
% Questa sezione finale presenta i risultati in modo chiaro, sia in forma
% di tabella che di mappa di calore (heatmap).

fprintf('\n--- 4. Risultati Finali dell''Analisi di Stabilità ---\n');

% Stampa della tabella riassuntiva
fprintf('\nNumero di poli instabili per ogni configurazione:\n');
disp(array2table(stability_results,...
    'RowNames', arrayfun(@(x) sprintf('CG=%.2f',x), delta_cg_x_range, 'UniformOutput', false),...
    'VariableNames', arrayfun(@(x) sprintf('Tilt=%.0fdeg',x), theta_tilt_range_deg, 'UniformOutput', false)))

% Creazione della mappa di calore (heatmap) per una visualizzazione intuitiva
figure;
h = heatmap(theta_tilt_range_deg, delta_cg_x_range * 100, stability_results, 'Colormap', flipud(parula));
h.Title = 'Mappa di Stabilità (Numero di Poli Instabili)';
h.XLabel = 'Angolo di Inclinazione Motori (theta_tilt) [gradi]';
h.YLabel = 'Posizione del Baricentro (CG) [cm dal centro ala]';
h.CellLabelFormat = '%.0f';

fprintf('\nAnalisi completata.\n');


%%
% 1. Definizione dello Scenario (L'Obiettivo):
% 
% All'inizio, impostiamo i "casi di studio" che vogliamo analizzare. 
% Definiamo un insieme di possibili posizioni per il baricentro (da molto arretrato a leggermente avanzato) e un insieme di angoli
% di inclinazione per i motori.
% 
% Definiamo anche tutti i parametri fissi del drone: massa, dimensioni, inerzie, coefficienti aerodinamici, ecc.
% 
% 2. Creazione del Modello Matematico "Universale" (Gli Strumenti):
% 
% Usando la matematica simbolica, scriviamo le equazioni complete del moto del drone (forze e momenti).
% 
% Questo crea un modello matematico flessibile che non dipende da nessun valore numerico specifico, 
% ma contiene i parametri (come la posizione del CG) come variabili.
% 
% Il risultato principale di questa fase è una "matrice di stabilità A" generica, 
% che descrive la stabilità del drone in termini puramente simbolici.
% 
% 3. Ciclo di Simulazione "Cosa Succede Se...?" (L'Esperimento):
% 
% Lo script esegue un ciclo automatico che testa ogni singola combinazione possibile dei parametri definiti nella Fase 1 
% (es. CG a -5cm con motori a 15°, poi CG a -5cm con motori a 20°, ecc.).
% 
% Per ogni combinazione, esegue tre passi cruciali:
% 
%   a) Trova l'Equilibrio: Calcola come il drone deve volare per mantenersi stabile a 25 m/s in quella specifica configurazione 
% (trova l'assetto di beccheggio e le spinte dei motori necessarie).
% 
%   b) Linearizza e Analizza: Inserisce i valori numerici di quella configurazione 
% (inclusi i valori di equilibrio appena trovati) nella matrice A "universale" per ottenere una matrice A numerica, specifica per quel caso.
% 
%   c) Controlla la Stabilità: Calcola gli autovalori (poli) di questa matrice A numerica e conta quanti di essi sono instabili 
% (hanno una parte reale positiva).
% 
% 4. Visualizzazione dei Risultati (La Conclusione):
% 
% Alla fine del ciclo, lo script ha raccolto i dati di stabilità per tutte le configurazioni.
% 
% Presenta questi dati in due modi chiari:
% 
% Una tabella che mostra il numero di poli instabili per ogni caso.
% 
% Una mappa di calore (heatmap) che fornisce una visualizzazione immediata delle "zone" di stabilità e instabilità, 
% permettendoci di capire a colpo d'occhio quali scelte di progettazione portano a un drone più robusto e sicuro.