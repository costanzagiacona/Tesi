% Analisi stabilità sul sistema linearizzato 

% p.to di equilibrio
x4e = 25;
x5e = 0;
x6e = 0;

% parametri
m = 6;
g = 9.8;

% parametri P e PD
Kpx = 5;
Kpz = -3; 
Kdz = -5;

% matrice del sistema linearizzato
A = [0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 -x6e 0 x4e 0 0 0;
     0 0 0 0 0 1 x5e -x4e 0 0 0 0;
     0 0 0 -Kpx/m 0 0 0 -g 0 0 0 0;
     0 0 0 0 0 0 g 0 0 0 0 -x4e;
     0 0 Kpz/m 0 0 Kdz/m 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0];

% disp(A)
% eig(A)

% Sottosistema  (x3, x4, x6)
idx = [3 4 6];
A_ridotto = A(idx, idx);

disp('Matrice A (sottosistema) =');
disp(A_ridotto)

disp('Autovalori del sottosistema:')
disp(eig(A_ridotto))

