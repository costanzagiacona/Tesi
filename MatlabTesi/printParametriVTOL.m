function printParametriVTOL(parametri,paramFlag)

if paramFlag == 1
    % === Stampa dei parametri ===
    fprintf('\n========= PARAMETRI VTOL =========\n');
    fprintf('Massa (m):            %.2f kg\n', parametri.m);
    fprintf('Gravità (g):          %.2f m/s^2\n', parametri.g);
    fprintf('Coefficente thrust (k):          %.2f Ns^2/rad^2\n', parametri.k);
    fprintf('Drag coeff. (asse x) (C_d):    %.2f\n', parametri.C_d);
    fprintf('Drag coeff. (asse z) (C_d_z):    %.2f\n', parametri.C_d_z);
    fprintf('Lift coeff. (C_l):    %.2f\n', parametri.C_l);
    fprintf('Densità aria (rho):   %.2f kg/m^3\n', parametri.rho);
    fprintf('Dimensioni ala (ala_x): %.2f m\n', parametri.ala_x);
    fprintf('Dimensioni ala (ala_y): %.2f m\n', parametri.ala_y);
    fprintf('Superficie alare (s): %.2f m^2\n', parametri.s);
    fprintf('Velocità aria:        %.2f m/s\n', parametri.v_air);
    fprintf('Lambda (torque/thrust): %.2f\n', parametri.lambda);
    fprintf('\n-- Distanze rotori rispetto al centro di massa --\n');
    disp('r_th_w_dx ='); disp(parametri.r_th_w_dx);
    disp('r_th_w_sx ='); disp(parametri.r_th_w_sx);
    disp('r_th_tail ='); disp(parametri.r_th_tail);

    fprintf('-- Distanze forze aerodinamiche --\n');
    disp('r_aerodyn_w_dx ='); disp(parametri.r_aerodyn_w_dx);
    disp('r_aerodyn_w_sx ='); disp(parametri.r_aerodyn_w_sx);
    %disp('r_aerodyn_tail ='); disp(parametri.r_aerodyn_tail);

    fprintf('-- Matrici di inerzia --\n');
    disp('I_body ='); disp(parametri.I_body);
    disp('I_rotor_w_dx ='); disp(parametri.I_rotor_w_dx);
    disp('I_rotor_w_sx ='); disp(parametri.I_rotor_w_sx);
    disp('I_rotor_tail ='); disp(parametri.I_rotor_tail);
    fprintf('===================================\n\n');
end


end