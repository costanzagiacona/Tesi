% Locatelli Andrea - Tesi Magistrale Ing. Automazione
% A.A 2024-2025

clear all
clc

% PER LA SIMULAZIONE : METTERE CONTROLLO A -1
% IN SIMULAZIONEVTOL2 METTERE IL FLAG PER IL SIMBOLICO AD 1

% flag print

paramFlag = 0;

% PER PARAMETRI SIMBOLICI

syms k m g Ixx Iyy Izz rho s I_rotor_xx I_rotor_yy I_rotor_zz C_d C_l C_d_z b ala_y ala_x d_mx d_my d_mz v_air d_tx d_ty d_tz l_w_dx_y l_w_dx_x l_w_dx_z l_w_sx_x l_w_sx_y l_w_sx_z
syms dmx dmy dmz dtx dty dtz b m_rotor

% d_mx = 0;%dmx;
% d_my = dmy;
% d_mz = 0;%dmz;
% 
% d_tx = dtx;
% d_ty = 0;%dty;  % assumo il tail rotor posto sull'asse X_body
% d_tz = 0;%dtz;

d_mx = dmx;
d_my = dmy;
d_mz = dmz;

d_tx = dtx;
d_ty = dty;  % assumo il tail rotor posto sull'asse X_body
d_tz = dtz;

I_rotor = [I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];   

% Struttura con tutti i parametri

% Parametri generali
parametri.m = m;
parametri.g = g;
parametri.k = k;

% Inerzia corpo
parametri.Ixx = Ixx;
parametri.Iyy = Iyy;
parametri.Izz = Izz;
parametri.I_body = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

% Inerzia rotore
parametri.I_rotor_xx = I_rotor_xx;
parametri.I_rotor_yy = I_rotor_yy;
parametri.I_rotor_zz = I_rotor_zz;
%parametri.I_rotor = [I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];
% parametri.I_rotor_w_dx = [I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];
% parametri.I_rotor_w_sx = [I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];
% parametri.I_rotor_tail = [I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];

% Parametri aerodinamici
parametri.rho = rho;
parametri.s = s;
parametri.C_d = C_d;
parametri.C_d_z = C_d_z;
parametri.C_l = C_l;
parametri.b = b;
parametri.v_air = v_air;

% Parametri ala
parametri.ala_x = ala_x;
parametri.ala_y = ala_y;

% Distanze momento (rotori/forze)
parametri.d_mx = d_mx;
parametri.d_my = d_my;
parametri.d_mz = d_mz;

% Distanze traslazione
parametri.d_tx = d_tx;
parametri.d_ty = d_ty;
parametri.d_tz = d_tz;

parametri.I_rotor_w_dx = [I_rotor_xx+m_rotor*d_mx^2 0 0; 0 I_rotor_yy+m_rotor*d_my^2 0; 0 0 I_rotor_zz+m_rotor*d_mz^2];
parametri.I_rotor_w_sx = [I_rotor_xx+m_rotor*d_mx^2 0 0; 0 I_rotor_yy+m_rotor*d_my^2 0; 0 0 I_rotor_zz+m_rotor*d_mz^2];
parametri.I_rotor_tail = [I_rotor_xx+m_rotor*d_tx^2 0 0; 0 I_rotor_yy+m_rotor*d_ty^2 0; 0 0 I_rotor_zz+m_rotor*d_tz^2];

% Distanze ala destra
parametri.l_w_dx_x = l_w_dx_x;
parametri.l_w_dx_y = l_w_dx_y;
parametri.l_w_dx_z = l_w_dx_z;

% Distanze ala sinistra
parametri.l_w_sx_x = l_w_sx_x;
parametri.l_w_sx_y = l_w_sx_y;
parametri.l_w_sx_z =l_w_sx_z;
parametri.b = b;

% parametri.r_th_w_dx = [d_mx ;  d_my ;  d_mz];
% parametri.r_th_w_sx = [d_mx ; -d_my ;  d_mz]; % considero rotore ala dx e sx in posizione simmetrica
% parametri.r_th_tail = [d_tx ;  d_ty ;  d_tz];
% 
% parametri.l_w_dx = [l_w_dx_x; l_w_dx_y; l_w_dx_z];
% parametri.l_w_sx = [l_w_sx_x; l_w_sx_y; l_w_sx_z];


parametri.r_th_w_dx = [0 ;  d_my ;  0];
parametri.r_th_w_sx = [0 ; -d_my ;  0]; % considero rotore ala dx e sx in posizione simmetrica
parametri.r_th_tail = [d_tx ;  0 ;  0];

parametri.l_w_dx = [0; l_w_dx_y; 0];
parametri.l_w_sx = [0; -l_w_dx_y; 0];


parametri.r_aerodyn_w_dx = parametri.l_w_dx;
parametri.r_aerodyn_w_sx = parametri.l_w_sx;

%disp(parametri);

printParametriVTOL(parametri,paramFlag);


%TEST

syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15 x16 x17 x18 x19 x20 x21 x22 x23 x24 x25 x26


% variante in assenza di rotore di coda
x = [x1;x2;x3;x4;x5;x6;x7;x8;x9;x10;x11;x12;x13;x14;x15;x16;0;0;0;0;x21 ;x22 ;x23 ;x24; 0; 0];


x_dot=simulazioneVTOL2(x,parametri); % nel body frame
disp(x_dot);