function TricopterPlot_v3(t, x)
    %% 1. Definizione Geometria Drone
    ala_x = 0.15/2;    
    ala_yy = 0.45;     
    ala_y = 0.6; 
    ala_z = 0.02;
    coda = 1.2;
    rotor_dmx = 0.4;
    b   = 0.6;   
    a   = b/3;   
    H   = 0.04;  
    H_m = H+H/2; 
    r_p = b/4;   
 
    base = [-a/2  -a/2 a/2; 
            -a/2 a/2 0  ;
              0    0   0 ];  
    to = linspace(0, 2*pi);
    xp = r_p*cos(to);
    yp = r_p*sin(to);
    zp = zeros(1,length(to));

    %% 2. Setup Figura
    figure(6); clf; % Pulisce la figura precedente
    set(gcf, 'Position', [100 100 1200 900], 'Color', 'w');
    hg = gca;
    view(30, 20); % Vista iniziale
    grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m] (Quota)');
    hold(gca, 'on');

    %% 3. Creazione Oggetti Grafici Drone
    % Corpo centrale
    drone(1) = patch(base(1,:), base(2,:), base(3,:), 'r');
    drone(2) = patch(base(1,:), base(2,:), base(3,:)+H, 'r');
    
    % Bracci
    [xc, yc, zc] = cylinder([1 0.5]);
    drone(3) = surface(-coda*zc, yc*ala_x, ala_z*xc+ala_z/2, 'FaceColor', 'b', 'EdgeColor', 'none');
    [xc, yc, zc] = cylinder([1 1]);
    drone(4) = surface(ala_x*yc, 2*ala_y*zc-ala_y, ala_z*xc+ala_z/2, 'FaceColor', 'b', 'EdgeColor', 'none'); 
    
    % Motori
    [xc, yc, zc] = cylinder([H/2 H/2]);
    drone(5) = surface(xc-coda, yc, H_m*zc+H/2, 'FaceColor', 'r'); % Tail motor
    drone(6) = surface(xc+rotor_dmx, yc+ala_yy, H_m*zc+H/2, 'FaceColor', 'r');
    drone(7) = surface(xc+rotor_dmx, yc-ala_yy, H_m*zc+H/2, 'FaceColor', 'r');
    
    % Eliche
    drone(8) = patch(xp-coda, yp, zp+(H_m+H/2), 'c', 'FaceAlpha', 0.3);
    drone(9) = patch(xp+rotor_dmx, yp+ala_yy, zp+(H_m+H/2), 'c', 'FaceAlpha', 0.3);
    drone(10) = patch(xp+rotor_dmx, yp-ala_yy, zp+(H_m+H/2), 'c', 'FaceAlpha', 0.3);
    
    % Dettagli extra
    [xc, yc, zc] = cylinder([0.5 0.5]);
    drone(11) = surface(rotor_dmx+ala_x*yc, 2*ala_yy*zc-ala_yy, ala_z*xc+ala_z/2, 'FaceColor', 'b'); 
    [xc, yc, zc] = cylinder([1 0.5]);
    drone(12) = surface(rotor_dmx*zc, yc*ala_x, ala_z*xc+ala_z/2, 'FaceColor', 'b');

    % Raggruppamento in un hgtransform
    combinedobject = hgtransform('Parent', hg);
    set(drone, 'Parent', combinedobject);
    
    %% 4. Setup Visualizzazione Vento e Assi Body Frame
    
    % Freccia del Vento (Inizialmente invisibile o a zero)
    % Usiamo quiver3: x,y,z, u,v,w. Scala 0 per nasconderla
    wind_arrow = quiver3(0,0,0, 0,0,0, 0, 'Color', 'c', 'LineWidth', 4, 'MaxHeadSize', 0.5);
    wind_text = text(0,0,0, 'WIND', 'Color', 'c', 'FontSize', 12, 'FontWeight', 'bold', 'Visible', 'off');

    % Assi Body Frame (Solidali al drone)
    scale_ax = 0.8;
    % Creiamo 3 linee per gli assi X, Y, Z
    h_ax_x = line([0 0], [0 0], [0 0], 'Color', 'r', 'LineWidth', 2); % X Body
    h_ax_y = line([0 0], [0 0], [0 0], 'Color', 'g', 'LineWidth', 2); % Y Body
    h_ax_z = line([0 0], [0 0], [0 0], 'Color', 'b', 'LineWidth', 2); % Z Body

    % Scia traiettoria
    trail = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 1.5);

    %% 5. Animation Loop
    xx = x(:,1);
    yy = x(:,2);
    zz = -x(:,3); % Conversione NED -> ENU per il plot (Z su = positivo graficamente)
    
    phi = x(:,7);
    theta = x(:,8);
    psi = x(:,9);
    
    % Determiniamo i limiti fissi della finestra rispetto al drone
    view_range = 4; % Metri visibili attorno al drone

    for i = 1:5:length(t)
        current_t = t(i);
        
        % --- Trasformazione Drone ---
        translation = makehgtform('translate', [xx(i) yy(i) zz(i)]);
        % Ordine rotazioni: Z * Y * X (standard aerospaziale per matrici rotazione)
        % Nota: theta negativo per correzione visuale se necessario, o standard
        rotationX = makehgtform('xrotate', phi(i));
        rotationY = makehgtform('yrotate', theta(i)); 
        rotationZ = makehgtform('zrotate', psi(i));
        
        % Matrice rotazione numerica (per calcolare i vettori degli assi)
        R_num = matriceRotazione(phi(i), theta(i), psi(i));
        
        % Applica trasformazione al gruppo grafico
        set(combinedobject, 'Matrix', translation * rotationZ * rotationY * rotationX);
        
        % --- Aggiornamento Scia ---
        set(trail, 'XData', xx(1:i), 'YData', yy(1:i), 'ZData', zz(1:i));

        % --- Aggiornamento Assi Body Frame ---
        % Posizione attuale drone
        pos = [xx(i); yy(i); zz(i)];
        
        % Vettori assi ruotati (convertiti per visualizzazione)
        % Nota: in NED Z è giù. In plot Z è su. Dobbiamo stare attenti.
        % Per semplicità visuale, usiamo la R calcolata sopra.
        vecX = R_num * [scale_ax; 0; 0];
        vecY = R_num * [0; scale_ax; 0];
        vecZ = R_num * [0; 0; scale_ax]; 
        
        % Plot assi (Body X, Body Y, Body Z)
        set(h_ax_x, 'XData', [pos(1) pos(1)+vecX(1)], 'YData', [pos(2) pos(2)+vecX(2)], 'ZData', [pos(3) pos(3)-vecX(3)]);
        set(h_ax_y, 'XData', [pos(1) pos(1)+vecY(1)], 'YData', [pos(2) pos(2)+vecY(2)], 'ZData', [pos(3) pos(3)-vecY(3)]);
        set(h_ax_z, 'XData', [pos(1) pos(1)+vecZ(1)], 'YData', [pos(2) pos(2)+vecZ(2)], 'ZData', [pos(3) pos(3)-vecZ(3)]);

        % --- VISUALIZZAZIONE VENTO (Logica da simulazioneVTOL3) ---
        if current_t > 5
            % Il vento è 30N verso il basso (NED +Z).
            % Nel plot (ENU), "basso" è -Z.
            % Disegniamo la freccia un po' sopra il drone che punta giù
            
            w_start_x = xx(i);
            w_start_y = yy(i);
            w_start_z = zz(i) + 2.5; % 2.5 metri sopra il drone
            
            w_vec_x = 0;
            w_vec_y = 0;
            w_vec_z = -2; % Vettore lungo 2m verso il basso
            
            set(wind_arrow, 'XData', w_start_x, 'YData', w_start_y, 'ZData', w_start_z, ...
                            'UData', w_vec_x, 'VData', w_vec_y, 'WData', w_vec_z, 'Color', 'c');
            
            set(wind_text, 'Position', [w_start_x, w_start_y+0.5, w_start_z], 'Visible', 'on');
        else
            % Nascondi vento
            set(wind_arrow, 'UData', 0, 'VData', 0, 'WData', 0);
            set(wind_text, 'Visible', 'off');
        end

        % --- Camera Follow (Campo visivo dinamico) ---
        xlim([xx(i)-view_range, xx(i)+view_range]);
        ylim([yy(i)-view_range, yy(i)+view_range]);
        % Z camera: seguiamo ma non andiamo sotto terra
        z_min_cam = max(-1, zz(i)-view_range); 
        zlim([z_min_cam, zz(i)+view_range]);
        
        drawnow;
    end
end