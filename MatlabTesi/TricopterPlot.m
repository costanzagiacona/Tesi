%%% MIGLIORAMENTI:
% 1 - Aggiungere quiver BF
% 2 - Decidere se far plottare il text dei motori
% 3 - Aprire il campo visivo già da subito e centralizzare la telecamera
% sempre sul drone

function TricopterPlot(t,x)

%% Define design parameters
    ala_x = 0.15/2;    
    ala_yy = 0.45;     % dove è il rotore rispetto Y
    ala_y = 0.6; 
    ala_z = 0.02;
    coda = 1.2;
    rotor_dmx = 0.4;
    b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
    a   = b/3;   % the legth of small square base of quadcopter(b/4)
    H   = 0.04;  % hight of drone in Z direction (4cm)
    H_m = H+H/2; % hight of motor in z direction (5 cm)
    r_p = b/4;   % radius of propeller
 
    base = [-a/2  -a/2 a/2; % Coordinates of Base 
            -a/2 a/2 0  ;
              0    0   0 ];  

    to = linspace(0, 2*pi);
    xp = r_p*cos(to);
    yp = r_p*sin(to);
    zp = zeros(1,length(to));

    % Define Figure plot
    %figure('pos', [0 50 800 600]);
    figure(6);
    set(gcf, 'Position', [100 100 1200 900])
    hg   = gca;
    view(-45,30);
    grid on;
    axis equal;
    % xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([0 3.5]);
    % title('(JITENDRA) Drone Animation')
    xlabel('X[m]');
    ylabel('Y[m]');
    zlabel('Z[m]');
    hold(gca, 'on');
    
    % Design Different parts
    % design the base square
    drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
    drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
    alpha(drone(1:2),0.7);
    % design 2 parpendiculer legs of quadcopter 
    [xcylinder, ycylinder, zcylinder] = cylinder([1 0.5]);
    drone(3) = surface(-coda*zcylinder,ycylinder*ala_x,ala_z*xcylinder+ala_z/2,'facecolor','b');
    [xcylinder, ycylinder, zcylinder] = cylinder([1 1]);
    drone(4) = surface(ala_x*ycylinder,2*ala_y*zcylinder-ala_y,ala_z*xcylinder+ala_z/2,'facecolor','b'); 
    alpha(drone(3:4),0.6);
    [xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);
    
    % design 3 cylindrical motors 
    drone(5) = surface(xcylinder-coda,ycylinder,H_m*zcylinder+H/2,'facecolor','r'); %tail
    drone(6) = surface(xcylinder+rotor_dmx,ycylinder+ala_yy,H_m*zcylinder+H/2,'facecolor','r');
    drone(7) = surface(xcylinder+rotor_dmx,ycylinder-ala_yy,H_m*zcylinder+H/2,'facecolor','r');
    alpha(drone(5:7),0.7);

    % design 3 propellers
    drone(8) = patch(xp-coda,yp,zp+(H_m+H/2),'c','LineWidth',0.5);  %tail
    drone(9) = patch(xp+rotor_dmx,yp+ala_yy,zp+(H_m+H/2),'p','LineWidth',0.5);
    drone(10) = patch(xp+rotor_dmx,yp-ala_yy,zp+(H_m+H/2),'p','LineWidth',0.5);
    alpha(drone(8:10),0.3);

    [xcylinder, ycylinder, zcylinder] = cylinder([0.5 0.5]);
    drone(11) = surface(rotor_dmx+ala_x*ycylinder,2*ala_yy*zcylinder-ala_yy,ala_z*xcylinder+ala_z/2,'facecolor','b'); 
    %alpha(drone(3:4),0.6);

    [xcylinder, ycylinder, zcylinder] = cylinder([1 0.5]);
    drone(12) = surface(rotor_dmx*zcylinder,ycylinder*ala_x,ala_z*xcylinder+ala_z/2,'facecolor','b');


    
    %  % Disegna gli assi
    % sdrXQuiver = quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 2);
    % sdrYQuiver = quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 2);
    % sdrZQuiver = quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 2);
    % alpha(drone(13:15),0.3);


    % % Initialize motor texts
    % motor1Text = text(b/2, 0, H_m + H, 'Motor 1', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    % motor2Text = text(-b/2, 0, H_m + H, 'Motor 3', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    % motor3Text = text(0, b/2, H_m + H, 'Motor 4', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    % motor4Text = text(0, -b/2, H_m + H, 'Motor 2', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

    % create a group object and parent surface
    combinedobject = hgtransform('parent',hg);
    set(drone,'parent',combinedobject)

    % Initialize camera view
    % campos([0 0 0]);  % Initial camera position
    

    % plot3(trajDes(1,:), trajDes(2,:), trajDes(3,:));

    hold on;
    % Animation Loop
    %propellerRotationAngle = 2; % Adjust this value to change propeller rotation speed
    
    xx = x(:,1);
    yy = x(:,2);
    zz = -x(:,3); % asse z positivo verso il basso (NED)
    
    phi = (x(:,7));
    theta = (x(:,8));
    psi = (x(:,9));
    
    % TODO: add propeller angles
    t = [0; t];
    trail = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0 0.5 1], 'LineWidth', 3);
    for i = 1:5:length(x)

        %plot3(x(1:i), y(1:i), z(1:i), 'b:', 'LineWidth', 1.5);
       
        translation = makehgtform('translate', [xx(i) yy(i) zz(i)]);
        rotationX = makehgtform('xrotate',  phi(i));
        rotationY = makehgtform('yrotate', -theta(i)); % per usare convenzione NED
        rotationZ = makehgtform('zrotate', psi(i));
        R = translation * rotationZ * rotationY * rotationX;
        set(combinedobject, 'matrix', R);

        % mostro la traiettoria
        % x_trail = [get(trail, 'XData'), xx(i)];
        % y_trail = [get(trail, 'YData'), yy(i)];
        % z_trail = [get(trail, 'ZData'), zz(i)];
        % set(trail, 'XData', x_trail, 'YData', y_trail, 'ZData', z_trail);

       % 
       % % Update motor text positions
       %  motor1Text.Position = translation(1:3, 4)' + [b/2, 0, H_m + H];
       %  motor2Text.Position = translation(1:3, 4)' + [-b/2, 0, H_m + H];
       %  motor3Text.Position = translation(1:3, 4)' + [0, b/2, H_m + H];
       %  motor4Text.Position = translation(1:3, 4)' + [0, -b/2, H_m + H];


        % % Update SDR quiver position and orientation
        % sdrXQuiver.XData = [0, sdrLength] * droneTransform(1,1) + translation(1, 4);
        % sdrXQuiver.YData = [0, sdrLength] * droneTransform(2,1) + translation(2, 4);
        % sdrXQuiver.ZData = [0, sdrLength] * droneTransform(3,1) + translation(3, 4);
        % 
        % sdrYQuiver.XData = [0, sdrWidth] * droneTransform(1,2) + translation(1, 4);
        % sdrYQuiver.YData = [0, sdrWidth] * droneTransform(2,2) + translation(2, 4);
        % sdrYQuiver.ZData = [0, sdrWidth] * droneTransform(3,2) + translation(3, 4);
        % 
        % sdrZQuiver.XData = [0, sdrHeight] * droneTransform(1,3) + translation(1, 4);
        % sdrZQuiver.YData = [0, sdrHeight] * droneTransform(2,3) + translation(2, 4);
        % sdrZQuiver.ZData = [0, sdrHeight] * droneTransform(3,3) + translation(3, 4);
       
        % Update propeller rotation
        % rotate(drone(8), [0 0 1], propellerRotationAngle, [b/2 0 H_m + H/2]);
        % rotate(drone(9), [0 0 1], propellerRotationAngle, [-b/2 0 H_m + H/2]);
        % rotate(drone(10), [0 0 1], propellerRotationAngle, [0 b/2 H_m + H/2]);
        %rotate(drone(12), [0 0 1], propellerRotationAngle, [0 -b/2 H_m + H/2]);

         % Update camera position and target
        %campos(gca, translation(1:3, 4)' + [0 0 0]);  % Adjust the distance from drone
        xlim([-3 3]); ylim([-3 3]); zlim([-0.5 12]);
        drawnow
        %pause(t(i+1)-t(i));
    end
end