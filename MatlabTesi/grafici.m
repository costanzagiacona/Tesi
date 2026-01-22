% Grafici
function grafici()

figure(4)

h1 = plot(time, xp, 'r', time, yp, 'b', time, zp, 'g');
yline(10,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
set(h1, 'LineWidth', 2)
legend('x_{inertial frame}','y_{inertial frame}','z_{inertial frame}', ...
    'FontSize', 14, 'Interpreter','tex', 'Location','best')
ylim([-20 50]); grid on
xlabel('Time [s]', 'FontSize', 14)
ylabel('Posizione [m]', 'FontSize', 14)
title('Andamento stati (x1,...,x12)','FontSize',16)
set(gca, 'FontSize', 14)

end