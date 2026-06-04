%% Paddle Angle Over Two Cycles - Scotch Yoke Mechanism
% Parameters sourced from Appendix D and design analysis

clear; clc; close all;

% --- Geometry (Appendix D) ---
x     = 10;    % Crank arm length, mm
a     = 21;    % Distance of motor midpoint from fulcrum, mm
alpha = asind(x / a);  % Max paddle angle (deg) — should equal ~28.4 deg

% --- Motor / Cycle Parameters (Appendix D) ---
rpm_max = 234.0;          % Max operating speed, rpm
T       = 60 / rpm_max;  % Period of one cycle, s
omega   = 2*pi / T;      % Motor angular velocity, rad/s

% Ejection phase fraction (paddle in contact with bag)
b = 0.45;

% --- Time vector: two full cycles ---
N = 2000;
t = linspace(0, 2*T, N);

% --- Paddle angle via scotch yoke kinematics ---
% Linear scotch yoke displacement: d(t) = x * sin(omega*t)
% Paddle rotates about fulcrum at distance a:
%   theta(t) = arcsin( x/a * sin(omega*t) )
theta = asind((x / a) .* sin(omega .* t));   % degrees

% --- Angular velocity of paddle (analytical derivative) ---
% d(theta)/dt = (x/a * omega * cos(omega*t)) / sqrt(1 - (x/a*sin(omega*t))^2)
r = x / a;
dtheta_dt = (r .* omega .* cos(omega .* t)) ./ ...
            sqrt(1 - (r .* sin(omega .* t)).^2);   % deg/s
dtheta_dt = rad2deg(dtheta_dt);  % convert rad/s → deg/s

% --- Identify ejection phase windows (bag contact) ---
% Ejection occurs when paddle sweeps from 0 → +alpha (positive half of stroke)
% b fraction of each cycle. Highlight from t=0 to b*T, and T to T+b*T.
t_eject1 = [0,       b*T];
t_eject2 = [T,   T + b*T];

% --- Plot ---
fig = figure('Name', 'Paddle Angle - Two Cycles', 'Color', 'w', ...
             'Position', [100 100 900 550]);

% -- Subplot 1: Paddle angle --
ax1 = subplot(2,1,1);
hold on;

% Shade ejection phases
for region = [t_eject1; t_eject2]
    xregion(region(1)*1000, region(2)*1000, ...
        'FaceColor', [0.85 0.95 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
end

plot(t*1000, theta, 'b-', 'LineWidth', 2);

% Max/min angle reference lines
yline( alpha, 'r--', 'LineWidth', 1.2, 'Label', ...
    sprintf('\\alpha_{max} = %.1f°', alpha), 'LabelHorizontalAlignment', 'left');
yline(-alpha, 'r--', 'LineWidth', 1.2, 'Label', ...
    sprintf('\\alpha_{min} = %.1f°', -alpha), 'LabelHorizontalAlignment', 'left');
yline(0, 'k:', 'LineWidth', 0.8);

% Cycle boundary markers
xline(T*1000,   'k--', 'LineWidth', 1, 'Label', 'Cycle 2', ...
    'LabelVerticalAlignment', 'bottom');
xline(2*T*1000, 'k--', 'LineWidth', 1, 'Label', 'End', ...
    'LabelVerticalAlignment', 'bottom');

hold off;
xlabel('Time (ms)');
ylabel('Paddle Angle (°)');
title('Paddle Angle vs. Time — Two Cycles');
legend('Ejection phase', 'Paddle angle', 'Location', 'northeast');
grid on; xlim([0 2*T*1000]); ylim([-alpha*1.3, alpha*1.3]);
ax1.FontSize = 11;

% -- Subplot 2: Angular velocity --
ax2 = subplot(2,1,2);
hold on;

for region = [t_eject1; t_eject2]
    xregion(region(1)*1000, region(2)*1000, ...
        'FaceColor', [0.85 0.95 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
end

plot(t*1000, dtheta_dt, 'm-', 'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 0.8);
xline(T*1000,   'k--', 'LineWidth', 1);
xline(2*T*1000, 'k--', 'LineWidth', 1);

hold off;
xlabel('Time (ms)');
ylabel('d\theta/dt (°/s)');
title('Paddle Angular Velocity vs. Time — Two Cycles');
grid on; xlim([0 2*T*1000]);
ax2.FontSize = 11;

% -- Shared annotation box --
annotation('textbox', [0.72 0.48 0.20 0.12], ...
    'String', { ...
        sprintf('x = %g mm  |  a = %g mm', x, a), ...
        sprintf('\\alpha_{max} = %.1f°', alpha), ...
        sprintf('rpm = %.1f  |  T = %.1f ms', rpm_max, T*1000), ...
        sprintf('Ejection fraction b = %.2f', b) }, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'w', ...
    'EdgeColor', [0.5 0.5 0.5], 'FontSize', 9);

sgtitle('Scotch Yoke Paddle Dynamics — LVAD Drive Unit', 'FontSize', 13, 'FontWeight', 'bold');

fprintf('--- Paddle Angle Summary ---\n');
fprintf('  Max angle:        %+.2f deg\n',  max(theta));
fprintf('  Min angle:        %+.2f deg\n',  min(theta));
fprintf('  Peak ang. vel:    %.1f deg/s\n', max(abs(dtheta_dt)));
fprintf('  Cycle period T:   %.2f ms\n',    T*1000);
fprintf('  Ejection window:  %.2f ms/cycle\n', b*T*1000);
