%% Crank-and-Slotted-Arm LVAD Drive Unit — Biventricular Paddle Dynamics
%
% MECHANISM: Motor crank pin sits in a RADIAL slot in the paddle arm
% (slot runs along the arm direction). Arm pivots about a fixed fulcrum
% at distance a from the crank centre.
%
% Kinematic formula (radial slot, crank-and-slotted-arm):
%   theta(phi) = arctan( x*sin(phi) / (a - x*cos(phi)) )
%
%   Max paddle angle alpha  = arcsin(x/a) ≈ 28.4°  (same value as scotch yoke)
%   Crank angle at max phi_pk = arccos(x/a) ≈ 61.6°  (NOT 90°)
%
%   The paddle reaches maximum and starts returning BEFORE the crank
%   reaches 90°. This is confirmed by the 2D mechanism model.
%
% Derivation of max crank angle:
%   dtheta/dphi = 0  =>  a*cos(phi) = x  =>  cos(phi) = x/a = r
%   phi_peak = arccos(r) = arccos(x/a) ≈ 61.6°
%
% Angular velocity (analytical derivative):
%   dtheta/dt = omega * x*(a*cos(phi) - x) / (a^2 - 2*a*x*cos(phi) + x^2)
%   Zero at phi = arccos(x/a) ✓
%   Profile is asymmetric: rapid compression, slower return through neutral.
%
% LV: paddle 0 -> +alpha  (crank 0° -> phi_pk ≈ 61.6°)
% RV: paddle 0 -> -alpha  (crank 180° -> 180°+phi_pk ≈ 241.6°)
%
% Overlap b in [0,1]: each bag encroaches b*alpha past the neutral position.
%   b = 0  => contact starts exactly at theta = 0
%   b = 1  => RV contact starts the instant LV reaches full compression
%
% Solving theta(phi) = b*alpha analytically for the return-stroke crossing:
%   phi_RV_start = 180 - arcsin(sin(b*alpha)/r) - b*alpha
%   phi_LV_start = 360 - arcsin(sin(b*alpha)/r) - b*alpha

clear; clc; close all;

%% Parameters
x     = 10;               % Crank arm length, mm
a     = 21;               % Crank centre to fulcrum distance, mm
r     = x / a;
alpha  = asind(r);        % Max paddle deflection, deg  (= arcsin(x/a) ≈ 28.4°)
phi_pk = acosd(r);        % Crank angle at max deflection  (= arccos(x/a) ≈ 61.6°)

rpm_max = 234.0;
T       = 60 / rpm_max;   % Cycle period, s
omega   = 2*pi / T;       % Motor angular velocity, rad/s

b     = 0;                              % Geometric overlap [0, 1]
delta = asind(sind(b * alpha) / r);       % Auxiliary overlap angle, deg

% Crank angles at which opposite bag starts contact (return-stroke crossing)
phi_RV_start = 180 - delta - b*alpha;
phi_LV_start = 360 - delta - b*alpha;
d_LV = phi_LV_start / 360;
d_RV = phi_RV_start / 360;

%% Time vector (2 cycles)
N   = 2000;
t   = linspace(0, 2*T, N);
phi = omega .* t;   % crank angle, radians

%% Kinematics — crank-and-slotted-arm (radial slot)
theta     = atand(x .* sin(phi) ./ (a - x .* cos(phi)));
dtheta_dt = rad2deg(omega .* x .* (a .* cos(phi) - x) ./ ...
            (a^2 - 2*a*x .* cos(phi) + x^2));

%% Ejection window time segments (ms)
lv_segs = [0,           T * phi_pk/360;
           T * d_LV,    T + T * phi_pk/360;
           T + T*d_LV,  2*T              ] * 1000;

rv_segs = [T * d_RV,    T * (180 + phi_pk)/360;
           T + T*d_RV,  T + T*(180 + phi_pk)/360] * 1000;

lv_col = [0.72 0.87 1.00];   % light blue — LV
rv_col = [1.00 0.78 0.78];   % light red  — RV

%% ================================================================
%  Figure 1: Time-domain dynamics — two cycles
%% ================================================================
figure('Name','Biventricular Paddle Dynamics','Color','w','Position',[80 80 960 600]);

ax1 = subplot(2,1,1);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000, theta,'k-','LineWidth',2);
yline( alpha,'b--','LineWidth',1.2,'Label',sprintf('+\\alpha = %.1f°', alpha),'LabelHorizontalAlignment','left');
yline(-alpha,'r--','LineWidth',1.2,'Label',sprintf('-\\alpha = %.1f°',-alpha),'LabelHorizontalAlignment','left');
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1,'Label','Cycle 2','LabelVerticalAlignment','bottom');
xline(2*T*1000,'k--','LineWidth',1,'Label','End',    'LabelVerticalAlignment','bottom');
hold off;
xlabel('Time (ms)'); ylabel('Paddle Angle (°)');
title('Paddle Angle vs Time');
legend({'LV ejection','RV ejection','Paddle angle'},'Location','northeast');
grid on; xlim([0 2*T*1000]); ylim([-alpha*1.3, alpha*1.3]);
ax1.FontSize = 11;

ax2 = subplot(2,1,2);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000, dtheta_dt,'k-','LineWidth',2);
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1);
xline(2*T*1000,'k--','LineWidth',1);
hold off;
xlabel('Time (ms)'); ylabel('d\theta/dt (°/s)');
title('Paddle Angular Velocity vs Time');
grid on; xlim([0 2*T*1000]);
ax2.FontSize = 11;

annotation('textbox',[0.71 0.47 0.23 0.19],'String',{
    sprintf('x = %g mm  |  a = %g mm', x, a),
    sprintf('r = x/a = %.3f', r),
    sprintf('\\alpha = %.1f°  (= arcsin r)', alpha),
    sprintf('\\phi_{peak} = %.1f°  (= arccos r,  NOT 90°)', phi_pk),
    sprintf('rpm = %.1f  |  T = %.1f ms', rpm_max, T*1000),
    sprintf('Overlap b = %.2f  =>  \\delta = %.1f°', b, delta)}, ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor',[.5 .5 .5],'FontSize',9);

sgtitle('Crank-and-Slotted-Arm LVAD Drive Unit — Biventricular Paddle Dynamics', ...
    'FontSize',13,'FontWeight','bold');

%% ================================================================
%  Figure 2: Crank angle vs Paddle angle
%% ================================================================
phi_deg   = linspace(0, 360, 1000);
theta_phi = atand(x .* sind(phi_deg) ./ (a - x .* cosd(phi_deg)));

figure('Name','Crank vs Paddle Angle','Color','w','Position',[120 120 820 520]);
hold on;

xregion(0,             phi_pk,       'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_LV_start,  360,          'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_RV_start,  180+phi_pk,   'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);

plot(phi_deg, theta_phi,'k-','LineWidth',2);

yline( alpha,'b--','LineWidth',1.2,'Label',sprintf('+\\alpha = %.1f°', alpha),'LabelHorizontalAlignment','left');
yline(-alpha,'r--','LineWidth',1.2,'Label',sprintf('-\\alpha = %.1f°',-alpha),'LabelHorizontalAlignment','left');
yline(0,'k:','LineWidth',0.8);

xline(0,          'k--','LineWidth',1,'Label','0°',                        'LabelVerticalAlignment','bottom');
xline(phi_pk,     'b:' ,'LineWidth',1,'Label',sprintf('%.1f°',phi_pk),     'LabelVerticalAlignment','bottom');
xline(180,        'k--','LineWidth',1,'Label','180°',                       'LabelVerticalAlignment','bottom');
xline(180+phi_pk, 'r:' ,'LineWidth',1,'Label',sprintf('%.1f°',180+phi_pk), 'LabelVerticalAlignment','bottom');
xline(360,        'k--','LineWidth',1,'Label','360°',                       'LabelVerticalAlignment','bottom');

xline(phi_RV_start,'r--','LineWidth',1,'Label',sprintf('RV start %.1f°',phi_RV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');
xline(phi_LV_start,'b--','LineWidth',1,'Label',sprintf('LV start %.1f°',phi_LV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');

plot(phi_pk,       alpha,'bs','MarkerFaceColor','b','MarkerSize',8);
plot(180+phi_pk,  -alpha,'rs','MarkerFaceColor','r','MarkerSize',8);

hold off;
xlabel('Crank Angle (°)');
ylabel('Paddle Angle (°)');
title('Crank Angle vs Paddle Angle — Crank-and-Slotted-Arm Kinematics','FontSize',12);
legend({'LV ejection','RV ejection','Paddle angle'},'Location','northeast');
grid on; xlim([0 360]); ylim([-alpha*1.3, alpha*1.3]);
xticks(0:45:360);

eject_deg = phi_pk + delta + b*alpha;
annotation('textbox',[0.64 0.11 0.24 0.27],'String',{
    sprintf('x = %g mm  |  a = %g mm', x, a),
    sprintf('r = x/a = %.3f', r),
    sprintf('\\alpha = %.1f°  (max paddle angle)', alpha),
    sprintf('\\phi_{peak} = %.1f°  (NOT 90°)', phi_pk),
    sprintf('Overlap b = %.2f  |  \\delta = %.1f°', b, delta),
    sprintf('LV: 0° to %.1f°  + lead-in from %.1f°', phi_pk, phi_LV_start),
    sprintf('RV: %.1f° to %.1f°', phi_RV_start, 180+phi_pk),
    sprintf('Each ventricle: %.1f° = %.0f%% of cycle', eject_deg, eject_deg/360*100)}, ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor',[.5 .5 .5],'FontSize',9);

%% Console summary
fprintf('=== Crank-and-Slotted-Arm LVAD ===\n');
fprintf('  Max paddle angle:         +/-%.2f deg  (= arcsin(x/a))\n', alpha);
fprintf('  Crank angle at max:       %.1f deg  (= arccos(x/a), NOT 90 deg)\n', phi_pk);
fprintf('  Overlap b:                %.2f  |  delta = %.1f deg\n', b, delta);
fprintf('  LV ejection:              crank %.1f to %.1f deg  (wrapping through 0)\n', phi_LV_start, phi_pk);
fprintf('  RV ejection:              crank %.1f to %.1f deg\n', phi_RV_start, 180+phi_pk);
fprintf('  Ejection per ventricle:   %.1f deg crank = %.0f%% of cycle\n', eject_deg, eject_deg/360*100);
fprintf('  Ejection time/ventricle:  %.2f ms\n', eject_deg/360*T*1000);
fprintf('  Diastole gap/ventricle:   %.1f deg = %.2f ms\n', 180-eject_deg, (180-eject_deg)/360*T*1000);
fprintf('  Peak angular velocity:    %.1f deg/s\n', max(abs(dtheta_dt)));
fprintf('  Cycle period T:           %.2f ms\n', T*1000);
