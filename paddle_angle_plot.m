%% Crank-and-Slotted-Arm LVAD Drive Unit — Biventricular Paddle Dynamics
%
% MECHANISM: Motor crank pin in a RADIAL slot in the paddle arm.
%   theta(phi) = arctan( x*sin(phi) / (a - x*cos(phi)) )
%   phi_peak   = arccos(x/a) ≈ 61.6°  (NOT 90° — quick-return crank-rocker)
%
% FLOW RATE:
%   Q = (dV/dtheta) * (dtheta/dt)
%   dV/dtheta = w * (f_contact * L)^2 / 2   [mm³/rad]
%   Contact zone: radially from pivot to f_contact * L
%   Q is proportional to angular velocity — peak flow at START of compression.
%
% KEY PARAMETERS TO TUNE:
%   b         — geometric bag overlap [0,1]
%   L         — paddle length (radial, mm)
%   w         — paddle width (mm)
%   L_contact — contact length from tip of paddle (mm)
%   p_bag     — blood bag pressure (Pa)
%   F_e       — bag elasticity force (N)

clear; clc; close all;

%% Mechanism parameters
x     = 10;               % Crank arm length, mm
a     = 21;               % Crank centre to fulcrum distance, mm
r     = x / a;
alpha  = asind(r);        % Max paddle deflection, deg
phi_pk = acosd(r);        % Crank angle at max deflection, deg  (≈ 61.6°)

rpm_max = 234.0;
T       = 60 / rpm_max;
omega_gb = 2*pi / T;   % Crank shaft (gearbox output) angular velocity, rad/s

b     = 0.9;                              % Geometric overlap [0, 1]
delta = asind(sind(b * alpha) / r);

phi_RV_start = 180 - delta - b*alpha;
phi_LV_start = 360 - delta - b*alpha;
d_LV = phi_LV_start / 360;
d_RV = phi_RV_start / 360;

%% Paddle geometry — tune these
L         = 40;     % Paddle length (radial extent from pivot), mm
w         = 66;     % Paddle width (perpendicular to arm), mm
L_contact = 20;     % Contact length from tip of paddle, mm  (0 < L_contact <= L)
%                     Contact zone: radius (L - L_contact) -> L

K_geom = w * (L^2 - (L - L_contact)^2) / 2;   % dV/dtheta, mm³/rad

%% Time vector (2 cycles)
N   = 2000;
t   = linspace(0, 2*T, N);
phi = omega_gb .* t;

%% Kinematics
theta     = atand(x .* sin(phi) ./ (a - x .* cos(phi)));
dtheta_dt = rad2deg(omega_gb .* x .* (a .* cos(phi) - x) ./ ...
            (a^2 - 2*a*x .* cos(phi) + x^2));

%% Flow rate
dtheta_dt_rad = dtheta_dt * pi/180;   % rad/s

% Crank angle at each timestep [0, 360)
phi_deg_t = mod(phi * 180/pi, 360);

% Ejection window masks
lv_mask = (phi_deg_t >= phi_LV_start) | (phi_deg_t <= phi_pk);
rv_mask = (phi_deg_t >= phi_RV_start) & (phi_deg_t <= 180 + phi_pk);

% Q [mL/s]: only compression direction, only during ejection window
Q_LV    = max(0,  dtheta_dt_rad) .* double(lv_mask) * K_geom / 1000;
Q_RV    = max(0, -dtheta_dt_rad) .* double(rv_mask) * K_geom / 1000;
Q_total = Q_LV + Q_RV;

% Stroke volume per ventricle [mL] = K * integral(dtheta) = K * alpha_rad
SV      = K_geom * alpha * pi/180 / 1000;          % mL
CO      = 2 * SV * rpm_max / 1000;                 % L/min (both ventricles)

%% Torque
p_bag     = 16e3;                          % Blood bag pressure, Pa
F_e       = 10;                            % Bag elasticity force, N
A_contact = w * L_contact * 1e-6;          % Bag contact area, m²
F_total   = p_bag * A_contact + F_e;       % Total resistive force, N  (constant)
r_moment  = (L - L_contact/2) * 1e-3;     % Moment arm: midpoint of contact zone, m

Tp_mag   = F_total * r_moment;             % N·m
Tp_LV    = Tp_mag * double(lv_mask);       % N·m, positive during LV ejection
Tp_RV    = Tp_mag * double(rv_mask);       % N·m, positive magnitude during RV ejection
Tp_total = Tp_LV - Tp_RV;                 % signed: +LV, −RV

% Gearbox torque: T_g = T_p * dtheta/dphi  (virtual work: T_g*dphi = T_p*dtheta)
% dtheta/dphi = dtheta_dt_rad / omega_gb
% LV: motor overcomes +Tp_mag, T_g = Tp_mag * dtheta/dphi  (positive, peaks at phi=0)
% RV: motor overcomes +Tp_mag in -theta direction, T_g = -Tp_mag * dtheta/dphi (positive, peaks at phi=180)
dtheta_dphi = dtheta_dt_rad / omega_gb;
Tg_LV    =  Tp_mag .* dtheta_dphi .* double(lv_mask);   % N·m, positive
Tg_RV    = -Tp_mag .* dtheta_dphi .* double(rv_mask);   % N·m, positive
Tg       = Tg_LV + Tg_RV;

% Motor torque: T_m = T_g / (GR * e_gb * e_mech)
GR      = 62;    % Gear ratio
e_gb    = 0.74;  % Gearbox efficiency
e_mech  = 0.72;  % Mechanical efficiency (crank-and-slotted-arm linkage)
e_motor = 0.81;  % Motor efficiency (from datasheet)
Tm      = Tg / (GR * e_gb * e_mech);

% Power
omega_motor = omega_gb * GR;               % Motor shaft speed, rad/s
P_mech      = Tm .* omega_motor;           % Mechanical power at motor shaft, W
P_elec      = P_mech / e_motor;            % Electrical input power, W

%% Ejection window time segments (ms)
lv_segs = [0,           T * phi_pk/360;
           T * d_LV,    T + T * phi_pk/360;
           T + T*d_LV,  2*T              ] * 1000;

rv_segs = [T * d_RV,    T * (180 + phi_pk)/360;
           T + T*d_RV,  T + T*(180 + phi_pk)/360] * 1000;

lv_col = [0.72 0.87 1.00];
rv_col = [1.00 0.78 0.78];

%% ================================================================
%  Figure 1: Kinematics — two cycles
%% ================================================================
figure('Name','Biventricular Paddle Dynamics','Color','w','Position',[60 60 980 700]);

ax1 = subplot(3,1,1);
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
ylabel('Paddle Angle (°)');
title('Paddle Angle vs Time');
legend({'LV ejection','RV ejection','Paddle angle'},'Location','northeast');
grid on; xlim([0 2*T*1000]); ylim([-alpha*1.3, alpha*1.3]);
ax1.FontSize = 10;

ax2 = subplot(3,1,2);
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
ylabel('d\theta/dt (°/s)');
title('Paddle Angular Velocity vs Time');
grid on; xlim([0 2*T*1000]);
ax2.FontSize = 10;

ax3 = subplot(3,1,3);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000, Q_LV,   'b-', 'LineWidth', 2);
plot(t*1000, Q_RV,   'r-', 'LineWidth', 2);
plot(t*1000, Q_total,'k--','LineWidth', 1.2);
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1);
xline(2*T*1000,'k--','LineWidth',1);
hold off;
xlabel('Time (ms)'); ylabel('Flow Rate (mL/s)');
title('Instantaneous Flow Rate vs Time');
legend({'LV flow','RV flow','Total'},'Location','northeast');
grid on; xlim([0 2*T*1000]); ylim([0, max(Q_total)*1.25]);
ax3.FontSize = 10;

annotation('textbox',[0.72 0.35 0.26 0.28],'String',{
    sprintf('x = %g mm  |  a = %g mm', x, a),
    sprintf('\\phi_{peak} = %.1f°  (arccos r,  NOT 90°)', phi_pk),
    sprintf('\\alpha = %.1f°  |  rpm = %.1f', alpha, rpm_max),
    sprintf('Overlap b = %.2f  |  \\delta = %.1f°', b, delta),
    sprintf('─────────────────────'),
    sprintf('L = %g mm  |  w = %g mm', L, w),
    sprintf('L_{contact} = %g mm from tip  (r: %.0f–%g mm)', L_contact, L-L_contact, L),
    sprintf('SV = %.1f mL / ventricle', SV),
    sprintf('CO = %.2f L/min / ventricle', SV * rpm_max / 1000),
    sprintf('CO = %.2f L/min (combined)', CO)}, ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor',[.5 .5 .5],'FontSize',8.5);

sgtitle('Crank-and-Slotted-Arm LVAD — Kinematics', ...
    'FontSize',12,'FontWeight','bold');

%% ================================================================
%  Figure 2: Forces & Torques — two cycles
%% ================================================================
figure('Name','LVAD Forces & Torques','Color','w','Position',[100 40 980 1050]);

ax4 = subplot(5,1,1);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000, F_total * double(lv_mask), 'b-', 'LineWidth', 2);
plot(t*1000, F_total * double(rv_mask), 'r-', 'LineWidth', 2);
yline(F_total,'k:','LineWidth',0.8,'Label',sprintf('F_{total} = %.1f N', F_total),'LabelHorizontalAlignment','left');
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1,'Label','Cycle 2','LabelVerticalAlignment','bottom');
xline(2*T*1000,'k--','LineWidth',1,'Label','End',    'LabelVerticalAlignment','bottom');
hold off;
ylabel('F_{total} (N)');
title('F_{total} Applied to Paddle vs Time');
legend({'LV','RV'},'Location','northeast');
grid on; xlim([0 2*T*1000]); ylim([0, F_total * 1.4]);
ax4.FontSize = 10;

ax5 = subplot(5,1,2);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000,  Tp_LV,   'b-', 'LineWidth', 2);
plot(t*1000, -Tp_RV,   'r-', 'LineWidth', 2);
plot(t*1000,  Tp_total,'k--','LineWidth', 1.2);
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1);
xline(2*T*1000,'k--','LineWidth',1);
hold off;
ylabel('T_p (N·m)');
title('Paddle Torque vs Time');
legend({'LV load','RV load','Net'},'Location','northeast');
grid on; xlim([0 2*T*1000]);
ax5.FontSize = 10;

ax6 = subplot(5,1,3);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000, Tg_LV,'b-','LineWidth',2);
plot(t*1000, Tg_RV,'r-','LineWidth',2);
plot(t*1000, Tg,   'k--','LineWidth',1.2);
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1);
xline(2*T*1000,'k--','LineWidth',1);
hold off;
ylabel('T_g (N·m)');
title('Gearbox Output Torque vs Time');
legend({'LV contribution','RV contribution','Total T_g'},'Location','northeast');
grid on; xlim([0 2*T*1000]); ylim([0, max(Tg)*1.25]);
ax6.FontSize = 10;

ax7 = subplot(5,1,4);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000, Tm,'k-','LineWidth',2);
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1);
xline(2*T*1000,'k--','LineWidth',1);
hold off;
xlabel('Time (ms)'); ylabel('T_m (N·m)');
title('Motor Torque vs Time');
grid on; xlim([0 2*T*1000]); ylim([0, max(Tm)*1.25]);
ax7.FontSize = 10;

ax8 = subplot(5,1,5);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000, P_elec,'k-','LineWidth',2);
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1);
xline(2*T*1000,'k--','LineWidth',1);
hold off;
xlabel('Time (ms)'); ylabel('P_{elec} (W)');
title('Electrical Input Power vs Time');
grid on; xlim([0 2*T*1000]); ylim([0, max(P_elec)*1.25]);
ax8.FontSize = 10;

annotation('textbox',[0.72 0.01 0.26 0.32],'String',{
    sprintf('p_{bag} = %g kPa  |  F_e = %g N', p_bag/1e3, F_e),
    sprintf('A_{contact} = %.0f mm²', w*L_contact),
    sprintf('F_{total} = %.1f N', F_total),
    sprintf('r_{moment} = %.0f mm', r_moment*1000),
    sprintf('T_{p} = %.3f N·m', Tp_mag),
    sprintf('─────────────────────'),
    sprintf('T_{g,LV} = %.4f N·m  (xT_p/(a-x))', max(Tg_LV)),
    sprintf('T_{g,RV} = %.4f N·m  (xT_p/(a+x))', max(Tg_RV)),
    sprintf('─────────────────────'),
    sprintf('GR = %g  |  \\eta_{gb} = %.2f  |  \\eta_{mech} = %.2f', GR, e_gb, e_mech),
    sprintf('\\eta_{motor} = %.2f  |  T_{m,peak} = %.5f N·m', e_motor, max(Tm)),
    sprintf('P_{mech,peak} = %.3f W', max(P_mech)),
    sprintf('P_{elec,peak} = %.3f W', max(P_elec))}, ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor',[.5 .5 .5],'FontSize',8.5);

sgtitle('Crank-and-Slotted-Arm LVAD — Forces & Torques', ...
    'FontSize',12,'FontWeight','bold');

%% ================================================================
%  Figure 3: Crank angle vs Paddle angle
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
xlabel('Crank Angle (°)'); ylabel('Paddle Angle (°)');
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
fprintf('  Max paddle angle:         +/-%.2f deg\n', alpha);
fprintf('  Crank angle at max:       %.1f deg  (arccos(x/a), NOT 90)\n', phi_pk);
fprintf('  Quick-return ratio:       %.2f:1  ((a+x)/(a-x))\n', (a+x)/(a-x));
fprintf('  Overlap b:                %.2f  |  delta = %.1f deg\n', b, delta);
fprintf('  ─────────────────────────────────────\n');
fprintf('  Paddle L = %g mm  |  w = %g mm\n', L, w);
fprintf('  Contact from tip:         %.0f mm  (r: %.0f to %g mm)\n', L_contact, L-L_contact, L);
fprintf('  K_geom:                   %.0f mm3/rad\n', K_geom);
fprintf('  Stroke volume/ventricle:  %.1f mL\n', SV);
fprintf('  Peak LV flow:             %.1f mL/s\n', max(Q_LV));
fprintf('  Peak RV flow:             %.1f mL/s\n', max(Q_RV));
fprintf('  Combined cardiac output:  %.2f L/min\n', CO);
fprintf('  ─────────────────────────────────────\n');
fprintf('  Bag pressure:             %g kPa\n', p_bag/1e3);
fprintf('  Elasticity force F_e:     %.0f N\n', F_e);
fprintf('  F_total:                  %.1f N  (%.1f N pressure + %.1f N elastic)\n', F_total, p_bag*A_contact, F_e);
fprintf('  Moment arm:               %.1f mm\n', r_moment*1000);
fprintf('  Peak paddle torque T_p:   %.4f N·m\n', Tp_mag);
fprintf('  Peak T_g (LV, phi=0):    %.4f N·m  (= x*Tp/(a-x))\n', max(Tg_LV));
fprintf('  Peak T_g (RV, phi=180):  %.4f N·m  (= x*Tp/(a+x))\n', max(Tg_RV));
fprintf('  T_g ratio LV/RV:         %.2f  (= (a+x)/(a-x))\n', max(Tg_LV)/max(Tg_RV));
fprintf('  ─────────────────────────────────────\n');
fprintf('  GR = %g  |  e_gb = %.2f  |  e_mech = %.2f  |  e_motor = %.2f\n', GR, e_gb, e_mech, e_motor);
fprintf('  omega_gb:                %.2f rad/s  (%g rpm crank)\n', omega_gb, rpm_max);
fprintf('  omega_motor:             %.1f rad/s  (%.0f rpm motor)\n', omega_motor, omega_motor*60/(2*pi));
fprintf('  Peak motor torque T_m:   %.5f N·m\n', max(Tm));
fprintf('  Peak P_mech:             %.3f W\n', max(P_mech));
fprintf('  Peak P_elec:             %.3f W\n', max(P_elec));
fprintf('  Mean P_elec:             %.3f W\n', mean(P_elec));
fprintf('  Cycle period T:           %.2f ms\n', T*1000);
