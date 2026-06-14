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
x     = 11;               % Crank arm length, mm
a     = 23;               % Crank centre to fulcrum distance, mm
r     = x / a;
alpha  = asind(r);        % Max paddle deflection, deg
phi_pk = acosd(r);        % Crank angle at max deflection, deg  (≈ 61.6°)

%% Teardrop slot parameters
R_top    = 24;    % mm — tip-end fillet radius (small; sets slot width for visualisation)
R_bottom = 6;   % mm — near-pivot arc radius (increase to slow quick-return more)
s_bc     = 5;    % mm — centre of bottom arc along paddle arm from fulcrum
%   Transition: pin enters bottom arc when R_fp < s_bc + R_bottom  (default: 17 mm)
%   R_fp at phi=0 is a-x=12mm; at phi_pk is ~20.2mm — arc only active near phi≈0

rpm_max = 145;
T       = 60 / rpm_max;
omega_gb = 2*pi / T;   % Crank shaft (gearbox output) angular velocity, rad/s

b     = 0.5;   % Bag overlap [0,0.5]: fill at theta=0 is (1-b)*100%; b=0 -> 100%, b=0.5 -> 50%
               % gamma = alpha*b/(1-b): paddle angle (deg) by which bag contact extends past neutral
               % LV bag first contacts at theta=-gamma; RV bag first contacts at theta=+gamma

% gamma: extension of bag contact past neutral (deg); fill at theta=0 = (1-b)
gamma = alpha * b / (1 - b);

asind_arg    = min(1, sind(gamma) / r);          % clamp to avoid NaN at b=0.5 (float round-trip)
phi_RV_start = 180 - gamma - asind(asind_arg);
phi_LV_start = 360 + gamma - asind(asind_arg);
d_LV = phi_LV_start / 360;
d_RV = phi_RV_start / 360;

%% Paddle geometry — tune these
L         = 30;     % Paddle length (radial extent from pivot), mm
w         = 80;     % Paddle width (perpendicular to arm), mm
L_contact = 25;     % Contact length from tip of paddle, mm  (0 < L_contact <= L)
%                     Contact zone: radius (L - L_contact) -> L

K_geom = w * (L^2 - (L - L_contact)^2) / 2;   % dV/dtheta, mm³/rad

%% Time vector (2 cycles)
N   = 2000;
t   = linspace(0, 2*T, N);
phi = omega_gb .* t;

%% Kinematics — teardrop slot
A_pin     = a - x .* cos(phi);
B_pin     =     x .* sin(phi);
R_fp2     = A_pin.^2 + B_pin.^2;
R_fp      = sqrt(R_fp2);
angle_pin = atan2(B_pin, A_pin);          % rad

% Pin radius = R_top (fits snugly in straight section), so pin centre traces
% an arc of radius R_eff = R_bottom - R_top around s_bc in the bottom cap.
R_eff  = R_bottom - R_top;              % effective arc radius for pin centre (mm)
R_eff2 = R_eff^2;

on_arc = R_fp < (s_bc + R_eff);         % logical mask — bottom arc active

% Straight section (original formula)
theta_straight = angle_pin;              % rad; equivalent to original atand formula

% Bottom arc: cos(theta - angle_pin) = C
C     = (R_fp2 + s_bc^2 - R_eff2) ./ (2 .* R_fp .* s_bc);
C     = max(-1, min(1, C));
theta_arc = angle_pin - acos(C);         % minus branch → slower dwell near phi=0

% Blend
theta          = theta_straight;
theta(on_arc)  = theta_arc(on_arc);
theta          = rad2deg(theta);         % degrees — matches rest of file

% dtheta/dt via numerical differentiation (handles both sections cleanly)
dtheta_dt = gradient(theta, t);          % deg/s

%% Ejection window boundaries — re-derived from actual teardrop theta(phi)
% The straight-slot formula for phi_LV/RV_start assumes theta = arctan(...).
% With the teardrop, theta(phi) differs near phi=0, so we find the crossings numerically.
half         = N/2;
phi_cyc1_deg = mod(phi(1:half) * 180/pi, 360);
theta_cyc1   = theta(1:half);

% LV start: theta rises through -gamma (bag first contacts paddle on LV return stroke)
sig_lv  = theta_cyc1 - (-gamma);
lv_rise = find(sig_lv(1:end-1) .* sig_lv(2:end) < 0 & diff(theta_cyc1) > 0);
if ~isempty(lv_rise)
    [~, i_best]  = min(abs(phi_cyc1_deg(lv_rise) - phi_LV_start));
    phi_LV_start = phi_cyc1_deg(lv_rise(i_best));
end

% RV start: theta falls through +gamma (bag first contacts paddle on RV return stroke)
sig_rv  = theta_cyc1 - gamma;
rv_fall = find(sig_rv(1:end-1) .* sig_rv(2:end) < 0 & diff(theta_cyc1) < 0);
if ~isempty(rv_fall)
    [~, i_best]  = min(abs(phi_cyc1_deg(rv_fall) - phi_RV_start));
    phi_RV_start = phi_cyc1_deg(rv_fall(i_best));
end

d_LV = phi_LV_start / 360;
d_RV = phi_RV_start / 360;

%% Flow rate
dtheta_dt_rad = dtheta_dt * pi/180;   % rad/s

% Crank angle at each timestep [0, 360)
phi_deg_t = mod(phi * 180/pi, 360);

% Ejection window masks
lv_mask = (phi_deg_t >= phi_LV_start) | (phi_deg_t <= phi_pk);
rv_mask = (phi_deg_t >= phi_RV_start) & (phi_deg_t <= 360 - phi_pk);

% Q [mL/s]: only compression direction, only during ejection window
Q_LV    = max(0,  dtheta_dt_rad) .* double(lv_mask) * K_geom / 1000;
Q_RV    = max(0, -dtheta_dt_rad) .* double(rv_mask) * K_geom / 1000;
Q_total = Q_LV + Q_RV;

% Stroke volume per ventricle [mL]:
% Bag compressed from theta=-b*alpha to theta=+alpha → total sweep = (1+b)*alpha
SV      = K_geom * alpha * pi/180 / (1000 * (1-b));  % mL
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
e_gb    = 0.90;  % Gearbox efficiency
e_mech  = 0.72;  % Mechanical efficiency (crank-and-slotted-arm linkage)
e_motor = 0.83;  % Motor efficiency (from datasheet)
Tm      = Tg / (GR * e_gb * e_mech);

% Power
omega_motor = omega_gb * GR;               % Motor shaft speed, rad/s
P_mech      = Tm .* omega_motor;           % Mechanical power at motor shaft, W
P_elec      = P_mech / e_motor;            % Electrical input power, W

%% Ejection window time segments (ms)
lv_segs = [0,           T * phi_pk/360;
           T * d_LV,    T + T * phi_pk/360;
           T + T*d_LV,  2*T              ] * 1000;

rv_segs = [T * d_RV,    T * (360 - phi_pk)/360;
           T + T*d_RV,  T + T*(360 - phi_pk)/360] * 1000;

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
    sprintf('Overlap b = %.2f  |  \\gamma = %.1f°', b, gamma),
    sprintf('─────────────────────'),
    sprintf('L = %g mm  |  w = %g mm', L, w),
    sprintf('L_{contact} = %g mm from tip  (r: %.0f–%g mm)', L_contact, L-L_contact, L),
    sprintf('SV = %.1f mL / ventricle', SV),
    sprintf('CO = %.2f L/min / ventricle', SV * rpm_max / 1000),
    sprintf('CO = %.2f L/min (combined)', CO)}, ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor',[.5 .5 .5],'FontSize',8.5);

sgtitle('Crank-and-Slotted-Arm LVAD — Kinematics (Teardrop Slot)', ...
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

sgtitle('Crank-and-Slotted-Arm LVAD — Forces & Torques (Teardrop Slot)', ...
    'FontSize',12,'FontWeight','bold');

%% ================================================================
%  Figure 3: Crank angle vs Paddle angle
%% ================================================================
phi_deg   = linspace(0, 360, 1000);
theta_phi = atand(x .* sind(phi_deg) ./ (a - x .* cosd(phi_deg)));

% Teardrop curve for comparison
phi_r  = deg2rad(phi_deg);
A_v    = a - x.*cos(phi_r);  B_v = x.*sin(phi_r);
R_v    = sqrt(A_v.^2 + B_v.^2);
C_v    = max(-1, min(1, (R_v.^2 + s_bc^2 - R_eff^2)./(2.*R_v.*s_bc)));
td_arc = R_v < (s_bc + R_eff);
theta_td           = rad2deg(atan2(B_v, A_v));
theta_td(td_arc)   = rad2deg(atan2(B_v(td_arc), A_v(td_arc)) - acos(C_v(td_arc)));

figure('Name','Crank vs Paddle Angle','Color','w','Position',[120 120 820 520]);
hold on;

xregion(0,             phi_pk,       'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_LV_start,  360,          'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_RV_start,  360-phi_pk,   'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);

plot(phi_deg, theta_phi,'k-','LineWidth',2);
plot(phi_deg, theta_td, 'b--','LineWidth',1.5);

yline( alpha,'b--','LineWidth',1.2,'Label',sprintf('+\\alpha = %.1f°', alpha),'LabelHorizontalAlignment','left');
yline(-alpha,'r--','LineWidth',1.2,'Label',sprintf('-\\alpha = %.1f°',-alpha),'LabelHorizontalAlignment','left');
yline(0,'k:','LineWidth',0.8);

xline(0,          'k--','LineWidth',1,'Label','0°',                        'LabelVerticalAlignment','bottom');
xline(phi_pk,     'b:' ,'LineWidth',1,'Label',sprintf('%.1f°',phi_pk),     'LabelVerticalAlignment','bottom');
xline(180,        'k--','LineWidth',1,'Label','180°',                       'LabelVerticalAlignment','bottom');
xline(360-phi_pk, 'r:' ,'LineWidth',1,'Label',sprintf('%.1f°',360-phi_pk), 'LabelVerticalAlignment','bottom');
xline(360,        'k--','LineWidth',1,'Label','360°',                       'LabelVerticalAlignment','bottom');

xline(phi_RV_start,'r--','LineWidth',1,'Label',sprintf('RV start %.1f°',phi_RV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');
xline(phi_LV_start,'b--','LineWidth',1,'Label',sprintf('LV start %.1f°',phi_LV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');

plot(phi_pk,       alpha,'bs','MarkerFaceColor','b','MarkerSize',8);
plot(360-phi_pk,  -alpha,'rs','MarkerFaceColor','r','MarkerSize',8);

hold off;
xlabel('Crank Angle (°)'); ylabel('Paddle Angle (°)');
title('Crank Angle vs Paddle Angle — Straight vs Teardrop Slot','FontSize',12);
legend({'LV ejection','RV ejection','Straight slot','Teardrop slot'},'Location','northeast');
grid on; xlim([0 360]); ylim([-alpha*1.3, alpha*1.3]);
xticks(0:45:360);

eject_deg = phi_pk + asind(asind_arg) - gamma;

%% ================================================================
%  Figure 4: Teardrop slot profile (external-tangent geometry)
%% ================================================================
% The slot outline = two circles joined by their external common tangents.
% sin(tilt) = (R_bottom - R_top) / D  where D = centre-to-centre distance.
% The tangent lines are NOT parallel — they converge toward the smaller circle.

s_top_cen_fig = L - R_top;                    % top circle centre (slot tip at s = L)
D_cen         = s_top_cen_fig - s_bc;         % centre-to-centre distance

A_tan = (R_bottom - R_top) / D_cen;           % sin of tangent-line tilt
B_tan = sqrt(max(0, 1 - A_tan^2));            % cos of tangent-line tilt
phi_T = atan2(B_tan, A_tan);                  % angle to tangent points on each circle

% Top cap: arc on top circle, clockwise from upper to lower tangent point (through tip)
th_top    = linspace(phi_T, -phi_T, 300);
s_top_arc = s_top_cen_fig + R_top.*cos(th_top);
n_top_arc = R_top.*sin(th_top);

% Bottom cap: arc on bottom circle, clockwise from lower to upper tangent point (through leftmost)
%   lower tangent point at angle -phi_T, upper at phi_T; go clockwise (decreasing) through -pi
th_bot    = linspace(-phi_T, phi_T - 2*pi, 300);
s_bot_arc = s_bc + R_bottom.*cos(th_bot);
n_bot_arc = R_bottom.*sin(th_bot);

% Lower tangent point on bottom circle — needed to close the gap between arcs
s_tp_bot_lo = s_bc + R_bottom*A_tan;   n_tp_bot_lo = -R_bottom*B_tan;

% Full outline: top arc → lower tangent line → bottom arc → (upper tangent auto-closed by fill)
s_outline = [s_top_arc, s_tp_bot_lo, s_bot_arc];
n_outline = [n_top_arc, n_tp_bot_lo,  n_bot_arc];

% Pin centre: centreline in straight section, R_eff circle in bottom cap
s_pc_str    = [s_bc + R_eff, s_top_cen_fig];
n_pc_str    = [0, 0];
th_eff      = linspace(0, 2*pi, 300);          % full R_eff circle for reference
s_pc_circle = s_bc + R_eff.*cos(th_eff);
n_pc_circle = R_eff.*sin(th_eff);

figure('Name','Slot Shape','Color','w','Position',[160 160 700 520]);
hold on;
fill(s_outline, n_outline, [0.75 0.88 1.00], 'EdgeColor',[0 0.45 0.74], ...
     'LineWidth', 2.5, 'DisplayName','Slot boundary');
plot(s_pc_str,    n_pc_str,    'k--', 'LineWidth', 1.5, 'DisplayName','Pin centre (straight)');
plot(s_pc_circle, n_pc_circle, 'r--', 'LineWidth', 1.2, ...
     'DisplayName', sprintf('Pin centre arc  R_{eff} = %.0f mm', R_eff));
xline(s_bc + R_eff,  'r:',  'LineWidth', 1, ...
      'Label', sprintf('Transition %.0f mm', s_bc+R_eff), 'LabelVerticalAlignment','bottom');
xline(L - L_contact, 'k-.', 'LineWidth', 1, ...
      'Label', 'Contact inner edge', 'LabelVerticalAlignment','bottom');
hold off;
xlabel('s — radial from fulcrum (mm)'); ylabel('n — transverse (mm)');
title(sprintf('Teardrop Slot  (R_{top} = %.0f mm,  R_{bottom} = %.0f mm,  s_{bc} = %.0f mm)', ...
      R_top, R_bottom, s_bc));
legend('Location','northwest'); grid on; axis equal;
xlim([s_bc - R_bottom - 2, L + R_top + 2]);
ylim([-(R_bottom + 2), R_bottom + 2]);

%% Console summary
fprintf('=== Crank-and-Slotted-Arm LVAD ===\n');
fprintf('  Max paddle angle:         +/-%.2f deg\n', alpha);
fprintf('  Crank angle at max:       %.1f deg  (arccos(x/a), NOT 90)\n', phi_pk);
fprintf('  Quick-return ratio:       %.2f:1  ((a+x)/(a-x))\n', (a+x)/(a-x));
fprintf('  Overlap b:                %.2f  |  gamma = %.1f deg\n', b, gamma);
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
fprintf('  ─────────────────────────────────────\n');
fprintf('  Teardrop: R_top=%.0f mm  R_bottom=%.0f mm  s_bc=%.0f mm\n', R_top, R_bottom, s_bc);
fprintf('  R_eff (pin centre arc) = %.0f mm  |  transition at R_fp = %.0f mm\n', ...
        R_eff, s_bc + R_eff);
