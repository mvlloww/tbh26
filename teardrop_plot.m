%% Crank-and-Slotted-Arm LVAD — Biventricular Paddle Dynamics (Teardrop Slot)
%
% Mirror of paddle_angle_plot.m for the teardrop-slot mechanism variant.
% The near-pivot end of the radial slot is replaced by a circular arc of
% radius r2, reducing the quick-return ratio.
%
% theta(phi) is computed numerically (no closed form for the teardrop).
% Ejection windows are found by numerical inversion of theta(phi).
% All torque/powpower er formulas are otherwise identical to paddle_angle_plot.m.
%
% KEY PARAMETERS TO TUNE:
%   r2        — teardrop (wall) radius (mm); must be >= r1 and r_eff=r2-r1 < x
%   r1        — crank pin radius (mm, fixed); physical floor for r2
%   b         — geometric bag overlap [0, 0.5]
%   L, w, L_contact — paddle geometry (mm)
%
% The pin's CENTRE doesn't ride on the wall itself -- it stays a constant
% distance r1 inside it. So the kinematics (theta(phi)) are driven by the
% effective arc radius r_eff = r2 - r1 (the pin centre's own path), not r2
% directly. r2 = r1 (smallest wall that can even contain the pin) gives
% r_eff = 0 -- i.e. zero teardrop effect, identical to the pure straight
% slot; any real quick-return benefit needs r2 > r1.
%
% t_paddle affects SV/CO (not the flow-rate curves): the paddle's own body
% over the contact zone is a solid wedge (w*t_paddle*L_contact) that
% permanently occupies housing volume the bag could otherwise fill -- a
% static displacement, independent of stroke angle -- subtracted directly
% from the swept (zero-thickness-blade) stroke volume.

clear; clc; close all;

%% Mechanism parameters
x     = 7;    % Crank arm length, mm
a     = 17;     % Crank centre to fulcrum distance, mm
r1    = 2.23;    % Crank pin radius, mm (fixed) — physical floor for r2
r2    = 2.85; % Teardrop (wall) radius, mm  (gives r_eff = r1 at this default)
r_eff = r2 - r1;   % pin-centre's effective arc radius — drives the kinematics

rpm_max  = 145;
T        = 60 / rpm_max;
omega_gb = 2*pi / T;

b = 0.5;   % Bag overlap [0, 0.5]: fill at theta=0 is (1-b)*100%

lv_fast = false;   % true → LV on quick-return (fast) stroke near phi=0
                  % false → LV on slow stroke near phi=180 (lower peak torque)

%% Paddle geometry — tune these
L         = 39;    % Paddle length (radial extent from pivot), mm
w         = 70;      % Paddle width (out of plane), mm
t_paddle  = 6;       % Paddle thickness (perpendicular to arm in mechanism plane), mm
L_contact = 32.5;      % Contact length from tip of paddle, mm

K_geom = w * (L^2 - (L - L_contact)^2) / 2;   % dV/dtheta, mm³/rad

%% Torque / power chain
p_LV_mmHg = 120;    % LV peak bag pressure, mmHg
p_RV_mmHg = 25;     % RV peak bag pressure, mmHg
mmHg2Pa   = 133.322;
p_LV      = p_LV_mmHg * mmHg2Pa;   % Pa
p_RV      = p_RV_mmHg * mmHg2Pa;   % Pa
F_e       = 10;     % Bag elasticity force, N (same for both)
GR        = 62;     % Gear ratio
e_gb      = 0.90;   % Gearbox efficiency
e_mech    = 0.72;   % Mechanical efficiency
e_motor   = 0.83;   % Motor efficiency

%% Fine phi sweep (one cycle) — basis for all kinematics
phi_fine        = linspace(0, 360, 1441);   % 0.25 deg resolution
theta_fine      = teardrop_theta(phi_fine, x, a, r_eff);
dth_dphi_fine   = gradient(theta_fine, phi_fine);   % dθ/dφ, deg/deg (= rad/rad)

% Kinematic derived quantities
[alpha, ipk] = max(theta_fine);
phi_pk       = phi_fine(ipk);
gamma        = alpha * b / (1-b);
qr_ratio     = max(dth_dphi_fine) / max(-dth_dphi_fine);   % LV/RV peak rate

% Ejection windows — numerical inversion of theta(phi)
% LV: rising half (phi <= phi_pk), find phi where theta = gamma
seg_up       = phi_fine <= phi_pk;
phi_a        = interp1(theta_fine(seg_up), phi_fine(seg_up), gamma);
phi_LV_start = 360 - phi_a;

% RV: falling half (phi_pk to 180), find phi where theta = gamma
seg_dn       = (phi_fine >= phi_pk) & (phi_fine <= 180);
th_dn        = theta_fine(seg_dn);
ph_dn        = phi_fine(seg_dn);
phi_RV_start = interp1(fliplr(th_dn), fliplr(ph_dn), gamma);   % flip: th_dn is decreasing

% Stroke volume & CO
% The paddle's own body (over the contact zone) is a solid wedge that
% permanently occupies housing volume the bag could otherwise fill --
% a static displacement, independent of stroke angle -- so it's
% subtracted directly from the swept (zero-thickness-blade) volume.
V_paddle = w * t_paddle * L_contact / 1000;      % mL — paddle's own displaced volume
SV_gross = K_geom * alpha * pi/180 / (1000 * (1-b));   % mL per ventricle, zero-thickness ideal
SV       = max(0, SV_gross - V_paddle);                % mL per ventricle, thickness-corrected
CO       = 2 * SV * rpm_max / 1000;                    % L/min combined

% Baseline (r2 = r1): the smallest buildable wall, giving r_eff = 0 -- i.e.
% zero teardrop effect, identical to the pure straight slot. Used as the
% reference in place of the unbuildable r_eff=0-via-r2=0 "straight slot".
r2_min           = r1;
theta_base       = teardrop_theta(phi_fine, x, a, r2_min - r1);
dth_dphi_base    = gradient(theta_base, phi_fine);
[alpha_base, ipk_base] = max(theta_base);
phi_pk_base      = phi_fine(ipk_base);
peak_fast_base   = max(dth_dphi_base);     % dth/dphi at fast stroke (phi~0)
peak_slow_base   = max(-dth_dphi_base);    % dth/dphi at slow stroke (phi~180)
qr_base          = peak_fast_base / peak_slow_base;

%% Time domain — 2 cycles
N         = 2000;
t         = linspace(0, 2*T, N);
phi_cont  = omega_gb * t;                          % rad, continuous
phi_deg_t = mod(phi_cont * 180/pi, 360);           % wrapped [0, 360)

theta_t       = interp1(phi_fine, theta_fine,    phi_deg_t);
dth_dphi_t    = interp1(phi_fine, dth_dphi_fine, phi_deg_t);  % dimensionless

dtheta_dt_rad = dth_dphi_t * omega_gb;            % rad/s  (dth_dphi_t is dimensionless)
dtheta_dt     = dtheta_dt_rad * (180/pi);         % °/s

% Physical ejection windows (fast = near phi=0, positive dtheta; slow = near phi=180)
fast_mask = (phi_deg_t >= phi_LV_start) | (phi_deg_t <= phi_pk);
slow_mask = (phi_deg_t >= phi_RV_start) & (phi_deg_t <= 360 - phi_pk);

% Assign LV/RV based on lv_fast flag
if lv_fast
    lv_mask = fast_mask;  rv_mask = slow_mask;
    sign_lv = +1;         sign_rv = -1;
else
    lv_mask = slow_mask;  rv_mask = fast_mask;
    sign_lv = -1;         sign_rv = +1;
end

% Flow rate (NOTE: uses K_geom directly, so these do NOT reflect the
% t_paddle displaced-volume correction applied to SV/CO below -- integrating
% Q_LV over its ejection window gives SV_gross, not the corrected SV)
Q_LV    = max(0,  sign_lv * dtheta_dt_rad) .* double(lv_mask) * K_geom / 1000;   % mL/s
Q_RV    = max(0,  sign_rv * dtheta_dt_rad) .* double(rv_mask) * K_geom / 1000;
Q_total = Q_LV + Q_RV;

% Paddle torque
A_contact  = w * L_contact * 1e-6;              % m²
F_total_LV = p_LV * A_contact + F_e;            % N
F_total_RV = p_RV * A_contact + F_e;            % N
r_moment   = (L - L_contact/2) * 1e-3;          % m
Tp_mag_LV  = F_total_LV * r_moment;             % N·m
Tp_mag_RV  = F_total_RV * r_moment;             % N·m

Tp_LV    = Tp_mag_LV * double(lv_mask);
Tp_RV    = Tp_mag_RV * double(rv_mask);
Tp_total = Tp_LV - Tp_RV;

% Gearbox torque: T_g = T_p × dθ/dφ  (virtual work, dimensionless ratio)
dth_dphi_dless =  dtheta_dt_rad / omega_gb;
Tg_LV = sign_lv * Tp_mag_LV .* dth_dphi_dless .* double(lv_mask);
Tg_RV = sign_rv * Tp_mag_RV .* dth_dphi_dless .* double(rv_mask);
Tg    = Tg_LV + Tg_RV;

% Motor torque & power
Tm          = Tg / (GR * e_gb * e_mech);
omega_motor = omega_gb * GR;
P_mech      = Tm .* omega_motor;
P_elec      = P_mech / e_motor;

%% Ejection shading segments (ms)
d_fast = phi_LV_start / 360;
d_slow = phi_RV_start / 360;
fast_segs = [0,            T * phi_pk/360;
             T * d_fast,   T + T * phi_pk/360;
             T + T*d_fast, 2*T              ] * 1000;
slow_segs = [T * d_slow,   T * (360 - phi_pk)/360;
             T + T*d_slow, T + T*(360 - phi_pk)/360] * 1000;
if lv_fast
    lv_segs = fast_segs;  rv_segs = slow_segs;
else
    lv_segs = slow_segs;  rv_segs = fast_segs;
end

lv_col = [0.72 0.87 1.00];
rv_col = [1.00 0.78 0.78];

%% ================================================================
%  Figure 1: Kinematics — two cycles
%% ================================================================
figure('Name','Biventricular Paddle Dynamics — Teardrop','Color','w','Position',[60 60 980 700]);

ax1 = subplot(3,1,1);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000, theta_t,'k-','LineWidth',2);
yline( alpha,'b--','LineWidth',1.2,'Label',sprintf('+\\alpha = %.1f°', alpha),'LabelHorizontalAlignment','left');
yline(-alpha,'r--','LineWidth',1.2,'Label',sprintf('-\\alpha = %.1f°',-alpha),'LabelHorizontalAlignment','left');
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1,'Label','Cycle 2','LabelVerticalAlignment','bottom');
xline(2*T*1000,'k--','LineWidth',1,'Label','End',    'LabelVerticalAlignment','bottom');
hold off;
ylabel('Paddle Angle (°)');
title('Paddle Angle vs Time  [teardrop slot]');
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
    sprintf('x = %g mm  |  a = %g mm  |  r2 = %g mm  (r1 = %g mm)', x, a, r2, r1),
    sprintf('\\phi_{peak} = %.1f°  (r2=r1: %.1f°)', phi_pk, phi_pk_base),
    sprintf('\\alpha = %.1f°  (r2=r1: %.1f°)', alpha, alpha_base),
    sprintf('QR ratio = %.3f  (r2=r1: %.3f)', qr_ratio, qr_base),
    sprintf('Overlap b = %.2f  |  \\gamma = %.1f°', b, gamma),
    sprintf('─────────────────────'),
    sprintf('L = %g mm  |  w = %g mm  |  t = %g mm', L, w, t_paddle),
    sprintf('L_{contact} = %g mm from tip', L_contact),
    sprintf('SV = %.1f mL / ventricle  (%.1f mL before t_{paddle}, -%.1f mL displaced)', SV, SV_gross, V_paddle),
    sprintf('CO = %.2f L/min / ventricle', SV * rpm_max / 1000),
    sprintf('CO = %.2f L/min (combined)', CO)}, ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor',[.5 .5 .5],'FontSize',8.5);

sgtitle('Crank-and-Slotted-Arm LVAD — Kinematics (Teardrop Slot)', ...
    'FontSize',12,'FontWeight','bold');

%% ================================================================
%  Figure 2: Forces & Torques — two cycles
%% ================================================================
figure('Name','LVAD Forces & Torques — Teardrop','Color','w','Position',[100 40 980 1050]);

ax4 = subplot(5,1,1);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000, F_total_LV * double(lv_mask),'b-','LineWidth',2);
plot(t*1000, F_total_RV * double(rv_mask),'r-','LineWidth',2);
yline(F_total_LV,'b:','LineWidth',0.8,'Label',sprintf('F_{LV} = %.1f N',F_total_LV),'LabelHorizontalAlignment','left');
yline(F_total_RV,'r:','LineWidth',0.8,'Label',sprintf('F_{RV} = %.1f N',F_total_RV),'LabelHorizontalAlignment','left');
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1,'Label','Cycle 2','LabelVerticalAlignment','bottom');
xline(2*T*1000,'k--','LineWidth',1,'Label','End',    'LabelVerticalAlignment','bottom');
hold off;
ylabel('F_{total} (N)');
title('F_{total} Applied to Paddle vs Time');
legend({'LV','RV'},'Location','northeast');
grid on; xlim([0 2*T*1000]); ylim([0, F_total_LV*1.4]);
ax4.FontSize = 10;

ax5 = subplot(5,1,2);
hold on;
for i = 1:size(lv_segs,1)
    xregion(lv_segs(i,1), lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
end
for i = 1:size(rv_segs,1)
    xregion(rv_segs(i,1), rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
end
plot(t*1000,  Tp_LV,   'b-', 'LineWidth',2);
plot(t*1000, -Tp_RV,   'r-', 'LineWidth',2);
plot(t*1000,  Tp_total,'k--','LineWidth',1.2);
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
plot(t*1000, Tg_LV,'b-', 'LineWidth',2);
plot(t*1000, Tg_RV,'r-', 'LineWidth',2);
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
ylabel('T_m (N·m)');
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
    sprintf('p_{LV} = %g mmHg  |  p_{RV} = %g mmHg', p_LV_mmHg, p_RV_mmHg),
    sprintf('F_e = %g N  |  A_{contact} = %.0f mm²', F_e, w*L_contact),
    sprintf('F_{LV} = %.1f N  |  F_{RV} = %.1f N', F_total_LV, F_total_RV),
    sprintf('r_{moment} = %.0f mm', r_moment*1000),
    sprintf('T_{p,LV} = %.3f N·m  |  T_{p,RV} = %.3f N·m', Tp_mag_LV, Tp_mag_RV),
    sprintf('─────────────────────'),
    sprintf('T_{g,LV} = %.4f N·m  (numerical peak)', max(Tg_LV)),
    sprintf('T_{g,RV} = %.4f N·m  (numerical peak)', max(Tg_RV)),
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
figure('Name','Crank vs Paddle Angle — Teardrop','Color','w','Position',[120 120 820 520]);
hold on;

if lv_fast
    fast_col3 = lv_col;  slow_col3 = rv_col;
    fast_lbl  = 'LV';    slow_lbl  = 'RV';
    pk_col    = 'b';     pk2_col   = 'r';
else
    fast_col3 = rv_col;  slow_col3 = lv_col;
    fast_lbl  = 'RV';    slow_lbl  = 'LV';
    pk_col    = 'r';     pk2_col   = 'b';
end
xregion(0,            phi_pk,     'FaceColor',fast_col3,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_LV_start, 360,        'FaceColor',fast_col3,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_RV_start, 360-phi_pk, 'FaceColor',slow_col3,'EdgeColor','none','FaceAlpha',0.75);

plot(phi_fine, theta_fine,'k-','LineWidth',2);

yline( alpha,'b--','LineWidth',1.2,'Label',sprintf('+\\alpha = %.1f°',alpha),'LabelHorizontalAlignment','left');
yline(-alpha,'r--','LineWidth',1.2,'Label',sprintf('-\\alpha = %.1f°',-alpha),'LabelHorizontalAlignment','left');
yline(0,'k:','LineWidth',0.8);

xline(0,          'k--','LineWidth',1,'Label','0°',                         'LabelVerticalAlignment','bottom');
xline(phi_pk,     [pk_col ':'] ,'LineWidth',1,'Label',sprintf('%.1f°',phi_pk),      'LabelVerticalAlignment','bottom');
xline(180,        'k--','LineWidth',1,'Label','180°',                        'LabelVerticalAlignment','bottom');
xline(360-phi_pk, [pk2_col ':'] ,'LineWidth',1,'Label',sprintf('%.1f°',360-phi_pk), 'LabelVerticalAlignment','bottom');
xline(360,        'k--','LineWidth',1,'Label','360°',                        'LabelVerticalAlignment','bottom');

xline(phi_RV_start,[pk2_col '--'],'LineWidth',1,'Label',sprintf('%s start %.1f°',slow_lbl,phi_RV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');
xline(phi_LV_start,[pk_col  '--'],'LineWidth',1,'Label',sprintf('%s start %.1f°',fast_lbl,phi_LV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');

plot(phi_pk,      alpha,[pk_col  's'],'MarkerFaceColor',pk_col, 'MarkerSize',8);
plot(360-phi_pk, -alpha,[pk2_col 's'],'MarkerFaceColor',pk2_col,'MarkerSize',8);

hold off;
xlabel('Crank Angle (°)'); ylabel('Paddle Angle (°)');
title('Crank Angle vs Paddle Angle — Teardrop Slot Kinematics','FontSize',12);
legend({'LV ejection','RV ejection','Paddle angle'},'Location','northeast');
grid on; xlim([0 360]); ylim([-alpha*1.3, alpha*1.3]);
xticks(0:45:360);

%% ================================================================
%  Figure 4: Slot Geometry & Crank Pin — static check
%% ================================================================
% Snapshots one crank angle (no animation) and draws the ACTUAL slot wall
% (not the pin-centre path) plus the pin's full circular path, so the
% r1/r2 geometry can be checked visually. phi_static=0 nests the pin in
% the near-pivot arc, mirroring the reference sketch used to derive this.
phi_static     = 0;    % crank angle to snapshot, deg
theta_static   = teardrop_theta(phi_static, x, a, r_eff);
theta_static_r = deg2rad(theta_static);

F = [0, 0];
C = [a, 0];
P = [a - x*cosd(phi_static), x*sind(phi_static)];

% Slot wall outline (body frame -> world frame): pin-centre path (r_eff,
% apex at a+x) scaled outward by k=r2/r_eff about the arc centre, far tip
% capped with its own r1 fillet -- same construction as teardrop_viz.m
ys  = a - x;
ci  = ys + r_eff;
Di  = 2*x - r_eff;
Txi = r_eff * sqrt(max(0, Di^2 - r_eff^2)) / Di;
Tyi = ci + r_eff^2 / Di;
thR = atan2(Tyi - ci, Txi);

k      = r2 / r_eff;
Txw    = Txi * k;
Tyw    = ci + (Tyi - ci) * k;
apex_w = ci + (a + x - ci) * k;

Yc  = apex_w - r1 * r2 / (Tyw - ci);
Txf = r1 * Txw / r2;
Tyf = Yc + r1 * (Tyw - ci) / r2;

aa_big = linspace(thR, -pi - thR, 200);   % near-pivot r2 arc
aa_tip = linspace(pi - thR, thR, 60);     % far-tip r1 fillet

bX = [r2*cos(aa_big),    r1*cos(aa_tip)   ];
bY = [ci+r2*sin(aa_big), Yc+r1*sin(aa_tip)];

slot_X = bY*cos(theta_static_r) - bX*sin(theta_static_r);
slot_Y = bY*sin(theta_static_r) + bX*cos(theta_static_r);

arm_dir = [cos(theta_static_r), sin(theta_static_r)];   % F -> slot apex direction
apex_pt = F + (a + x) * arm_dir;

figure('Name','Teardrop Slot Geometry — Static Check','Color','w','Position',[160 160 620 650]);
hold on;

ang = linspace(0, 2*pi, 200);
h1 = plot(C(1)+x*cos(ang), C(2)+x*sin(ang), 'b:', 'LineWidth', 1.2);                 % pin's circular path
h2 = plot([F(1) C(1)], [F(2) C(2)], 'k:', 'LineWidth', 0.8);                        % neutral axis F->C
h3 = plot([C(1) P(1)], [C(2) P(2)], 'b-', 'LineWidth', 2);                          % crank throw arm
h4 = plot([F(1) apex_pt(1)], [F(2) apex_pt(2)], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 3);  % arm centreline
h5 = fill(slot_X, slot_Y, [0.35 0.75 0.35], 'FaceAlpha', 0.35, 'EdgeColor', [0.15 0.55 0.15], 'LineWidth', 1.5); % slot wall
h6 = fill(P(1)+r1*cos(ang), P(2)+r1*sin(ang), [1.00 0.85 0.10], 'FaceAlpha', 0.85, 'EdgeColor', [0.55 0.40 0], 'LineWidth', 1.2); % pin, actual size
plot(F(1), F(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
plot(C(1), C(2), 'ko', 'MarkerFaceColor', [0.2 0.2 0.8], 'MarkerSize', 8, 'HandleVisibility', 'off');
text(F(1)-2, F(2)-4, 'F (pivot)', 'FontSize', 8);
text(C(1)+1, C(2)-4, 'C (crank centre)', 'FontSize', 8);

axis equal; grid on;
xlabel('mm'); ylabel('mm');
title(sprintf('Slot Wall & Crank Pin at \\phi=%.0f°  (static check)', phi_static));
legend([h1 h2 h3 h4 h5 h6], {'Pin path (radius x)','Neutral axis','Crank arm', ...
    'Arm centreline','Slot wall (r2, r1-filleted)','Crank pin (actual size, r1)'}, ...
    'Location','southoutside','NumColumns',2);

all_X = [C(1)+x*[-1 1], slot_X, apex_pt(1), F(1)];
all_Y = [C(2)+x*[-1 1], slot_Y, apex_pt(2), F(2)];
mgn   = 0.1*(a+x);
xlim([min(all_X)-mgn, max(all_X)+mgn]);
ylim([min(all_Y)-mgn, max(all_Y)+mgn]);

annotation('textbox',[0.15 0.01 0.7 0.08], ...
    'String', sprintf('x=%.2f mm  a=%.2f mm  r1(pin)=%.2f mm  r2(wall)=%.2f mm  r_{eff}=%.2f mm', ...
        x, a, r1, r2, r_eff), ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor',[.5 .5 .5],'FontSize',8.5, ...
    'HorizontalAlignment','center');

%% Console summary
fprintf('=== Crank-and-Slotted-Arm LVAD — Teardrop Slot ===\n');
if lv_fast
    fprintf('  LV assignment:            fast stroke (phi~0°, high QR torque)\n');
else
    fprintf('  LV assignment:            slow stroke (phi~180°, lower peak torque)\n');
end
fprintf('  r2 = %.2f mm  (%.0f%% of x)  |  r1 = %.2f mm (pin, floor)  |  r_eff = %.2f mm\n', r2, 100*r2/x, r1, r_eff);
fprintf('  Max paddle angle:         +/-%.2f deg  (r2=r1: +/-%.2f)\n', alpha, alpha_base);
fprintf('  Crank angle at max:       %.1f deg  (r2=r1: %.1f)\n', phi_pk, phi_pk_base);
fprintf('  Quick-return ratio:       %.3f  (r2=r1: %.3f)\n', qr_ratio, qr_base);
fprintf('  Overlap b:                %.2f  |  gamma = %.1f deg\n', b, gamma);
fprintf('  ─────────────────────────────────────\n');
Di_slot  = 2*x - r_eff;
Tx_slot  = r2 * sqrt(max(Di_slot^2 - r_eff^2, 0)) / Di_slot;   % actual wall half-width (not r_eff) in body X'
fprintf('  Paddle L = %g mm  |  w = %g mm  |  t = %g mm\n', L, w, t_paddle);
fprintf('  Slot half-width Tx:       %.2f mm  (min t_paddle for clearance = %.1f mm)\n', Tx_slot, 2*Tx_slot);
fprintf('  Contact from tip:         %.0f mm  (r: %.0f to %g mm)\n', L_contact, L-L_contact, L);
fprintf('  K_geom:                   %.0f mm3/rad\n', K_geom);
fprintf('  Paddle displaced volume:  %.1f mL  (w*t_paddle*L_contact, static)\n', V_paddle);
fprintf('  Stroke volume/ventricle:  %.1f mL  (%.1f mL before t_paddle)\n', SV, SV_gross);
fprintf('  Peak LV flow:             %.1f mL/s\n', max(Q_LV));
fprintf('  Peak RV flow:             %.1f mL/s\n', max(Q_RV));
fprintf('  Combined cardiac output:  %.2f L/min\n', CO);
fprintf('  ─────────────────────────────────────\n');
fprintf('  LV pressure:              %g mmHg (%.1f kPa)\n', p_LV_mmHg, p_LV/1e3);
fprintf('  RV pressure:              %g mmHg (%.1f kPa)\n', p_RV_mmHg, p_RV/1e3);
fprintf('  F_total (LV):             %.1f N  (%.1f N pressure + %.1f N elastic)\n', F_total_LV, p_LV*A_contact, F_e);
fprintf('  F_total (RV):             %.1f N  (%.1f N pressure + %.1f N elastic)\n', F_total_RV, p_RV*A_contact, F_e);
fprintf('  Moment arm:               %.1f mm\n', r_moment*1000);
fprintf('  Peak paddle torque T_p (LV): %.4f N·m\n', Tp_mag_LV);
fprintf('  Peak paddle torque T_p (RV): %.4f N·m\n', Tp_mag_RV);
fprintf('  Peak T_g (LV):           %.4f N·m  (r2=r1: %.4f)\n', max(Tg_LV), peak_fast_base*Tp_mag_LV);
fprintf('  Peak T_g (RV):           %.4f N·m  (r2=r1: %.4f)\n', max(Tg_RV), peak_slow_base*Tp_mag_RV);
fprintf('  T_g ratio LV/RV:         %.3f  (r2=r1: %.3f)\n', max(Tg_LV)/max(Tg_RV), qr_base);
fprintf('  ─────────────────────────────────────\n');
fprintf('  GR = %g  |  e_gb = %.2f  |  e_mech = %.2f  |  e_motor = %.2f\n', GR, e_gb, e_mech, e_motor);
fprintf('  omega_gb:                %.2f rad/s  (%g rpm crank)\n', omega_gb, rpm_max);
fprintf('  omega_motor:             %.1f rad/s  (%.0f rpm motor)\n', omega_motor, omega_motor*60/(2*pi));
fprintf('  Peak motor torque T_m:   %.5f N·m\n', max(Tm));
fprintf('  Peak P_mech:             %.3f W\n', max(P_mech));
fprintf('  Peak P_elec:             %.3f W\n', max(P_elec));
fprintf('  Mean P_elec:             %.3f W\n', mean(P_elec));
fprintf('  Cycle period T:           %.2f ms\n', T*1000);

%% ================================================================
function theta = teardrop_theta(phi_deg, x, a, r_arc)
% theta(phi) for the teardrop-slot mechanism (vectorised).
% r_arc=0 recovers the straight-slot formula atand(x*sind / (a - x*cosd)).
% r_arc is the pin CENTRE's effective arc radius (r_eff = r2 - r1), not the
% wall's own radius r2 -- see header comment.
    theta_o = atand(x*sind(phi_deg) ./ (a - x*cosd(phi_deg)));
    if r_arc < 1e-9
        theta = theta_o;  return;
    end
    y  = a - x;
    ci = y + r_arc;
    Di = 2*x - r_arc;
    if Di <= 0
        theta = theta_o;  return;
    end
    Tx  = r_arc * sqrt(max(Di^2 - r_arc^2, 0)) / Di;
    Ty  = ci + r_arc^2 / Di;
    R_T = hypot(Tx, Ty);

    R    = sqrt(a^2 + x^2 - 2*a*x*cosd(phi_deg));
    beta = zeros(size(phi_deg));
    sgn  = ones(size(phi_deg));
    sgn(phi_deg > 180) = -1;

    % Arc branch: pin centre rides the r_arc arc (R <= R_T, near phi=0/360)
    arc = R <= R_T;
    if any(arc)
        Yp = (R(arc).^2 - r_arc^2 + ci^2) / (2*ci);
        Xp = sqrt(max(R(arc).^2 - Yp.^2, 0));
        beta(arc) = atan2d(sgn(arc).*Xp, Yp);
    end

    % Tangent-line branch: pin centre rides the straight path (R > R_T)
    lin = ~arc;
    if any(lin)
        Ay  = a + x;
        Ac  = Tx^2 + (Ay-Ty)^2;
        Bc  = -2*Tx^2 + 2*Ty*(Ay-Ty);
        Cc  = R_T^2 - R(lin).^2;
        dsc = max(Bc^2 - 4*Ac*Cc, 0);
        t1  = (-Bc + sqrt(dsc)) / (2*Ac);
        t2  = (-Bc - sqrt(dsc)) / (2*Ac);
        t   = t1;
        bad = t < 0 | t > 1;
        d1  = max(0, t1-1) + max(0, -t1);   % unsigned distance from [0,1]
        d2  = max(0, t2-1) + max(0, -t2);
        t(bad & d2 < d1) = t2(bad & d2 < d1);  % use t2 only if it is closer to [0,1]
        t   = max(min(t, 1), 0);               % clamp residual float error (e.g. t=1+eps)
        Xp  = Tx*(1-t);
        Yp  = Ty + t.*(Ay-Ty);
        beta(lin) = atan2d(sgn(lin).*Xp, Yp);
    end

    theta = theta_o - beta;
end
