%% Crank-and-Slotted-Arm LVAD — Biventricular Paddle Dynamics (Teardrop Slot)
%
% Mirror of paddle_angle_plot.m for the teardrop-slot mechanism variant.
% The near-pivot end of the radial slot is replaced by a circular arc of
% radius r1, reducing the quick-return ratio.
%
% theta(phi) is computed numerically (no closed form for the teardrop).
% Ejection windows are found by numerical inversion of theta(phi).
% All torque/powpower er formulas are otherwise identical to paddle_angle_plot.m.
%
% KEY PARAMETERS TO TUNE:
%   r1        — teardrop arc radius (mm); 0 → straight slot, must be < x
%   b         — geometric bag overlap [0, 0.5]
%   L, w, L_contact — paddle geometry (mm)

clear; clc; close all;

%% Mechanism parameters
x   = 9.9;    % Crank arm length, mm
a   = 28;     % Crank centre to fulcrum distance, mm
r1  = 1.98;   % Teardrop arc radius, mm  (= 0.2x; set 0 for straight slot)

rpm_max  = 145;
T        = 60 / rpm_max;
omega_gb = 2*pi / T;

b = 0.5;   % Bag overlap [0, 0.5]: fill at theta=0 is (1-b)*100%

%% Paddle geometry — tune these
L         = 40;    % Paddle length (radial extent from pivot), mm
w         = 80;    % Paddle width (perpendicular to arm), mm
L_contact = 35;    % Contact length from tip of paddle, mm

K_geom = w * (L^2 - (L - L_contact)^2) / 2;   % dV/dtheta, mm³/rad

%% Torque / power chain
p_bag   = 16e3;   % Blood bag pressure, Pa
F_e     = 10;     % Bag elasticity force, N
GR      = 62;     % Gear ratio
e_gb    = 0.90;   % Gearbox efficiency
e_mech  = 0.72;   % Mechanical efficiency
e_motor = 0.83;   % Motor efficiency

%% Fine phi sweep (one cycle) — basis for all kinematics
phi_fine        = linspace(0, 360, 1441);   % 0.25 deg resolution
theta_fine      = teardrop_theta(phi_fine, x, a, r1);
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
SV = K_geom * alpha * pi/180 / (1000 * (1-b));   % mL per ventricle
CO = 2 * SV * rpm_max / 1000;                    % L/min combined

%% Time domain — 2 cycles
N         = 2000;
t         = linspace(0, 2*T, N);
phi_cont  = omega_gb * t;                          % rad, continuous
phi_deg_t = mod(phi_cont * 180/pi, 360);           % wrapped [0, 360)

theta_t       = interp1(phi_fine, theta_fine,    phi_deg_t);
dth_dphi_t    = interp1(phi_fine, dth_dphi_fine, phi_deg_t);  % dimensionless

dtheta_dt_rad = dth_dphi_t * omega_gb;            % rad/s  (dth_dphi_t is dimensionless)
dtheta_dt     = dtheta_dt_rad * (180/pi);         % °/s

% Ejection masks
lv_mask = (phi_deg_t >= phi_LV_start) | (phi_deg_t <= phi_pk);
rv_mask = (phi_deg_t >= phi_RV_start) & (phi_deg_t <= 360 - phi_pk);

% Flow rate
Q_LV    = max(0,  dtheta_dt_rad) .* double(lv_mask) * K_geom / 1000;   % mL/s
Q_RV    = max(0, -dtheta_dt_rad) .* double(rv_mask) * K_geom / 1000;
Q_total = Q_LV + Q_RV;

% Paddle torque
A_contact = w * L_contact * 1e-6;           % m²
F_total   = p_bag * A_contact + F_e;        % N
r_moment  = (L - L_contact/2) * 1e-3;      % m
Tp_mag    = F_total * r_moment;             % N·m

Tp_LV    = Tp_mag * double(lv_mask);
Tp_RV    = Tp_mag * double(rv_mask);
Tp_total = Tp_LV - Tp_RV;

% Gearbox torque: T_g = T_p × dθ/dφ  (virtual work, dimensionless ratio)
dth_dphi_dless =  dtheta_dt_rad / omega_gb;
Tg_LV =  Tp_mag .* dth_dphi_dless .* double(lv_mask);
Tg_RV = -Tp_mag .* dth_dphi_dless .* double(rv_mask);
Tg    = Tg_LV + Tg_RV;

% Motor torque & power
Tm          = Tg / (GR * e_gb * e_mech);
omega_motor = omega_gb * GR;
P_mech      = Tm .* omega_motor;
P_elec      = P_mech / e_motor;

%% Ejection shading segments (ms)
d_LV = phi_LV_start / 360;
d_RV = phi_RV_start / 360;
lv_segs = [0,          T * phi_pk/360;
           T * d_LV,   T + T * phi_pk/360;
           T + T*d_LV, 2*T              ] * 1000;
rv_segs = [T * d_RV,   T * (360 - phi_pk)/360;
           T + T*d_RV, T + T*(360 - phi_pk)/360] * 1000;

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
    sprintf('x = %g mm  |  a = %g mm  |  r1 = %g mm', x, a, r1),
    sprintf('\\phi_{peak} = %.1f°  (straight slot: %.1f°)', phi_pk, acosd(x/a)),
    sprintf('\\alpha = %.1f°  (straight slot: %.1f°)', alpha, asind(x/a)),
    sprintf('QR ratio = %.3f  (straight: %.3f)', qr_ratio, (a+x)/(a-x)),
    sprintf('Overlap b = %.2f  |  \\gamma = %.1f°', b, gamma),
    sprintf('─────────────────────'),
    sprintf('L = %g mm  |  w = %g mm', L, w),
    sprintf('L_{contact} = %g mm from tip', L_contact),
    sprintf('SV = %.1f mL / ventricle', SV),
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
plot(t*1000, F_total * double(lv_mask),'b-','LineWidth',2);
plot(t*1000, F_total * double(rv_mask),'r-','LineWidth',2);
yline(F_total,'k:','LineWidth',0.8,'Label',sprintf('F_{total} = %.1f N',F_total),'LabelHorizontalAlignment','left');
yline(0,'k:','LineWidth',0.8);
xline(T*1000,  'k--','LineWidth',1,'Label','Cycle 2','LabelVerticalAlignment','bottom');
xline(2*T*1000,'k--','LineWidth',1,'Label','End',    'LabelVerticalAlignment','bottom');
hold off;
ylabel('F_{total} (N)');
title('F_{total} Applied to Paddle vs Time');
legend({'LV','RV'},'Location','northeast');
grid on; xlim([0 2*T*1000]); ylim([0, F_total*1.4]);
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
    sprintf('p_{bag} = %g kPa  |  F_e = %g N', p_bag/1e3, F_e),
    sprintf('A_{contact} = %.0f mm²', w*L_contact),
    sprintf('F_{total} = %.1f N', F_total),
    sprintf('r_{moment} = %.0f mm', r_moment*1000),
    sprintf('T_{p} = %.3f N·m', Tp_mag),
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

xregion(0,            phi_pk,      'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_LV_start, 360,         'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_RV_start, 360-phi_pk,  'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);

plot(phi_fine, theta_fine,'k-','LineWidth',2);

yline( alpha,'b--','LineWidth',1.2,'Label',sprintf('+\\alpha = %.1f°',alpha),'LabelHorizontalAlignment','left');
yline(-alpha,'r--','LineWidth',1.2,'Label',sprintf('-\\alpha = %.1f°',-alpha),'LabelHorizontalAlignment','left');
yline(0,'k:','LineWidth',0.8);

xline(0,          'k--','LineWidth',1,'Label','0°',                         'LabelVerticalAlignment','bottom');
xline(phi_pk,     'b:' ,'LineWidth',1,'Label',sprintf('%.1f°',phi_pk),      'LabelVerticalAlignment','bottom');
xline(180,        'k--','LineWidth',1,'Label','180°',                        'LabelVerticalAlignment','bottom');
xline(360-phi_pk, 'r:' ,'LineWidth',1,'Label',sprintf('%.1f°',360-phi_pk),  'LabelVerticalAlignment','bottom');
xline(360,        'k--','LineWidth',1,'Label','360°',                        'LabelVerticalAlignment','bottom');

xline(phi_RV_start,'r--','LineWidth',1,'Label',sprintf('RV start %.1f°',phi_RV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');
xline(phi_LV_start,'b--','LineWidth',1,'Label',sprintf('LV start %.1f°',phi_LV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');

plot(phi_pk,       alpha,'bs','MarkerFaceColor','b','MarkerSize',8);
plot(360-phi_pk,  -alpha,'rs','MarkerFaceColor','r','MarkerSize',8);

hold off;
xlabel('Crank Angle (°)'); ylabel('Paddle Angle (°)');
title('Crank Angle vs Paddle Angle — Teardrop Slot Kinematics','FontSize',12);
legend({'LV ejection','RV ejection','Paddle angle'},'Location','northeast');
grid on; xlim([0 360]); ylim([-alpha*1.3, alpha*1.3]);
xticks(0:45:360);

%% Console summary
fprintf('=== Crank-and-Slotted-Arm LVAD — Teardrop Slot ===\n');
fprintf('  r1 = %.2f mm  (%.0f%% of x)\n', r1, 100*r1/x);
fprintf('  Max paddle angle:         +/-%.2f deg  (straight slot: +/-%.2f)\n', alpha, asind(x/a));
fprintf('  Crank angle at max:       %.1f deg  (straight slot: %.1f)\n', phi_pk, acosd(x/a));
fprintf('  Quick-return ratio:       %.3f  (straight slot: %.3f)\n', qr_ratio, (a+x)/(a-x));
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
fprintf('  F_total:                  %.1f N  (%.1f N pressure + %.1f N elastic)\n', F_total, p_bag*A_contact, F_e);
fprintf('  Moment arm:               %.1f mm\n', r_moment*1000);
fprintf('  Peak paddle torque T_p:   %.4f N·m\n', Tp_mag);
fprintf('  Peak T_g (LV):           %.4f N·m  (straight slot: %.4f)\n', max(Tg_LV), x/(a-x)*Tp_mag);
fprintf('  Peak T_g (RV):           %.4f N·m  (straight slot: %.4f)\n', max(Tg_RV), x/(a+x)*Tp_mag);
fprintf('  T_g ratio LV/RV:         %.3f  (straight slot: %.3f)\n', max(Tg_LV)/max(Tg_RV), (a+x)/(a-x));
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
function theta = teardrop_theta(phi_deg, x, a, r1)
% theta(phi) for the teardrop-slot mechanism (vectorised).
% r1=0 recovers the straight-slot formula atand(x*sind / (a - x*cosd)).
    theta_o = atand(x*sind(phi_deg) ./ (a - x*cosd(phi_deg)));
    if r1 < 1e-9
        theta = theta_o;  return;
    end
    y  = a - x;
    ci = y + r1;
    Di = 2*x - r1;
    if Di <= 0
        theta = theta_o;  return;
    end
    Tx  = r1 * sqrt(max(Di^2 - r1^2, 0)) / Di;
    Ty  = ci + r1^2 / Di;
    R_T = hypot(Tx, Ty);

    R    = sqrt(a^2 + x^2 - 2*a*x*cosd(phi_deg));
    beta = zeros(size(phi_deg));
    sgn  = ones(size(phi_deg));
    sgn(phi_deg > 180) = -1;

    % Arc branch: pin rides the r1 arc (R <= R_T, near phi=0/360)
    arc = R <= R_T;
    if any(arc)
        Yp = (R(arc).^2 - r1^2 + ci^2) / (2*ci);
        Xp = sqrt(max(R(arc).^2 - Yp.^2, 0));
        beta(arc) = atan2d(sgn(arc).*Xp, Yp);
    end

    % Tangent-line branch: pin rides the straight slot wall (R > R_T)
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
