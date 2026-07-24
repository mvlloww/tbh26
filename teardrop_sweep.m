%% Crank-and-Slotted-Arm LVAD — Teardrop Slot Variant (kinematics only)
%
% MECHANISM CHANGE vs paddle_angle_plot.m:
%   The near-pivot end of the radial slot is no longer a sharp point on the
%   centreline — it's a teardrop: a circular arc, tangent to the two
%   straight sides, replaces the end near phi=0/360.
%
%   r1 = crank pin radius (fixed). The pin's CENTRE doesn't ride on the
%   wall itself -- it stays a constant distance r1 inside it. r2 = teardrop
%   (wall) radius, the design/sweep variable. Kinematics (theta(phi)) are
%   driven by r_eff = r2 - r1 (the pin centre's own path), not r2 directly.
%   r2 = r1 gives r_eff = 0 -- zero teardrop effect, identical to the pure
%   straight slot.
%
%   Geometry (centreline = original slot axis through O4; in terms of the
%   kinematic r_eff):
%     y = a - x              % bottom of r_eff-circle, distance from O4
%     c = y + r_eff            % r_eff-circle centre, distance from O4
%     apex (z = 2x) sits at distance (a+x) from O4
%     Motor centre O2 is at the midpoint of the teardrop span (y+x = a),
%     so theta(0) = theta(180) = 0, same reference as the straight slot.
%
%   For R(phi) = |O4P| = sqrt(a^2+x^2-2*a*x*cosd(phi))  (unchanged):
%     R <= R_T  -> P on the r_eff arc     -> theta = theta_orig - beta(arc)
%     R >  R_T  -> P on the tangent line -> theta = theta_orig - beta(tan)
%   beta = atan2(X',Y') of P in the body frame (X'=0 along the centreline).
%   R_T  = |O4| to the tangent point between the r_eff-circle and the line
%   from the apex.
%
%   r_eff in (0,x):  r_eff -> 0 recovers the original straight-slot theta(phi).
%                    r_eff -> x degenerates to a full circle (no straight side).
%
%   The drawn/machined WALL (not the kinematics) is the r_eff-arc geometry
%   scaled outward by k=r2/r_eff about the arc centre -- concentric r2 arc,
%   apex pushed out past a+x, far tip capped with its own r1 fillet (a
%   finite pin can never get its centre all the way into a sharp point).

clear; clc; close all;

%% Mechanism parameters (unchanged from paddle_angle_plot.m)
x = 9.9;     % Crank arm length, mm
a = 28;    % Crank centre to fulcrum distance, mm
r = x/a;
alpha  = asind(r);
phi_pk = acosd(r);

phi_deg    = linspace(0,360,1441);   % 0.25 deg resolution
theta_orig = atand(x*sind(phi_deg) ./ (a - x*cosd(phi_deg)));
dth_orig   = gradient(theta_orig, phi_deg);

%% r_eff sweep (fraction of x; must be in (0,x)) — pin radius is fixed
r1         = 2;                        % Crank pin radius, mm (fixed)
r_eff_vals = [0.2 0.4 0.6 0.8] * x;    % pin-centre's effective arc radius (drives kinematics)
r2_vals    = r_eff_vals + r1;          % corresponding teardrop (wall) radius

theta_sweep = zeros(numel(r_eff_vals), numel(phi_deg));
alpha_new   = zeros(size(r_eff_vals));
phipk_new   = zeros(size(r_eff_vals));
qr_new      = zeros(size(r_eff_vals));

for i = 1:numel(r_eff_vals)
    theta_i = teardrop_theta(phi_deg, x, a, r_eff_vals(i));
    theta_sweep(i,:) = theta_i;

    [amax,imax]  = max(theta_i);
    alpha_new(i) = amax;
    phipk_new(i) = phi_deg(imax);

    dth = gradient(theta_i, phi_deg);
    qr_new(i) = max(dth) / max(-dth);   % peak fwd rate / peak return rate
end

qr_orig = max(dth_orig) / max(-dth_orig);

%% Bag overlap & flow-rate parameters (unchanged from paddle_angle_plot.m)
b        = 0.5;     % Bag overlap [0,0.5]: fill at theta=0 is (1-b)*100%
lv_fast  = true;    % true → LV on quick-return stroke; false → LV on slow stroke
rpm_max  = 145;
T       = 60 / rpm_max;
omega_gb = 2*pi / T;

L         = 40;
w         = 80;    % out of plane
t_paddle  = 4;     % perpendicular to arm in mechanism plane
L_contact = 35;
K_geom    = w * (L^2 - (L-L_contact)^2) / 2;

%% Ejection windows & stroke volume — original
gamma_orig = alpha * b / (1-b);
[phi_LV_start_orig, phi_RV_start_orig] = ejection_windows(phi_deg, theta_orig, phi_pk, gamma_orig);
SV_orig = K_geom * alpha * pi/180 / (1000*(1-b));
CO_orig = 2 * SV_orig * rpm_max / 1000;

%% Ejection windows & stroke volume — teardrop sweep
phi_LV_start = zeros(size(r_eff_vals));
phi_RV_start = zeros(size(r_eff_vals));
SV_new       = zeros(size(r_eff_vals));
CO_new       = zeros(size(r_eff_vals));
for i = 1:numel(r_eff_vals)
    gamma_i = alpha_new(i) * b / (1-b);
    [phi_LV_start(i), phi_RV_start(i)] = ...
        ejection_windows(phi_deg, theta_sweep(i,:), phipk_new(i), gamma_i);
    SV_new(i) = K_geom * alpha_new(i) * pi/180 / (1000*(1-b));
    CO_new(i) = 2 * SV_new(i) * rpm_max / 1000;
end

%% Figure: theta(phi) and dtheta/dphi — original vs teardrop sweep
figure('Name','Teardrop Slot — Kinematics','Color','w','Position',[80 80 900 650]);

subplot(2,1,1); hold on;
plot(phi_deg, theta_orig,'k-','LineWidth',2);
cmap = lines(numel(r_eff_vals));
for i = 1:numel(r_eff_vals)
    plot(phi_deg, theta_sweep(i,:),'--','Color',cmap(i,:),'LineWidth',1.5);
end
yline(0,'k:'); xline(0,'k--'); xline(180,'k--'); xline(360,'k--');
xlabel('Crank Angle \phi (deg)'); ylabel('Paddle Angle \theta (deg)');
title('\theta(\phi): straight slot vs teardrop (r2 sweep)');
legend([{'original'}, arrayfun(@(v) sprintf('r2=%.2f mm',v), r2_vals,'UniformOutput',false)], ...
    'Location','south','NumColumns',3);
grid on; xlim([0 360]); xticks(0:45:360);

subplot(2,1,2); hold on;
plot(phi_deg, dth_orig,'k-','LineWidth',2);
for i = 1:numel(r_eff_vals)
    plot(phi_deg, gradient(theta_sweep(i,:),phi_deg),'--','Color',cmap(i,:),'LineWidth',1.5);
end
yline(0,'k:');
xlabel('Crank Angle \phi (deg)'); ylabel('d\theta/d\phi (deg/deg)');
title('Angular rate vs crank angle');
grid on; xlim([0 360]); xticks(0:45:360);

%% Figure: Flow rate Q_LV/Q_RV — original vs teardrop sweep, one cycle
t     = linspace(0, T, 1000);
phi_t = mod(omega_gb*t*180/pi, 360);

[Q_LV_o, Q_RV_o] = flow_rate(phi_deg, theta_orig, phi_pk, phi_LV_start_orig, phi_RV_start_orig, omega_gb, K_geom, phi_t, lv_fast);

Q_LV_sweep = zeros(numel(r_eff_vals), numel(t));
Q_RV_sweep = zeros(numel(r_eff_vals), numel(t));
for i = 1:numel(r_eff_vals)
    [Q_LV_sweep(i,:), Q_RV_sweep(i,:)] = ...
        flow_rate(phi_deg, theta_sweep(i,:), phipk_new(i), phi_LV_start(i), phi_RV_start(i), omega_gb, K_geom, phi_t, lv_fast);
end

figure('Name','Teardrop Slot — Flow Rate','Color','w','Position',[120 120 900 650]);

subplot(2,1,1); hold on;
plot(t*1000, Q_LV_o,'k-','LineWidth',2);
for i = 1:numel(r_eff_vals)
    plot(t*1000, Q_LV_sweep(i,:),'--','Color',cmap(i,:),'LineWidth',1.5);
end
ylabel('Q_{LV} (mL/s)'); title('LV Flow Rate — one cycle');
legend([{'original'}, arrayfun(@(v) sprintf('r2=%.2f mm',v), r2_vals,'UniformOutput',false)], ...
    'Location','best');
grid on; xlim([0 T*1000]);

subplot(2,1,2); hold on;
plot(t*1000, Q_RV_o,'k-','LineWidth',2);
for i = 1:numel(r_eff_vals)
    plot(t*1000, Q_RV_sweep(i,:),'--','Color',cmap(i,:),'LineWidth',1.5);
end
xlabel('Time (ms)'); ylabel('Q_{RV} (mL/s)'); title('RV Flow Rate — one cycle');
grid on; xlim([0 T*1000]);

%% Figure: Slot geometry (body frame) — the ACTUAL wall, not the pin-centre path
% Plots the teardrop wall in the arm's own frame: X' (perpendicular to arm)
% vs Y' (along arm from pivot O4). The wall is the pin-centre path (radius
% r_eff, apex at a+x) scaled outward by k=r2/r_eff about the arc centre, so
% the pin (radius r1) rides inside it -- concentric r2 arc, apex pushed out
% past a+x, far tip capped with its own r1 fillet (see header comment).
figure('Name','Teardrop Slot — Slot Geometry','Color','w','Position',[160 160 420 600]);
hold on;

% Original straight slot: centreline from y=a-x to apex at a+x
plot([0 0], [a-x, a+x], 'k-', 'LineWidth', 2);

for i = 1:numel(r_eff_vals)
    r_eff_i = r_eff_vals(i);
    r2_i    = r2_vals(i);

    ci  = (a - x) + r_eff_i;                       % r_eff circle centre along Y'
    Di  = 2*x - r_eff_i;                            % apex-to-circle-centre distance
    Txi = r_eff_i * sqrt(Di^2 - r_eff_i^2) / Di;    % pin-centre tangent point X'
    Tyi = ci + r_eff_i^2 / Di;                      %                          Y'
    thR = atan2(Tyi - ci, Txi);                     % angle of right tangent from circle centre

    k      = r2_i / r_eff_i;              % pin-centre path -> actual wall
    Txw    = Txi * k;
    Tyw    = ci + (Tyi - ci) * k;
    apex_w = ci + (a + x - ci) * k;

    % Tip fillet: circle of radius r1, centred on the axis, tangent to both
    % wall lines (a finite pin can't reach the mathematically sharp apex)
    Yc  = apex_w - r1 * r2_i / (Tyw - ci);
    Txf = r1 * Txw / r2_i;
    Tyf = Yc + r1 * (Tyw - ci) / r2_i;

    aa_big = linspace(thR, -pi - thR, 300);   % near-pivot r2 arc
    aa_tip = linspace(pi - thR, thR, 60);     % far-tip r1 fillet

    % Outline: near-pivot arc -> (implicit line) -> tip fillet -> (implicit line, closes)
    outline_X = [r2_i*cos(aa_big),    r1*cos(aa_tip)   ];
    outline_Y = [ci+r2_i*sin(aa_big), Yc+r1*sin(aa_tip)];

    plot(outline_X, outline_Y, '--', 'Color', cmap(i,:), 'LineWidth', 1.5);
end

% Annotations
plot(0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
yline(a, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
text(x*0.15, 0.8,     'O_4 (pivot)',  'FontSize', 8);
text(x*0.15, a + 0.8, 'O_2 (motor)', 'FontSize', 8);

xlabel("X' — perpendicular to arm (mm)");
ylabel("Y' — along arm from pivot O_4 (mm)");
title('Slot wall geometry in body frame (r2 sweep, r1 pin fixed)');
legend([{'original'}, arrayfun(@(v) sprintf('r2=%.2f mm',v), r2_vals,'UniformOutput',false)], ...
    'Location','south','NumColumns',3);
grid on; axis equal;
xlim([-x*1.3, x*1.3]);
ylim([a - x - 3, a + x + 3]);

%% Console summary
fprintf('=== Teardrop slot sweep (x=%.2f mm, a=%.2f mm, b=%.2f, r1=%.2f mm pin) ===\n', x, a, b, r1);
fprintf('  Paddle L=%g mm  w=%g mm  t_paddle=%g mm  Lc=%g mm\n', L, w, t_paddle, L_contact);
fprintf('  Original:  alpha=%.2f deg @ phi=%.1f  |  QR=%.2f  |  SV=%.2f mL  |  CO=%.2f L/min\n', ...
    alpha, phi_pk, qr_orig, SV_orig, CO_orig);
for i = 1:numel(r_eff_vals)
    Di_i   = 2*x - r_eff_vals(i);
    Tx_i   = r2_vals(i) * sqrt(max(Di_i^2 - r_eff_vals(i)^2, 0)) / Di_i;   % actual wall half-width (not r_eff)
    warn_i = ''; if t_paddle < 2*Tx_i; warn_i = '  *** t TOO THIN ***'; end
    fprintf('  r2=%.2f mm (r_eff=%.2f): alpha=%.2f deg  QR=%.2f  CO=%.2f L/min  min_t=%.1f mm%s\n', ...
        r2_vals(i), r_eff_vals(i), alpha_new(i), qr_new(i), CO_new(i), 2*Tx_i, warn_i);
end

%% ================================================================
function theta = teardrop_theta(phi_deg, x, a, r_arc)
% theta(phi) for the teardrop-slot mechanism.
%   y = a - x (motor centre at the midpoint of the teardrop span z=2x),
%   so theta(0) = theta(180) = 0, as in the straight-slot design.
% r_arc is the pin CENTRE's effective arc radius (r_eff = r2 - r1), not the
% wall's own radius r2 -- see header comment.
    y = a - x;
    c = y + r_arc;             % r_arc-circle centre, distance from O4
    D = 2*x - r_arc;           % apex-to-circle-centre distance (apex at a+x)

    Tx  = r_arc * sqrt(D^2 - r_arc^2) / D;   % tangent point (right side), X'
    Ty  = c + r_arc^2/D;                     %                              Y'
    R_T = hypot(Tx,Ty);

    R          = sqrt(a^2 + x^2 - 2*a*x*cosd(phi_deg));
    theta_orig = atand(x*sind(phi_deg) ./ (a - x*cosd(phi_deg)));

    beta = zeros(size(phi_deg));
    sgn  = ones(size(phi_deg));
    sgn(phi_deg > 180) = -1;

    % --- Arc branch: R <= R_T (near phi=0/360) ---
    m  = R <= R_T;
    Yp = (R(m).^2 - r_arc^2 + c^2) / (2*c);
    Xp = sqrt(max(0, R(m).^2 - Yp.^2));
    beta(m) = atan2d(sgn(m).*Xp, Yp);

    % --- Tangent-line branch: R > R_T ---
    m  = ~m;
    Ay = a + x;
    A_coef = Tx^2 + (Ay-Ty)^2;
    B_coef = -2*Tx^2 + 2*Ty*(Ay-Ty);
    C_coef = R_T^2 - R(m).^2;
    disc   = max(0, B_coef^2 - 4*A_coef*C_coef);
    t1 = (-B_coef + sqrt(disc)) / (2*A_coef);
    t2 = (-B_coef - sqrt(disc)) / (2*A_coef);
    t   = t1;
    bad = t < 0 | t > 1;
    d1  = max(0, t1-1) + max(0, -t1);
    d2  = max(0, t2-1) + max(0, -t2);
    t(bad & d2 < d1) = t2(bad & d2 < d1);
    t   = max(min(t, 1), 0);
    Xp = Tx*(1-t);
    Yp = Ty + t.*(Ay-Ty);
    beta(m) = atan2d(sgn(m).*Xp, Yp);

    theta = theta_orig - beta;
end

%% ================================================================
function [phi_LV_start, phi_RV_start] = ejection_windows(phi_deg, theta, phipk_i, gamma_i)
% Crank angles where theta crosses +/- gamma during the increasing (LV)
% and decreasing (RV) halves of theta(phi). Generalises the closed-form
% phi_LV_start/phi_RV_start formulas to any theta(phi) shape.
    seg = phi_deg <= phipk_i;
    phi_a = solve_phi_for_theta(phi_deg(seg), theta(seg), gamma_i);
    phi_LV_start = 360 - phi_a;

    seg = (phi_deg >= phipk_i) & (phi_deg <= 180);
    phi_RV_start = solve_phi_for_theta(phi_deg(seg), theta(seg), gamma_i);
end

function phi_out = solve_phi_for_theta(phi_seg, theta_seg, theta_target)
    if theta_seg(1) > theta_seg(end)
        phi_seg   = fliplr(phi_seg);
        theta_seg = fliplr(theta_seg);
    end
    phi_out = interp1(theta_seg, phi_seg, theta_target);
end

function [Q_LV, Q_RV] = flow_rate(phi_deg, theta, phipk_i, phi_LV_start, phi_RV_start, omega_gb, K_geom, phi_t, lv_fast)
    dth   = gradient(theta, phi_deg);          % dtheta/dphi, dimensionless
    dth_t = interp1(phi_deg, dth, phi_t);
    dtheta_dt = omega_gb * dth_t;              % rad/s

    fast_mask = (phi_t >= phi_LV_start) | (phi_t <= phipk_i);
    slow_mask = (phi_t >= phi_RV_start) & (phi_t <= 360-phipk_i);

    if lv_fast
        lv_mask = fast_mask;  rv_mask = slow_mask;
        sign_lv = +1;         sign_rv = -1;
    else
        lv_mask = slow_mask;  rv_mask = fast_mask;
        sign_lv = -1;         sign_rv = +1;
    end

    Q_LV = max(0,  sign_lv * dtheta_dt) .* lv_mask * K_geom / 1000;
    Q_RV = max(0,  sign_rv * dtheta_dt) .* rv_mask * K_geom / 1000;
end
