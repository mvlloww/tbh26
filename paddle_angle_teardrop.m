%% Crank-and-Slotted-Arm LVAD — Teardrop Slot Variant (kinematics only)
%
% MECHANISM CHANGE vs paddle_angle_plot.m:
%   The near-pivot end of the radial slot is no longer a sharp point on the
%   centreline — it's a teardrop: a circular arc of radius r1, tangent to
%   the two straight sides, replaces the end near phi=0/360.
%
%   Geometry (centreline = original slot axis through O4):
%     y = a - x              % bottom of r1-circle, distance from O4
%     c = y + r1              % r1-circle centre, distance from O4
%     apex (z = 2x) sits at distance (a+x) from O4
%     Motor centre O2 is at the midpoint of the teardrop span (y+x = a),
%     so theta(0) = theta(180) = 0, same reference as the straight slot.
%
%   For R(phi) = |O4P| = sqrt(a^2+x^2-2*a*x*cosd(phi))  (unchanged):
%     R <= R_T  -> P on the r1 arc      -> theta = theta_orig - beta(arc)
%     R >  R_T  -> P on the tangent line -> theta = theta_orig - beta(tan)
%   beta = atan2(X',Y') of P in the body frame (X'=0 along the centreline).
%   R_T  = |O4| to the tangent point between the r1-circle and the line
%   from the apex.
%
%   r1 in (0,x):  r1 -> 0 recovers the original straight-slot theta(phi).
%                 r1 -> x degenerates to a full circle (no straight side).

clear; clc; close all;

%% Mechanism parameters (unchanged from paddle_angle_plot.m)
x = 7.6;     % Crank arm length, mm
a = 22.4;    % Crank centre to fulcrum distance, mm
r = x/a;
alpha  = asind(r);
phi_pk = acosd(r);

phi_deg    = linspace(0,360,1441);   % 0.25 deg resolution
theta_orig = atand(x*sind(phi_deg) ./ (a - x*cosd(phi_deg)));
dth_orig   = gradient(theta_orig, phi_deg);

%% r1 sweep (fraction of x; must be in (0,x))
r1_vals = [0.2 0.4 0.6 0.8] * x;

theta_sweep = zeros(numel(r1_vals), numel(phi_deg));
alpha_new   = zeros(size(r1_vals));
phipk_new   = zeros(size(r1_vals));
qr_new      = zeros(size(r1_vals));

for i = 1:numel(r1_vals)
    theta_i = teardrop_theta(phi_deg, x, a, r1_vals(i));
    theta_sweep(i,:) = theta_i;

    [amax,imax]  = max(theta_i);
    alpha_new(i) = amax;
    phipk_new(i) = phi_deg(imax);

    dth = gradient(theta_i, phi_deg);
    qr_new(i) = max(dth) / max(-dth);   % peak fwd rate / peak return rate
end

qr_orig = max(dth_orig) / max(-dth_orig);

%% Bag overlap & flow-rate parameters (unchanged from paddle_angle_plot.m)
b       = 0.5;     % Bag overlap [0,0.5]: fill at theta=0 is (1-b)*100%
rpm_max = 145;
T       = 60 / rpm_max;
omega_gb = 2*pi / T;

L         = 31.5;
w         = 100;
L_contact = 30;
K_geom    = w * (L^2 - (L-L_contact)^2) / 2;

%% Ejection windows & stroke volume — original
gamma_orig = alpha * b / (1-b);
[phi_LV_start_orig, phi_RV_start_orig] = ejection_windows(phi_deg, theta_orig, phi_pk, gamma_orig);
SV_orig = K_geom * alpha * pi/180 / (1000*(1-b));
CO_orig = 2 * SV_orig * rpm_max / 1000;

%% Ejection windows & stroke volume — teardrop sweep
phi_LV_start = zeros(size(r1_vals));
phi_RV_start = zeros(size(r1_vals));
SV_new       = zeros(size(r1_vals));
CO_new       = zeros(size(r1_vals));
for i = 1:numel(r1_vals)
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
cmap = lines(numel(r1_vals));
for i = 1:numel(r1_vals)
    plot(phi_deg, theta_sweep(i,:),'--','Color',cmap(i,:),'LineWidth',1.5);
end
yline(0,'k:'); xline(0,'k--'); xline(180,'k--'); xline(360,'k--');
xlabel('Crank Angle \phi (deg)'); ylabel('Paddle Angle \theta (deg)');
title('\theta(\phi): straight slot vs teardrop (r1 sweep)');
legend([{'original'}, arrayfun(@(v) sprintf('r1=%.2f mm',v), r1_vals,'UniformOutput',false)], ...
    'Location','south','NumColumns',3);
grid on; xlim([0 360]); xticks(0:45:360);

subplot(2,1,2); hold on;
plot(phi_deg, dth_orig,'k-','LineWidth',2);
for i = 1:numel(r1_vals)
    plot(phi_deg, gradient(theta_sweep(i,:),phi_deg),'--','Color',cmap(i,:),'LineWidth',1.5);
end
yline(0,'k:');
xlabel('Crank Angle \phi (deg)'); ylabel('d\theta/d\phi (deg/deg)');
title('Angular rate vs crank angle');
grid on; xlim([0 360]); xticks(0:45:360);

%% Figure: Flow rate Q_LV/Q_RV — original vs teardrop sweep, one cycle
t     = linspace(0, T, 1000);
phi_t = mod(omega_gb*t*180/pi, 360);

[Q_LV_o, Q_RV_o] = flow_rate(phi_deg, theta_orig, phi_pk, phi_LV_start_orig, phi_RV_start_orig, omega_gb, K_geom, phi_t);

Q_LV_sweep = zeros(numel(r1_vals), numel(t));
Q_RV_sweep = zeros(numel(r1_vals), numel(t));
for i = 1:numel(r1_vals)
    [Q_LV_sweep(i,:), Q_RV_sweep(i,:)] = ...
        flow_rate(phi_deg, theta_sweep(i,:), phipk_new(i), phi_LV_start(i), phi_RV_start(i), omega_gb, K_geom, phi_t);
end

figure('Name','Teardrop Slot — Flow Rate','Color','w','Position',[120 120 900 650]);

subplot(2,1,1); hold on;
plot(t*1000, Q_LV_o,'k-','LineWidth',2);
for i = 1:numel(r1_vals)
    plot(t*1000, Q_LV_sweep(i,:),'--','Color',cmap(i,:),'LineWidth',1.5);
end
ylabel('Q_{LV} (mL/s)'); title('LV Flow Rate — one cycle');
legend([{'original'}, arrayfun(@(v) sprintf('r1=%.2f mm',v), r1_vals,'UniformOutput',false)], ...
    'Location','best');
grid on; xlim([0 T*1000]);

subplot(2,1,2); hold on;
plot(t*1000, Q_RV_o,'k-','LineWidth',2);
for i = 1:numel(r1_vals)
    plot(t*1000, Q_RV_sweep(i,:),'--','Color',cmap(i,:),'LineWidth',1.5);
end
xlabel('Time (ms)'); ylabel('Q_{RV} (mL/s)'); title('RV Flow Rate — one cycle');
grid on; xlim([0 T*1000]);

%% Console summary
fprintf('=== Teardrop slot sweep (x=%.2f mm, a=%.2f mm, y=a-x=%.2f mm, b=%.2f) ===\n', x, a, a-x, b);
fprintf('  Original:  alpha=%.2f deg @ phi=%.1f  |  QR ratio=%.2f  |  SV=%.2f mL  |  CO=%.2f L/min\n', ...
    alpha, phi_pk, qr_orig, SV_orig, CO_orig);
for i = 1:numel(r1_vals)
    fprintf('  r1=%.2f mm: alpha=%.2f deg @ phi=%.1f  |  QR ratio=%.2f  |  SV=%.2f mL  |  CO=%.2f L/min\n', ...
        r1_vals(i), alpha_new(i), phipk_new(i), qr_new(i), SV_new(i), CO_new(i));
end

%% ================================================================
function theta = teardrop_theta(phi_deg, x, a, r1)
% theta(phi) for the teardrop-slot mechanism.
%   y = a - x (motor centre at the midpoint of the teardrop span z=2x),
%   so theta(0) = theta(180) = 0, as in the straight-slot design.
    y = a - x;
    c = y + r1;             % r1-circle centre, distance from O4
    D = 2*x - r1;           % apex-to-circle-centre distance (apex at a+x)

    Tx  = r1 * sqrt(D^2 - r1^2) / D;   % tangent point (right side), X'
    Ty  = c + r1^2/D;                  %                              Y'
    R_T = hypot(Tx,Ty);

    R          = sqrt(a^2 + x^2 - 2*a*x*cosd(phi_deg));
    theta_orig = atand(x*sind(phi_deg) ./ (a - x*cosd(phi_deg)));

    beta = zeros(size(phi_deg));
    sgn  = ones(size(phi_deg));
    sgn(phi_deg > 180) = -1;

    % --- Arc branch: R <= R_T (near phi=0/360) ---
    m  = R <= R_T;
    Yp = (R(m).^2 - r1^2 + c^2) / (2*c);
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
    t  = t1;  t(t<0 | t>1) = t2(t<0 | t>1);
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

function [Q_LV, Q_RV] = flow_rate(phi_deg, theta, phipk_i, phi_LV_start, phi_RV_start, omega_gb, K_geom, phi_t)
    dth   = gradient(theta, phi_deg);          % dtheta/dphi, dimensionless
    dth_t = interp1(phi_deg, dth, phi_t);
    dtheta_dt = omega_gb * dth_t;              % rad/s

    lv_mask = (phi_t >= phi_LV_start) | (phi_t <= phipk_i);
    rv_mask = (phi_t >= phi_RV_start) & (phi_t <= 360-phipk_i);

    Q_LV = max(0,  dtheta_dt) .* lv_mask * K_geom / 1000;
    Q_RV = max(0, -dtheta_dt) .* rv_mask * K_geom / 1000;
end
