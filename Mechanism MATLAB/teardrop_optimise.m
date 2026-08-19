%% Teardrop Paddle / Mechanism Geometry Optimiser
%
% Minimises  size_weight*(a + L) + lambda * kink
% subject to:  CO = CO_target,  alpha >= alpha_min,  P_elec_peak <= P_max,
%              QR_min <= QR ratio <= QR_max.
%
% Power is a hard constraint (budget), not the primary objective.
% The optimizer trades the power headroom for smaller mechanism dimensions.
%   size_weight — W/mm; raise to push harder for compact geometry
%   lambda      — W/(deg/deg kink); raise to prefer smooth power profiles
%
% Design vector v = [x, a, f, L, w, L_contact]
%   f = r2/x in [0.05, 0.9]: r2=f*x is the teardrop (wall) radius
%
% r1 = crank pin radius (fixed). The pin's CENTRE doesn't ride on the wall
% itself -- it stays a constant distance r1 inside it. So the kinematics
% (theta(phi)) are driven by the effective arc radius r_eff = r2 - r1 (the
% pin centre's own path), not r2 directly. r2 = r1 (smallest wall that can
% even contain the pin) gives r_eff = 0 -- i.e. zero teardrop effect,
% identical to the pure straight slot. Any real quick-return benefit
% requires r2 > r1 by more than a hair.

clear; clc;

%% Fixed parameters (match paddle_optimise.m)
rpm_max   = 145;
b         = 0.5;
p_LV_mmHg = 120;     % LV peak bag pressure, mmHg
p_RV_mmHg = 25;      % RV peak bag pressure, mmHg
mmHg2Pa   = 133.322;
p_LV      = p_LV_mmHg * mmHg2Pa;   % Pa
p_RV      = p_RV_mmHg * mmHg2Pa;   % Pa
lv_fast   = false;    % true → LV on quick-return stroke; false → LV on slow stroke
F_e       = 10;      % N (elastic restoring force)
e_gb      = 0.90;
e_mech    = 0.72;
e_motor   = 0.83;

omega_gb  = 2*pi*rpm_max/60;
CO_target = 6;       % L/min per ventricle
P_max     = 15.6;    % W power budget
alpha_min          = 8;   % deg — floor for teardrop (≤8° pushes Lc to its 50mm bound)
QR_min             = 1.0; % lower bound on quick-return ratio (peak fast-stroke / slow-stroke rate)
QR_max             = 2.0; % upper bound on quick-return ratio
L_paddle_pin_radius = 3;  % mm — inner edge of contact zone must clear the pivot pin
t_paddle            = 3;  % mm — paddle body thickness (perpendicular to arm in mechanism plane)
r1                  = 2.25;  % mm — crank pin radius (fixed); r2 cannot be smaller than this

lambda      = 100;  % smoothness weight (W per deg/deg kink)
size_weight = 0.2;  % W/mm — penalises a + L; raise to push harder for compact geometry

phi_deg = linspace(0, 360, 361);   % 1-deg resolution for numerical derivatives

%% Bounds  v = [x, a, f, L, w, L_contact]
lb = [ 7,   17,  0.05, 15,    33,   5];
ub = [20,   40,  0.90, 60,   75,  50];

% Linear inequalities:
%   x - a                  <= -1                  (x < a)
%   L_contact - L          <= -L_paddle_pin_radius (inner edge clears pivot pin)
A  = [1 -1  0  0 0 0;
      0  0  0 -1 0 1];
bb = [-1; -L_paddle_pin_radius];

objective = @(v) size_weight * (v(2) + v(4)) + lambda * kink_penalty(v, phi_deg, r1);
nonlcon   = @(v) nlcon(v, b, rpm_max, CO_target, alpha_min, phi_deg, ...
                       omega_gb, e_gb, e_mech, e_motor, p_LV, p_RV, lv_fast, F_e, P_max, r1, QR_min, QR_max);

%% Multi-start seeds
v0_list = {
    [9.9, 28, 0.20, 33, 75, 33],   % current x/a, f=0.2
    [8.5, 38, 0.15, 41, 75, 41],   % large a, small teardrop
    [9.0, 35, 0.40, 43, 75, 42],   % moderate a, heavy teardrop
};
opts_s = optimoptions('fmincon','Display','none','Algorithm','sqp','MaxFunctionEvaluations',4000);
opts_v = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',4000);

fprintf('Multi-start search (%d seeds)...\n', numel(v0_list));
obj_best = inf;  v_best = v0_list{1};
for k = 1:numel(v0_list)
    try
        [vk, fk, efk] = fmincon(objective, v0_list{k}, A, bb, [], [], lb, ub, nonlcon, opts_s);
        if efk > 0 && fk < obj_best
            obj_best = fk;  v_best = vk;
        end
    catch
    end
end
fprintf('Best seed gives obj = %.3f; refining with display:\n\n', obj_best);
[v_opt, ~] = fmincon(objective, v_best, A, bb, [], [], lb, ub, nonlcon, opts_v);

%% Unpack and report
[xo, ao, fo, Lo, wo, Lco] = unpack(v_opt);
r2o    = fo * xo;
r_effo = r2o - r1;

theta_o  = teardrop_theta(phi_deg, xo, ao, r_effo);
dth_o    = gradient(theta_o, phi_deg);
alpha_o  = max(theta_o);
qr_o     = max(dth_o) / max(-dth_o);
co_o     = co_calc(v_opt, b, rpm_max, phi_deg, r1);
P_elec_o = p_elec_peak(v_opt, omega_gb, e_gb, e_mech, e_motor, p_LV, p_RV, lv_fast, F_e, phi_deg, r1);
kink_o   = kink_penalty(v_opt, phi_deg, r1);

% Comparison: minimum-feasible slot at same L/w/Lc (r2 = r1, the smallest
% wall that can even contain the pin). This gives r_eff = 0 -- i.e. the
% smallest buildable teardrop has ZERO kinematic effect and is identical
% to the pure straight slot; any real quick-return benefit needs r2 > r1.
r2_min    = r1;
v_str     = [xo, ao, r2_min/xo, Lo, wo, Lco];
theta_str = teardrop_theta(phi_deg, xo, ao, r2_min - r1);
dth_str   = gradient(theta_str, phi_deg);
alpha_str = max(theta_str);
qr_str    = max(dth_str) / max(-dth_str);
p_str     = p_elec_peak(v_str, omega_gb, e_gb, e_mech, e_motor, p_LV, p_RV, lv_fast, F_e, phi_deg, r1);
co_str    = co_calc(v_str, b, rpm_max, phi_deg, r1);
kink_str  = kink_penalty(v_str, phi_deg, r1);

if P_elec_o <= P_max, s_td = 'within budget'; else, s_td = 'OVER BUDGET'; end
if p_str    <= P_max, s_st = 'within budget'; else, s_st = 'OVER BUDGET'; end

fprintf('\n=== Optimised Teardrop Mechanism  (size_weight=%.2f  lambda=%.0f) ===\n', size_weight, lambda);
fprintf('  x         = %.3f mm\n', xo);
fprintf('  a         = %.3f mm\n', ao);
fprintf('  f = r2/x  = %.3f  ->  r2 = %.3f mm  (floor %.1f = r1, crank pin radius; r_eff = %.3f mm)\n', ...
    fo, r2o, r1, r_effo);
fprintf('  L         = %.3f mm\n', Lo);
fprintf('  w         = %.3f mm\n', wo);
fprintf('  L_contact = %.3f mm\n', Lco);
fprintf('  -----------------------------------------\n');
fprintf('  alpha       = %.2f deg  (floor %.0f)\n', alpha_o, alpha_min);
fprintf('  QR ratio    = %.3f  (bounds [%.1f, %.1f])\n', qr_o, QR_min, QR_max);
fprintf('  CO          = %.3f L/min/ventricle  (target %.1f)\n', co_o, CO_target);
fprintf('  P_elec_peak = %.3f W  (%s)\n', P_elec_o, s_td);
fprintf('  kink        = %.4f deg/deg\n', kink_o);
Di_slot = 2*xo - r_effo;
Tx_slot = r2o * sqrt(max(Di_slot^2 - r_effo^2, 0)) / Di_slot;   % actual wall half-width (not r_eff)
warn_t  = ''; if t_paddle < 2*Tx_slot; warn_t = '  *** TOO THIN ***'; end
fprintf('  t_paddle    = %.1f mm  (slot half-width Tx=%.2f mm, min t=%.1f mm)%s\n', ...
    t_paddle, Tx_slot, 2*Tx_slot, warn_t);
fprintf('\n=== Same L/w/Lc at r2=%.1fmm (min-feasible, crank pin limit) ===\n', r1);
fprintf('  alpha       = %.2f deg\n', alpha_str);
fprintf('  QR ratio    = %.3f\n', qr_str);
fprintf('  CO          = %.3f L/min/ventricle\n', co_str);
fprintf('  P_elec_peak = %.3f W  (%s)\n', p_str, s_st);
fprintf('  kink        = %.4f deg/deg\n', kink_str);

%% ===================================================================
function P = p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_LV, p_RV, lv_fast, F_e, phi_deg, r1)
    [x, ~, f, L, w, Lc] = unpack(v);
    r2     = f * x;
    r_eff  = r2 - r1;
    theta      = teardrop_theta(phi_deg, x, v(2), r_eff);
    dth        = gradient(theta, phi_deg);
    peak_fast  = max(dth);                            % fast stroke (phi~0), dimensionless
    peak_slow  = max(-dth);                           % slow stroke (phi~180), dimensionless
    A_contact  = w * Lc * 1e-6;                      % m²
    if lv_fast
        F_fast = p_LV * A_contact + F_e;
        F_slow = p_RV * A_contact + F_e;
    else
        F_fast = p_RV * A_contact + F_e;
        F_slow = p_LV * A_contact + F_e;
    end
    r_moment   = (L - Lc/2) * 1e-3;                 % m
    P_mech     = max(F_fast * peak_fast, F_slow * peak_slow) * r_moment * omega_gb / (e_gb * e_mech);
    P          = P_mech / e_motor;
end

function CO = co_calc(v, b, rpm_max, phi_deg, r1)
    [x, ~, f, L, w, Lc] = unpack(v);
    r2     = f * x;
    r_eff  = r2 - r1;
    theta  = teardrop_theta(phi_deg, x, v(2), r_eff);
    alpha  = max(theta);                                    % deg
    K_geom = w * (L^2 - (L-Lc)^2) / 2;                   % mm³/rad (deg/deg cancel)
    SV     = K_geom * alpha * pi/180 / (1000*(1-b));      % mL
    CO     = SV * rpm_max / 1000;                          % L/min
end

function [c, ceq] = nlcon(v, b, rpm_max, CO_target, alpha_min, phi_deg, ...
                          omega_gb, e_gb, e_mech, e_motor, p_LV, p_RV, lv_fast, F_e, P_max, r1, QR_min, QR_max)
    [x, a, f, ~, ~, ~] = unpack(v);
    r2    = f * x;
    r_eff = r2 - r1;
    theta = teardrop_theta(phi_deg, x, a, r_eff);
    dth   = gradient(theta, phi_deg);
    alpha = max(theta);
    QR    = max(dth) / max(-dth);
    CO    = co_calc(v, b, rpm_max, phi_deg, r1);
    P     = p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_LV, p_RV, lv_fast, F_e, phi_deg, r1);
    ceq   = [];
    c     = [alpha_min - alpha;
             CO - 1.0*CO_target;
             1.0*CO_target - CO;
             P - P_max;
             r1 - r2;
             QR_min - QR;
             QR - QR_max];
end

function k = kink_penalty(v, phi_deg, r1)
    [x, ~, f, ~, ~, ~] = unpack(v);
    r2    = f * x;
    r_eff = r2 - r1;
    theta = teardrop_theta(phi_deg, x, v(2), r_eff);
    dth   = gradient(theta, phi_deg);
    k     = max(abs(diff(dth)));
end

function [x, a, f, L, w, Lc] = unpack(v)
    x = v(1); a = v(2); f = v(3); L = v(4); w = v(5); Lc = v(6);
end

function theta = teardrop_theta(phi_deg, x, a, r_arc)
% theta(phi) for the teardrop-slot (vectorised). Mirrors teardrop_theta()
% in paddle_angle_teardrop.m; r_arc=0 recovers the straight-slot formula.
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
    Tx  = r_arc * sqrt(max(Di^2 - r_arc^2, 0)) / Di;   % scalar
    Ty  = ci + r_arc^2 / Di;                          % scalar
    R_T = hypot(Tx, Ty);                           % scalar

    R    = sqrt(a^2 + x^2 - 2*a*x*cosd(phi_deg)); % array
    beta = zeros(size(phi_deg));
    sgn  = ones(size(phi_deg));
    sgn(phi_deg > 180) = -1;

    % Arc branch: pin-centre rides the r_arc arc (R <= R_T, near phi=0/360)
    arc = R <= R_T;
    if any(arc)
        Yp = (R(arc).^2 - r_arc^2 + ci^2) / (2*ci);
        Xp = sqrt(max(R(arc).^2 - Yp.^2, 0));
        beta(arc) = atan2d(sgn(arc).*Xp, Yp);
    end

    % Tangent-line branch: pin-centre rides the straight path (R > R_T)
    lin = ~arc;
    if any(lin)
        Ay  = a + x;
        Ac  = Tx^2 + (Ay-Ty)^2;           % scalar
        Bc  = -2*Tx^2 + 2*Ty*(Ay-Ty);    % scalar
        Cc  = R_T^2 - R(lin).^2;          % array
        dsc = max(Bc^2 - 4*Ac*Cc, 0);     % array, element-wise max
        t1  = (-Bc + sqrt(dsc)) / (2*Ac);
        t2  = (-Bc - sqrt(dsc)) / (2*Ac);
        t   = t1;
        bad = t < 0 | t > 1;
        d1  = max(0, t1-1) + max(0, -t1);
        d2  = max(0, t2-1) + max(0, -t2);
        t(bad & d2 < d1) = t2(bad & d2 < d1);
        t   = max(min(t, 1), 0);
        Xp  = Tx*(1-t);
        Yp  = Ty + t.*(Ay-Ty);
        beta(lin) = atan2d(sgn(lin).*Xp, Yp);
    end

    theta = theta_o - beta;
end
