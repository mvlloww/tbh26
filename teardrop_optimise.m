%% Teardrop Paddle / Mechanism Geometry Optimiser
%
% Minimises  size_weight*(a + L) + lambda * kink
% subject to:  CO = CO_target,  alpha >= alpha_min,  P_elec_peak <= P_max.
%
% Power is a hard constraint (budget), not the primary objective.
% The optimizer trades the power headroom for smaller mechanism dimensions.
%   size_weight — W/mm; raise to push harder for compact geometry
%   lambda      — W/(deg/deg kink); raise to prefer smooth power profiles
%
% Design vector v = [x, a, f, L, w, L_contact]
%   f = r1/x in [0.05, 0.9]: r1=f*x is the teardrop arc radius

clear; clc;

%% Fixed parameters (match paddle_optimise.m)
rpm_max = 145;
b       = 0.5;
p_bag   = 16e3;      % Pa
F_e     = 10;        % N (elastic restoring force)
e_gb    = 0.90;
e_mech  = 0.72;
e_motor = 0.83;

omega_gb  = 2*pi*rpm_max/60;
CO_target = 5;       % L/min per ventricle
P_max     = 15.6;    % W power budget
alpha_min          = 8;   % deg — floor for teardrop (≤8° pushes Lc to its 50mm bound)
L_paddle_pin_radius = 3;  % mm — inner edge of contact zone must clear the pivot pin
t_paddle            = 4;  % mm — paddle body thickness (perpendicular to arm in mechanism plane)

lambda      = 100;  % smoothness weight (W per deg/deg kink)
size_weight = 0.2;  % W/mm — penalises a + L; raise to push harder for compact geometry

phi_deg = linspace(0, 360, 361);   % 1-deg resolution for numerical derivatives

%% Bounds  v = [x, a, f, L, w, L_contact]
lb = [ 5,   12,  0.05, 15,    33,   5];
ub = [20,   40,  0.90, 60,   75,  50];

% Linear inequalities:
%   x - a                  <= -1                  (x < a)
%   L_contact - L          <= -L_paddle_pin_radius (inner edge clears pivot pin)
A  = [1 -1  0  0 0 0;
      0  0  0 -1 0 1];
bb = [-1; -L_paddle_pin_radius];

objective = @(v) size_weight * (v(2) + v(4)) + lambda * kink_penalty(v, phi_deg);
nonlcon   = @(v) nlcon(v, b, rpm_max, CO_target, alpha_min, phi_deg, ...
                       omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, P_max);

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
r1o = fo * xo;

theta_o  = teardrop_theta(phi_deg, xo, ao, r1o);
dth_o    = gradient(theta_o, phi_deg);
alpha_o  = max(theta_o);
qr_o     = max(dth_o) / max(-dth_o);
co_o     = co_calc(v_opt, b, rpm_max, phi_deg);
P_elec_o = p_elec_peak(v_opt, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg);
kink_o   = kink_penalty(v_opt, phi_deg);

% Comparison: straight slot at same L/w/Lc (r1=0)
v_str    = [xo, ao, 0, Lo, wo, Lco];
alpha_str = asind(xo/ao);
qr_str    = (ao+xo) / (ao-xo);
p_str     = p_elec_peak(v_str, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg);
co_str    = co_calc(v_str, b, rpm_max, phi_deg);
kink_str  = kink_penalty(v_str, phi_deg);

if P_elec_o <= P_max, s_td = 'within budget'; else, s_td = 'OVER BUDGET'; end
if p_str    <= P_max, s_st = 'within budget'; else, s_st = 'OVER BUDGET'; end

fprintf('\n=== Optimised Teardrop Mechanism  (size_weight=%.2f  lambda=%.0f) ===\n', size_weight, lambda);
fprintf('  x         = %.3f mm\n', xo);
fprintf('  a         = %.3f mm\n', ao);
fprintf('  f = r1/x  = %.3f  ->  r1 = %.3f mm\n', fo, r1o);
fprintf('  L         = %.3f mm\n', Lo);
fprintf('  w         = %.3f mm\n', wo);
fprintf('  L_contact = %.3f mm\n', Lco);
fprintf('  -----------------------------------------\n');
fprintf('  alpha       = %.2f deg  (floor %.0f)\n', alpha_o, alpha_min);
fprintf('  QR ratio    = %.3f\n', qr_o);
fprintf('  CO          = %.3f L/min/ventricle  (target %.1f)\n', co_o, CO_target);
fprintf('  P_elec_peak = %.3f W  (%s)\n', P_elec_o, s_td);
fprintf('  kink        = %.4f deg/deg\n', kink_o);
Di_slot = 2*xo - r1o;
Tx_slot = r1o * sqrt(max(Di_slot^2 - r1o^2, 0)) / Di_slot;
warn_t  = ''; if t_paddle < 2*Tx_slot; warn_t = '  *** TOO THIN ***'; end
fprintf('  t_paddle    = %.1f mm  (slot half-width Tx=%.2f mm, min t=%.1f mm)%s\n', ...
    t_paddle, Tx_slot, 2*Tx_slot, warn_t);
fprintf('\n=== Same L/w/Lc at r1=0 (straight slot) ===\n');
fprintf('  alpha       = %.2f deg\n', alpha_str);
fprintf('  QR ratio    = %.3f\n', qr_str);
fprintf('  CO          = %.3f L/min/ventricle\n', co_str);
fprintf('  P_elec_peak = %.3f W  (%s)\n', p_str, s_st);
fprintf('  kink        = %.4f deg/deg\n', kink_str);

%% ===================================================================
function P = p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg)
    [x, ~, f, L, w, Lc] = unpack(v);
    r1 = f * x;
    theta      = teardrop_theta(phi_deg, x, v(2), r1);
    peak_rate  = max(abs(gradient(theta, phi_deg)));   % max |dθ/dφ|, dimensionless
    A_contact  = w * Lc * 1e-6;                       % m²
    F_total    = p_bag * A_contact + F_e;             % N
    r_moment   = (L - Lc/2) * 1e-3;                   % m
    Tp         = F_total * r_moment;                  % N·m
    P_mech     = Tp * peak_rate * omega_gb / (e_gb * e_mech);
    P          = P_mech / e_motor;
end

function CO = co_calc(v, b, rpm_max, phi_deg)
    [x, ~, f, L, w, Lc] = unpack(v);
    r1     = f * x;
    theta  = teardrop_theta(phi_deg, x, v(2), r1);
    alpha  = max(theta);                                    % deg
    K_geom = w * (L^2 - (L-Lc)^2) / 2;                   % mm³/rad (deg/deg cancel)
    SV     = K_geom * alpha * pi/180 / (1000*(1-b));      % mL
    CO     = SV * rpm_max / 1000;                          % L/min
end

function [c, ceq] = nlcon(v, b, rpm_max, CO_target, alpha_min, phi_deg, ...
                          omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, P_max)
    [x, a, f, ~, ~, ~] = unpack(v);
    r1    = f * x;
    theta = teardrop_theta(phi_deg, x, a, r1);
    alpha = max(theta);
    CO    = co_calc(v, b, rpm_max, phi_deg);
    P     = p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg);
    ceq   = [];
    c     = [alpha_min - alpha;
             CO - 1.0*CO_target;
             1.0*CO_target - CO;
             P - P_max];
end

function k = kink_penalty(v, phi_deg)
    [x, ~, f, ~, ~, ~] = unpack(v);
    theta = teardrop_theta(phi_deg, x, v(2), f*x);
    dth   = gradient(theta, phi_deg);
    k     = max(abs(diff(dth)));
end

function [x, a, f, L, w, Lc] = unpack(v)
    x = v(1); a = v(2); f = v(3); L = v(4); w = v(5); Lc = v(6);
end

function theta = teardrop_theta(phi_deg, x, a, r1)
% theta(phi) for the teardrop-slot (vectorised). Mirrors teardrop_theta()
% in paddle_angle_teardrop.m; f=0 (r1=0) recovers the straight-slot formula.
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
    Tx  = r1 * sqrt(max(Di^2 - r1^2, 0)) / Di;   % scalar
    Ty  = ci + r1^2 / Di;                          % scalar
    R_T = hypot(Tx, Ty);                           % scalar

    R    = sqrt(a^2 + x^2 - 2*a*x*cosd(phi_deg)); % array
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
