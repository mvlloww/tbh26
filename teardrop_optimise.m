%% Teardrop Paddle / Mechanism Geometry Optimiser
%
% Finds x, a, f (=r1/x), L, w, L_contact that minimise P_elec_peak subject to:
%   CO    in [0.95, 1.05] * CO_target  (±5% band, per ventricle)
%   alpha >= alpha_min                 (teardrop paddle swing floor)
%
% The teardrop slot adds r1 = f*x as a free variable. Both alpha and the
% peak angular rate dtheta/dphi are computed numerically (no closed form
% for the teardrop geometry). The power formula is unchanged from
% paddle_optimise.m — GR still cancels:
%
%   P_elec_peak = T_p × max|dθ/dφ| × omega_gb / (e_gb × e_mech × e_motor)
%
% where max|dθ/dφ| is the numerical peak from the teardrop theta curve
% (reduced vs the straight-slot value x/(a-x) when f > 0).
%
% A 3-point multi-start is used to reduce sensitivity to the initial guess.

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
alpha_min = 12;      % deg — relaxed vs straight-slot (teardrop reduces alpha)

phi_deg = linspace(0, 360, 361);   % 1-deg resolution for numerical derivatives

%% Design vector v = [x, a, f, L, w, L_contact]
%   f = r1/x in [0, 0.9]: f=0 → straight slot, f→1 → full circle (invalid)
v0 = [7.6, 22.4, 0.20, 31.5, 100, 30];
lb = [ 5,   12,  0.00, 15,    33,   5];
ub = [20,   40,  0.90, 60,   150,  50];

% Linear inequalities:
%   x - a       <= -1   (x < a: pin orbit inside crank-fulcrum span)
%   L_contact - L <= 0  (contact zone <= paddle length)
A  = [1 -1  0  0 0 0;
      0  0  0 -1 0 1];
bb = [-1; 0];

objective = @(v) p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg);
nonlcon   = @(v) nlcon(v, b, rpm_max, CO_target, alpha_min, phi_deg);

%% Multi-start: 3 seeds, silent; refine best with display
v0_list = {
    [7.6, 22.4, 0.20, 31.5, 100, 30],   % current design, small teardrop
    [10,  28,   0.40, 40,    80, 35],   % larger r1
    [ 6,  18,   0.30, 28,   120, 22],   % smaller mechanism
};
opts_s = optimoptions('fmincon','Display','none',   'Algorithm','sqp', ...
    'MaxFunctionEvaluations',4000);
opts_v = optimoptions('fmincon','Display','iter',   'Algorithm','sqp', ...
    'MaxFunctionEvaluations',4000);

fprintf('Multi-start search (%d seeds)...\n', numel(v0_list));
P_best = inf;  v_best = v0_list{1};
for k = 1:numel(v0_list)
    try
        [vk, Pk, efk] = fmincon(objective, v0_list{k}, A, bb, [], [], lb, ub, nonlcon, opts_s);
        if efk > 0 && Pk < P_best
            P_best = Pk;  v_best = vk;
        end
    catch
    end
end
fprintf('Best seed gives P = %.3f W; refining with display:\n\n', P_best);
[v_opt, P_opt] = fmincon(objective, v_best, A, bb, [], [], lb, ub, nonlcon, opts_v);

%% Unpack and derive quantities
[xo, ao, fo, Lo, wo, Lco] = unpack(v_opt);
r1o = fo * xo;

theta_o = teardrop_theta(phi_deg, xo, ao, r1o);
dth_o   = gradient(theta_o, phi_deg);
alpha_o = max(theta_o);
qr_o    = max(dth_o) / max(-dth_o);   % LV-peak / RV-peak rate ratio
co_o    = co_calc(v_opt, b, rpm_max, phi_deg);

%% Straight-slot comparison at the same L/w/Lc (r1=0, same x,a)
v_str    = [xo, ao, 0, Lo, wo, Lco];
alpha_str = asind(xo/ao);
qr_str    = (ao+xo) / (ao-xo);
p_str     = p_elec_peak(v_str, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg);
co_str    = co_calc(v_str, b, rpm_max, phi_deg);

%% Report
if P_opt <= P_max, s_td = 'within budget'; else, s_td = 'OVER BUDGET'; end
if p_str <= P_max, s_st = 'within budget'; else, s_st = 'OVER BUDGET'; end

fprintf('\n=== Optimised Teardrop Mechanism ===\n');
fprintf('  x         = %.3f mm\n', xo);
fprintf('  a         = %.3f mm\n', ao);
fprintf('  f = r1/x  = %.3f  ->  r1 = %.3f mm\n', fo, r1o);
fprintf('  L         = %.3f mm\n', Lo);
fprintf('  w         = %.3f mm\n', wo);
fprintf('  L_contact = %.3f mm\n', Lco);
fprintf('  -----------------------------------------\n');
fprintf('  alpha       = %.2f deg  (floor %.0f)\n', alpha_o, alpha_min);
fprintf('  QR ratio    = %.3f  (LV/RV peak rate; <1 means RV is faster)\n', qr_o);
fprintf('  CO          = %.3f L/min/ventricle  (target %.1f +/-2%%)\n', co_o, CO_target);
fprintf('  P_elec_peak = %.3f W  (budget %.1f)  -> %s\n', P_opt, P_max, s_td);
fprintf('\n');
fprintf('=== Same L/w/Lc at r1=0 (straight slot, no teardrop) ===\n');
fprintf('  alpha       = %.2f deg\n', alpha_str);
fprintf('  QR ratio    = %.3f\n', qr_str);
fprintf('  CO          = %.3f L/min/ventricle\n', co_str);
fprintf('  P_elec_peak = %.3f W  -> %s\n', p_str, s_st);
fprintf('\n');
fprintf('  Teardrop benefit: dP = %.3f W  (%.1f%% reduction)\n', ...
    p_str - P_opt, 100*(p_str - P_opt)/p_str);
fprintf('  CO penalty at r1=0: dCO = %.3f L/min (same paddles, no teardrop)\n', ...
    co_str - co_o);

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

function [c, ceq] = nlcon(v, b, rpm_max, CO_target, alpha_min, phi_deg)
    [x, a, f, ~, ~, ~] = unpack(v);
    r1    = f * x;
    theta = teardrop_theta(phi_deg, x, a, r1);
    alpha = max(theta);
    CO    = co_calc(v, b, rpm_max, phi_deg);
    ceq   = [];
    c     = [alpha_min - alpha;          % alpha >= alpha_min
             CO - 1.02*CO_target;        % CO <= 1.05 * target
             0.98*CO_target - CO];       % CO >= 0.95 * target
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
