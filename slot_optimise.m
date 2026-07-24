%% Paddle/Mechanism Geometry Optimiser
%
% Finds x, a, L, w, L_contact that minimise P_elec_peak subject to:
%   CO    == 5 L/min per ventricle   (equality)
%   alpha >= 20 deg                  (paddle swing floor)
%
% CO and P_elec_peak use closed-form peak-cycle expressions — GR cancels out
% of P_elec_peak entirely (P_mech = Tg*omega_gb/(e_gb*e_mech), and
% omega_motor = omega_gb*GR while Tm = Tg/(GR*e_gb*e_mech)). No time-domain
% simulation needed. Mirrors the physics in paddle_angle_plot.m
% (Tp, Tg, Tm/P_elec sections). Peak occurs at phi=0 (fast stroke, factor
% x/(a-x)) or phi=180 (slow stroke, factor x/(a+x)) depending on which
% ventricle's pressure lands on the faster stroke — see lv_fast.

clear; clc;

%% Fixed parameters (match paddle_angle_plot.m)
rpm_max   = 145;
b         = 0.5;
p_LV_mmHg = 120;     % LV peak bag pressure, mmHg
p_RV_mmHg = 25;      % RV peak bag pressure, mmHg
mmHg2Pa   = 133.322;
p_LV      = p_LV_mmHg * mmHg2Pa;   % Pa
p_RV      = p_RV_mmHg * mmHg2Pa;   % Pa
lv_fast   = false;   % true → LV on quick-return stroke; false → LV on slow stroke
F_e       = 10;
e_gb      = 0.90;
e_mech    = 0.72;
e_motor   = 0.83;

omega_gb  = 2*pi*rpm_max/60;
CO_target = 6;       % L/min per ventricle
P_max     = 15.6;    % W  (power budget, see tbh27_mechanism_archived.m)
alpha_min = 20;      % deg, paddle swing floor

%% Design vector v = [x, a, L, w, L_contact]  (mm)
v0 = [10, 20, 30, 66, 20];
lb = [ 5, 10, 15, 33,  5];
ub = [20, 40, 60, 75, 30];

% Linear inequalities A*v <= bb:  x - a <= -1 (x < a),  L_contact - L <= 0
A  = [1 -1  0 0 0;
      0  0 -1 0 1];
bb = [-1; 0];

objective = @(v) p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_LV, p_RV, lv_fast, F_e);
nonlcon   = @(v) constraints(v, b, rpm_max, CO_target, alpha_min);

opts = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[v_opt, P_opt] = fmincon(objective, v0, A, bb, [], [], lb, ub, nonlcon, opts);

x = v_opt(1); a = v_opt(2); L = v_opt(3); w = v_opt(4); L_contact = v_opt(5);
alpha = asind(x/a);
CO    = co(v_opt, b, rpm_max);

%% Report
verdict = {'OVER BUDGET','within budget'};
fprintf('=== Optimised Paddle/Mechanism Geometry ===\n');
fprintf('  x         = %.3f mm\n', x);
fprintf('  a         = %.3f mm\n', a);
fprintf('  L         = %.3f mm\n', L);
fprintf('  w         = %.3f mm\n', w);
fprintf('  L_contact = %.3f mm\n', L_contact);
fprintf('  ---------------------------------------\n');
fprintf('  alpha       = %.2f deg  (floor %.0f)\n', alpha, alpha_min);
fprintf('  CO          = %.3f L/min/ventricle  (target %.1f)\n', CO, CO_target);
fprintf('  P_elec_peak = %.3f W  (budget %.1f)  -> %s\n', P_opt, P_max, verdict{(P_opt<=P_max)+1});
fprintf('  ---------------------------------------\n');
if lv_fast
    fprintf('  LV assignment: fast stroke (phi~0, high QR torque)\n');
else
    fprintf('  LV assignment: slow stroke (phi~180, lower peak torque)\n');
end
fprintf('  p_LV = %g mmHg  |  p_RV = %g mmHg\n', p_LV_mmHg, p_RV_mmHg);
fprintf('\n');
fprintf('  NOTE: x and a affect CO and P_elec_peak only via x/a = sin(alpha).\n');
fprintf('        With alpha pinned at the floor, x and a are not individually\n');
fprintf('        unique -- any pair within bounds satisfying x/a = sin(alpha)\n');
fprintf('        gives the same CO and P_elec_peak.\n');

%% ===================================================================
function P = p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_LV, p_RV, lv_fast, F_e)
    [x, a, L, w, Lc] = unpack(v);
    A_contact = w * Lc * 1e-6;               % m^2
    r_moment  = (L - Lc/2) * 1e-3;           % m
    peak_fast = x/(a-x);                     % dtheta/dphi at phi=0   (fast stroke)
    peak_slow = x/(a+x);                     % dtheta/dphi at phi=180 (slow stroke)
    if lv_fast
        F_fast = p_LV * A_contact + F_e;
        F_slow = p_RV * A_contact + F_e;
    else
        F_fast = p_RV * A_contact + F_e;
        F_slow = p_LV * A_contact + F_e;
    end
    Tg_peak   = max(F_fast * peak_fast, F_slow * peak_slow) * r_moment;   % N.m
    P_mech    = Tg_peak * omega_gb / (e_gb*e_mech);   % GR cancels
    P = P_mech / e_motor;
end

function CO = co(v, b, rpm_max)
    [x, a, L, w, Lc] = unpack(v);
    alpha  = asind(x/a);
    K_geom = w * (L^2 - (L-Lc)^2) / 2;                 % mm^3/rad
    SV     = K_geom * alpha * pi/180 / (1000*(1-b));   % mL
    CO     = SV * rpm_max / 1000;                      % L/min
end

function [c, ceq] = constraints(v, b, rpm_max, CO_target, alpha_min)
    [x, a, ~, ~, ~] = unpack(v);
    alpha = asind(x/a);
    ceq = co(v, b, rpm_max) - CO_target;   % CO == target
    c   = alpha_min - alpha;               % alpha >= alpha_min
end

function [x, a, L, w, Lc] = unpack(v)
    x = v(1); a = v(2); L = v(3); w = v(4); Lc = v(5);
end
