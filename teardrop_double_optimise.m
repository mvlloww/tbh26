%% Double-Radius Paddle / Mechanism Geometry Optimiser
%
% Extends teardrop_optimise.m by adding r2 (side-wall arc radius) as a
% 7th design variable. r2 arcs replace the straight slot sides, smoothing
% dtheta/dphi at the r1/wall junction to reduce power peaks without
% changing stroke volume significantly.
%
% Design vector v = [x, a, f, g, L, w, L_contact]
%   f = r1/x  in [0, 0.9]   — teardrop arc fraction
%   g = r2/x  in [0, 5]     — side-wall arc fraction (g=0 → straight sides / single teardrop)
%
% Minimises P_elec_peak subject to CO in [0.98, 1.02]*CO_target.

clear; clc;

%% Fixed parameters
rpm_max = 145;
b       = 0.5;
p_bag   = 16e3;
F_e     = 10;
e_gb    = 0.90;
e_mech  = 0.72;
e_motor = 0.83;

omega_gb  = 2*pi*rpm_max/60;
CO_target = 5;
P_max     = 15.6;
alpha_min = 8;   % deg

phi_deg = linspace(0, 360, 361);

%% Bounds  v = [x, a, f, g, L, w, Lc]
lb = [ 5,  12, 0.00, 0.00, 15,  33,  5];
ub = [20,  40, 0.90, 30.0, 60, 75, 50];  % g up to 30x so r2 can reach ~200mm

% Linear inequalities:
%   x - a         <= -1   (x < a)
%   L_contact - L <= 0    (Lc <= L)
A  = [1 -1  0  0  0 0 0;
      0  0  0  0 -1 0 1];
bb = [-1; 0];

objective = @(v) p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg);
nonlcon   = @(v) nlcon(v, b, rpm_max, CO_target, alpha_min, phi_deg);

%% Multi-start seeds  (w→100, L≈Lc, CO≈5 L/min, f+g >= 1 satisfied)
v0_list = {
    [9.9, 28, 0.20, 0.00, 33, 100, 33],   % g=0 → straight sides (single teardrop baseline)
    [9.9, 28, 0.20, 1.00, 33, 100, 33],   % moderate r2
    [8.5, 38, 0.15, 1.00, 41, 100, 41],   % large a, moderate r2
    [9.0, 35, 0.40, 2.00, 43, 100, 42],   % moderate a, large r2
};

opts_s = optimoptions('fmincon','Display','none','Algorithm','sqp','MaxFunctionEvaluations',5000);
opts_v = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',5000);

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

%% Unpack and report
[xo, ao, fo, go, Lo, wo, Lco] = unpack(v_opt);
r1o = fo * xo;
r2o = go * xo;

theta_o = teardrop_double_theta(phi_deg, xo, ao, r1o, r2o);
dth_o   = gradient(theta_o, phi_deg);
alpha_o = max(theta_o);
qr_o    = max(dth_o) / max(-dth_o);
co_o    = co_calc(v_opt, b, rpm_max, phi_deg);

% Comparison: same L/w/Lc with r1 only (r2=0)
v_r1only = [xo, ao, fo, 0, Lo, wo, Lco];
p_r1only = p_elec_peak(v_r1only, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg);
co_r1only= co_calc(v_r1only, b, rpm_max, phi_deg);

% Comparison: straight slot (r1=r2=0)
v_str  = [xo, ao, 0, 0, Lo, wo, Lco];
p_str  = p_elec_peak(v_str, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg);
co_str = co_calc(v_str, b, rpm_max, phi_deg);

if P_opt   <= P_max, s1='within budget'; else, s1='OVER BUDGET'; end
if p_r1only<= P_max, s2='within budget'; else, s2='OVER BUDGET'; end
if p_str   <= P_max, s3='within budget'; else, s3='OVER BUDGET'; end

fprintf('\n=== Optimised Double-Radius Mechanism ===\n');
fprintf('  x         = %.3f mm\n', xo);
fprintf('  a         = %.3f mm\n', ao);
fprintf('  f = r1/x  = %.3f  ->  r1 = %.3f mm\n', fo, r1o);
fprintf('  g = r2/x  = %.3f  ->  r2 = %.3f mm\n', go, r2o);
fprintf('  L         = %.3f mm\n', Lo);
fprintf('  w         = %.3f mm\n', wo);
fprintf('  L_contact = %.3f mm\n', Lco);
fprintf('  -----------------------------------------\n');
fprintf('  alpha       = %.2f deg  (floor %.0f)\n', alpha_o, alpha_min);
fprintf('  QR ratio    = %.3f\n', qr_o);
fprintf('  CO          = %.3f L/min/ventricle  (target %.1f ±2%%)\n', co_o, CO_target);
fprintf('  P_elec_peak = %.3f W  (%s)\n', P_opt, s1);
fprintf('\n=== Same L/w/Lc with r1 only (r2=0) ===\n');
fprintf('  P_elec_peak = %.3f W  (%s)\n', p_r1only, s2);
fprintf('  r2 benefit: %.3f W  (%.1f%% reduction)\n', p_r1only-P_opt, 100*(p_r1only-P_opt)/max(p_r1only,eps));
fprintf('\n=== Same L/w/Lc straight slot (r1=r2=0) ===\n');
fprintf('  P_elec_peak = %.3f W  (%s)\n', p_str, s3);

%% ================================================================
function P = p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg)
    [x, ~, f, g, L, w, Lc] = unpack(v);
    r1 = f*x;  r2 = g*x;
    theta     = teardrop_double_theta(phi_deg, x, v(2), r1, r2);
    peak_rate = max(abs(gradient(theta, phi_deg)));
    A_contact = w * Lc * 1e-6;
    F_total   = p_bag * A_contact + F_e;
    r_moment  = (L - Lc/2) * 1e-3;
    Tp        = F_total * r_moment;
    P         = Tp * peak_rate * omega_gb / (e_gb * e_mech * e_motor);
end

function CO = co_calc(v, b, rpm_max, phi_deg)
    [x, ~, f, g, L, w, Lc] = unpack(v);
    r1    = f*x;  r2 = g*x;
    theta = teardrop_double_theta(phi_deg, x, v(2), r1, r2);
    alpha = max(theta);
    K_geom = w * (L^2 - (L-Lc)^2) / 2;
    SV     = K_geom * alpha * pi/180 / (1000*(1-b));
    CO     = SV * rpm_max / 1000;
end

function [c, ceq] = nlcon(v, b, rpm_max, CO_target, alpha_min, phi_deg)
    alpha = max(teardrop_double_theta(phi_deg, v(1), v(2), v(3)*v(1), v(4)*v(1)));
    CO    = co_calc(v, b, rpm_max, phi_deg);
    ceq   = [];
    c     = [alpha_min - alpha;
             CO - 1.02*CO_target;
             0.98*CO_target - CO];
end

function [x, a, f, g, L, w, Lc] = unpack(v)
    x=v(1); a=v(2); f=v(3); g=v(4); L=v(5); w=v(6); Lc=v(7);
end

function theta = teardrop_double_theta(phi_deg, x, a, r1, r2)
    theta_o = atand(x*sind(phi_deg) ./ (a - x*cosd(phi_deg)));
    if r1 < 1e-9; theta = theta_o; return; end
    if r2 > 0 && r2 < x - r1; r2 = 0; end  % invalid r2 → single teardrop
    ci = (a - x) + r1;  D = 2*x - r1;
    if D <= 0; theta = theta_o; return; end
    R    = sqrt(a^2 + x^2 - 2*a*x*cosd(phi_deg));
    beta = zeros(size(phi_deg));
    sgn  = ones(size(phi_deg));  sgn(phi_deg > 180) = -1;

    if r2 < 1e-9
        Tx=r1*sqrt(max(D^2-r1^2,0))/D; Ty=ci+r1^2/D; R_T=hypot(Tx,Ty);
        arc=R<=R_T;
        if any(arc)
            Yp=(R(arc).^2-r1^2+ci^2)/(2*ci); Xp=sqrt(max(R(arc).^2-Yp.^2,0));
            beta(arc)=atan2d(sgn(arc).*Xp,Yp);
        end
        lin=~arc;
        if any(lin)
            Ay=a+x; Ac=Tx^2+(Ay-Ty)^2; Bc=-2*Tx^2+2*Ty*(Ay-Ty); Cc=R_T^2-R(lin).^2;
            dsc=max(Bc^2-4*Ac*Cc,0);
            t1=(-Bc+sqrt(dsc))/(2*Ac); t2=(-Bc-sqrt(dsc))/(2*Ac); t=t1;
            bad=t<0|t>1; d1=max(0,t1-1)+max(0,-t1); d2=max(0,t2-1)+max(0,-t2);
            t(bad&d2<d1)=t2(bad&d2<d1); t=max(min(t,1),0);
            beta(lin)=atan2d(sgn(lin).*Tx.*(1-t), Ty+t.*(Ay-Ty));
        end
    else
        dy=(r1*(2*x+r2)-2*x^2)/D; Y2=(a+x)+dy; X2=sqrt(max(r2^2-dy^2,0));
        r_sm=r1+r2; R_T=hypot(r1*X2/r_sm,(ci*r2+r1*Y2)/r_sm);
        arc=R<=R_T;
        if any(arc)
            Yp=(R(arc).^2-r1^2+ci^2)/(2*ci); Xp=sqrt(max(R(arc).^2-Yp.^2,0));
            beta(arc)=atan2d(sgn(arc).*Xp,Yp);
        end
        r2b=~arc;
        if any(r2b) && X2>1e-12
            D2=X2^2+Y2^2; K=(D2+R(r2b).^2-r2^2)/2;
            disc=max(D2.*R(r2b).^2-K.^2,0);
            Yp=(K.*Y2+X2.*sqrt(disc))/D2; Xp=(K-Y2.*Yp)/X2;
            beta(r2b)=atan2d(sgn(r2b).*Xp,Yp);
        end
    end
    theta = theta_o - beta;
end
