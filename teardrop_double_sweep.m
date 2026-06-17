%% Crank-and-Slotted-Arm LVAD — Double-Radius Slot Variant (kinematics sweep)
%
% Extends the teardrop slot by replacing the straight side walls with a
% second circular arc of radius r2. The r2 arc is externally tangent to the
% r1 arc and passes through the slot apex, eliminating the C1 kink at the
% r1/straight-wall junction and smoothing dtheta/dphi (hence reducing power
% peaks).
%
% Geometry (body frame Y' = along arm from O4, X' = perpendicular):
%   r1 arc  : centre (0, ci=a-x+r1), radius r1  — near-pivot end
%   r2 arc  : passes through apex (0, a+x), externally tangent to r1 arc
%             centre at (X2, Y2) where:
%               dy = (r1*(2x+r2) - 2x^2) / (2x-r1)
%               Y2 = (a+x) + dy
%               X2 = sqrt(r2^2 - dy^2)
%   tangent point between r1 and r2: Tp = r1-weighted interpolation of centres
%   R_T = |O4 -> Tp| : branch boundary
%     R <= R_T  ->  pin on r1 arc
%     R >  R_T  ->  pin on r2 arc (r2->inf recovers straight-line teardrop)

clear; clc; close all;

%% Mechanism parameters
x  = 9.9;    % mm
a  = 28;     % mm
r1 = 0.2*x;  % fixed teardrop arc (0.2x = same as teardrop_sweep.m baseline)
b  = 0.5;
rpm_max  = 145;
T        = 60 / rpm_max;
omega_gb = 2*pi / T;

L         = 40;
w         = 80;
L_contact = 35;
K_geom    = w * (L^2 - (L-L_contact)^2) / 2;

phi_deg = linspace(0, 360, 1441);   % 0.25 deg resolution

%% r2 sweep
r2_vals = [0, 0.5, 1.0, 2.0] * x;   % r2=0 → original single-teardrop

theta_sweep = zeros(numel(r2_vals), numel(phi_deg));
alpha_new   = zeros(size(r2_vals));
phipk_new   = zeros(size(r2_vals));
qr_new      = zeros(size(r2_vals));

for i = 1:numel(r2_vals)
    theta_i = teardrop_double_theta(phi_deg, x, a, r1, r2_vals(i));
    theta_sweep(i,:) = theta_i;
    [amax, imax]  = max(theta_i);
    alpha_new(i)  = amax;
    phipk_new(i)  = phi_deg(imax);
    dth = gradient(theta_i, phi_deg);
    qr_new(i) = max(dth) / max(-dth);
end

%% Ejection windows & stroke volume
phi_LV_start = zeros(size(r2_vals));
phi_RV_start = zeros(size(r2_vals));
SV_new       = zeros(size(r2_vals));
CO_new       = zeros(size(r2_vals));
for i = 1:numel(r2_vals)
    gamma_i = alpha_new(i) * b / (1-b);
    [phi_LV_start(i), phi_RV_start(i)] = ...
        ejection_windows(phi_deg, theta_sweep(i,:), phipk_new(i), gamma_i);
    SV_new(i) = K_geom * alpha_new(i) * pi/180 / (1000*(1-b));
    CO_new(i) = 2 * SV_new(i) * rpm_max / 1000;
end

cmap   = lines(numel(r2_vals));
labels = arrayfun(@(v) sprintf('r2=%.1f mm',v), r2_vals,'UniformOutput',false);
labels{1} = 'r2=0 (teardrop only)';

%% Figure 1: theta(phi) and dtheta/dphi
figure('Name','Double-Radius Slot — Kinematics','Color','w','Position',[80 80 900 650]);

subplot(2,1,1); hold on;
for i = 1:numel(r2_vals)
    ls = '-'; if i==1; ls='k-'; end
    plot(phi_deg, theta_sweep(i,:), ls, 'Color',cmap(i,:), 'LineWidth',1.5);
end
yline(0,'k:'); xline(180,'k--');
xlabel('Crank Angle \phi (deg)'); ylabel('Paddle Angle \theta (deg)');
title(sprintf('\\theta(\\phi): double-radius slot r2 sweep  (r1=%.2f mm = 0.2x)', r1));
legend(labels,'Location','south','NumColumns',2); grid on; xlim([0 360]); xticks(0:45:360);

subplot(2,1,2); hold on;
for i = 1:numel(r2_vals)
    ls = '-'; if i==1; ls='k-'; end
    plot(phi_deg, gradient(theta_sweep(i,:),phi_deg), ls, 'Color',cmap(i,:),'LineWidth',1.5);
end
yline(0,'k:');
xlabel('Crank Angle \phi (deg)'); ylabel('d\theta/d\phi (deg/deg)');
title('Angular rate — note smoothed peak with r2 > 0');
legend(labels,'Location','best'); grid on; xlim([0 360]); xticks(0:45:360);

%% Figure 2: Flow rate
t     = linspace(0, T, 1000);
phi_t = mod(omega_gb*t*180/pi, 360);

Q_LV_sweep = zeros(numel(r2_vals), numel(t));
Q_RV_sweep = zeros(numel(r2_vals), numel(t));
for i = 1:numel(r2_vals)
    [Q_LV_sweep(i,:), Q_RV_sweep(i,:)] = ...
        flow_rate(phi_deg, theta_sweep(i,:), phipk_new(i), phi_LV_start(i), phi_RV_start(i), omega_gb, K_geom, phi_t);
end

figure('Name','Double-Radius Slot — Flow Rate','Color','w','Position',[120 120 900 650]);
subplot(2,1,1); hold on;
for i = 1:numel(r2_vals)
    ls='-'; if i==1; ls='k-'; end
    plot(t*1000, Q_LV_sweep(i,:), ls,'Color',cmap(i,:),'LineWidth',1.5);
end
ylabel('Q_{LV} (mL/s)'); title('LV Flow Rate — one cycle');
legend(labels,'Location','best'); grid on; xlim([0 T*1000]);

subplot(2,1,2); hold on;
for i = 1:numel(r2_vals)
    ls='-'; if i==1; ls='k-'; end
    plot(t*1000, Q_RV_sweep(i,:), ls,'Color',cmap(i,:),'LineWidth',1.5);
end
xlabel('Time (ms)'); ylabel('Q_{RV} (mL/s)'); title('RV Flow Rate — one cycle');
legend(labels,'Location','best'); grid on; xlim([0 T*1000]);

%% Figure 3: Slot geometry (body frame)
figure('Name','Double-Radius Slot — Geometry','Color','w','Position',[160 160 460 650]);
hold on;
plot([0 0],[a-x, a+x],'k-','LineWidth',2,'DisplayName','original straight');

for i = 1:numel(r2_vals)
    r1i = r1;
    r2i = r2_vals(i);
    ci  = (a-x) + r1i;
    Di  = 2*x - r1i;

    if r2i < 1e-6
        % Original teardrop outline
        Txi = r1i * sqrt(max(0, Di^2-r1i^2)) / Di;
        Tyi = ci + r1i^2/Di;
        thR = atan2(Tyi-ci, Txi);
        aa  = linspace(thR, -pi-thR, 200);
        r1X = r1i*cos(aa); r1Y = ci + r1i*sin(aa);
        oX  = [0,   Txi, r1X, -Txi, 0  ];
        oY  = [a+x, Tyi, r1Y,  Tyi, a+x];
    else
        dy   = (r1i*(2*x + r2i) - 2*x^2) / Di;
        Y2   = (a+x) + dy;
        X2   = sqrt(max(r2i^2 - dy^2, 0));
        r_sm = r1i + r2i;
        Tp_x = r1i * X2 / r_sm;
        Tp_y = (ci*r2i + r1i*Y2) / r_sm;

        % r1 arc: right tangent -> bottom -> left tangent
        thR = atan2(Tp_y-ci, Tp_x);
        aa  = linspace(thR, -pi-thR, 200);
        r1X = r1i*cos(aa); r1Y = ci+r1i*sin(aa);

        % r2 arc right side: tangent point -> apex
        a_Tp   = atan2(Tp_y-Y2, Tp_x-X2);
        a_apex = atan2((a+x)-Y2, -X2);
        aa2  = linspace(a_Tp, a_apex, 100);
        r2X  = X2 + r2i*cos(aa2);
        r2Y  = Y2 + r2i*sin(aa2);

        oX = [0, fliplr(r2X), r1X, r2X, 0];
        oY = [a+x, fliplr(r2Y), r1Y, r2Y, a+x];
    end
    plot(oX, oY, '--', 'Color',cmap(i,:),'LineWidth',1.5,'DisplayName',labels{i});
end

plot(0,0,'k+','MarkerSize',10,'LineWidth',2,'HandleVisibility','off');
yline(a,'k:','LineWidth',0.8,'HandleVisibility','off');
text(x*0.15, 0.8,     'O_4 (pivot)',  'FontSize',8);
text(x*0.15, a + 0.8, 'O_2 (motor)', 'FontSize',8);
xlabel("X' — perpendicular to arm (mm)");
ylabel("Y' — along arm from pivot O_4 (mm)");
title(sprintf('Double-radius slot geometry  (r1=%.2f mm = 0.2x, r2 sweep)',r1));
legend('Location','south','NumColumns',2); grid on; axis equal;
xlim([-x*1.5, x*1.5]); ylim([a-x-3, a+x+3]);

%% Console summary
fprintf('=== Double-radius slot sweep (x=%.2f mm, a=%.2f mm, r1=%.2f mm, b=%.2f) ===\n', x, a, r1, b);
for i = 1:numel(r2_vals)
    fprintf('  r2=%5.2f mm: alpha=%.2f deg @ phi=%.1f  |  QR=%.3f  |  SV=%.2f mL  |  CO=%.2f L/min\n', ...
        r2_vals(i), alpha_new(i), phipk_new(i), qr_new(i), SV_new(i), CO_new(i));
end

%% =======================================================================
function theta = teardrop_double_theta(phi_deg, x, a, r1, r2)
% theta(phi) for double-radius slot: r1 arc at bottom, r2 arcs on sides.
    theta_o = atand(x*sind(phi_deg) ./ (a - x*cosd(phi_deg)));
    if r1 < 1e-9; theta = theta_o; return; end
    ci = (a - x) + r1;
    D  = 2*x - r1;
    if D <= 0; theta = theta_o; return; end
    R    = sqrt(a^2 + x^2 - 2*a*x*cosd(phi_deg));
    beta = zeros(size(phi_deg));
    sgn  = ones(size(phi_deg));  sgn(phi_deg > 180) = -1;

    if r2 < 1e-9
        Tx  = r1 * sqrt(max(D^2 - r1^2, 0)) / D;
        Ty  = ci + r1^2 / D;
        R_T = hypot(Tx, Ty);
        arc = R <= R_T;
        if any(arc)
            Yp = (R(arc).^2 - r1^2 + ci^2) / (2*ci);
            Xp = sqrt(max(R(arc).^2 - Yp.^2, 0));
            beta(arc) = atan2d(sgn(arc).*Xp, Yp);
        end
        lin = ~arc;
        if any(lin)
            Ay=a+x; Ac=Tx^2+(Ay-Ty)^2; Bc=-2*Tx^2+2*Ty*(Ay-Ty); Cc=R_T^2-R(lin).^2;
            dsc=max(Bc^2-4*Ac*Cc,0);
            t1=(-Bc+sqrt(dsc))/(2*Ac); t2=(-Bc-sqrt(dsc))/(2*Ac); t=t1;
            bad=t<0|t>1;
            d1=max(0,t1-1)+max(0,-t1); d2=max(0,t2-1)+max(0,-t2);
            t(bad & d2<d1)=t2(bad & d2<d1); t=max(min(t,1),0);
            beta(lin)=atan2d(sgn(lin).*Tx.*(1-t), Ty+t.*(Ay-Ty));
        end
    else
        dy   = (r1*(2*x + r2) - 2*x^2) / D;
        Y2   = (a+x) + dy;
        X2   = sqrt(max(r2^2 - dy^2, 0));
        r_sm = r1 + r2;
        R_T  = hypot(r1*X2/r_sm, (ci*r2 + r1*Y2)/r_sm);
        arc  = R <= R_T;
        if any(arc)
            Yp = (R(arc).^2 - r1^2 + ci^2) / (2*ci);
            Xp = sqrt(max(R(arc).^2 - Yp.^2, 0));
            beta(arc) = atan2d(sgn(arc).*Xp, Yp);
        end
        r2b = ~arc;
        if any(r2b) && X2 > 1e-12
            D2   = X2^2 + Y2^2;
            K    = (D2 + R(r2b).^2 - r2^2) / 2;
            disc = max(D2.*R(r2b).^2 - K.^2, 0);
            Yp   = (K.*Y2 + X2.*sqrt(disc)) / D2;
            Xp   = (K - Y2.*Yp) / X2;
            beta(r2b) = atan2d(sgn(r2b).*Xp, Yp);
        end
    end
    theta = theta_o - beta;
end

%% =======================================================================
function [phi_LV_start, phi_RV_start] = ejection_windows(phi_deg, theta, phipk_i, gamma_i)
    seg = phi_deg <= phipk_i;
    phi_a = solve_phi_for_theta(phi_deg(seg), theta(seg), gamma_i);
    phi_LV_start = 360 - phi_a;
    seg = (phi_deg >= phipk_i) & (phi_deg <= 180);
    phi_RV_start = solve_phi_for_theta(phi_deg(seg), theta(seg), gamma_i);
end

function phi_out = solve_phi_for_theta(phi_seg, theta_seg, theta_target)
    if theta_seg(1) > theta_seg(end)
        phi_seg = fliplr(phi_seg); theta_seg = fliplr(theta_seg);
    end
    phi_out = interp1(theta_seg, phi_seg, theta_target);
end

function [Q_LV, Q_RV] = flow_rate(phi_deg, theta, phipk_i, phi_LV_start, phi_RV_start, omega_gb, K_geom, phi_t)
    dth   = gradient(theta, phi_deg);
    dth_t = interp1(phi_deg, dth, phi_t);
    dtheta_dt = omega_gb * dth_t;
    lv_mask = (phi_t >= phi_LV_start) | (phi_t <= phipk_i);
    rv_mask = (phi_t >= phi_RV_start) & (phi_t <= 360-phipk_i);
    Q_LV = max(0,  dtheta_dt) .* lv_mask * K_geom / 1000;
    Q_RV = max(0, -dtheta_dt) .* rv_mask * K_geom / 1000;
end
