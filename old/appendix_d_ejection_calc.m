%% Appendix D – Paddle Geometry and Output Calculator
%  Scotch-yoke paddle mechanism: LV ejected first half-cycle, RV second.
%  All formulas derived directly from the Appendix D spreadsheet.

clear; clc; close all;

%% ===== INPUTS ===========================================================

% ── Scotch yoke geometry ─────────────────────────────────────────────────
x   = 10;       % mm    Crank arm length
a   = 21;       % mm    Distance of motor midpoint from fulcrum

% ── Paddle geometry ──────────────────────────────────────────────────────
w   = 66;       % mm    Width of paddle  (26-ventricle design)
L   = 40.0;     % mm    Paddle length: fulcrum → tip

% ── Ventricle volumes ────────────────────────────────────────────────────
V_avg = 41.7;   % ml    Average ventricle volume (Angus current design)
EF    = 0.80;   %       Ejection fraction  (based on human heart)
AS    = 0.05;   %       Anatomical shunt (split evenly ↑LV / ↓RV)

% ── Ventricle bag ────────────────────────────────────────────────────────
h_b = 20.0;     % mm    Estimated bag height
F_e = 10.0;     % N     Estimated bag elasticity force
b   = 0.50;     %       Paddle-bag contact fraction of full cycle (contact starts at phi=0)

% ── Pressures & cardiac-output spec ──────────────────────────────────────
p      = 16;    % kPa   Max pressure in left ventricle (research value)
CO_min = 2.5;   % L/min Min cardiac output spec
CO_max = 8.0;   % L/min Max cardiac output spec

% ── Efficiencies & gearbox ───────────────────────────────────────────────
eta_mech  = 0.70;   % Mechanical — well-lubricated scotch yoke
eta_motor = 0.83;   % Brushless DC motor (worst case: 85%)
eta_gb    = 0.74;   % Gearbox
GR        = 62;      % Gear reduction ratio  ← SET THIS

%% ===== DERIVED QUANTITIES ===============================================

% ── Paddle angle calculator ───────────────────────────────────────────────
alpha     = asin(x / a);            % rad   max paddle angle
alpha_deg = rad2deg(alpha);         % deg   → should be 28.4°

% ── Paddle length calculator ──────────────────────────────────────────────
LV   = V_avg * (1 + AS/2);         % ml    LV = V*(1+AS/2)
RV   = V_avg * (1 - AS/2);         % ml    RV = V*(1-AS/2)
LVEV = LV * EF;                    % ml    LV ejection volume
RVEV = RV * EF;                    % ml    RV ejection volume

% ── Ventricle bag / force analysis ────────────────────────────────────────
% Contact area = w × h_b  (F = p×A, area assumed full paddle-bag face)
A_pad = w * h_b * 1e-6;            % m²
F_p   = p * 1e3 * A_pad;           % N     pressure force on paddle
F_max = F_p + F_e;                  % N     max force (includes bag elasticity)

% Moment arm: bag centre is at (L − h_b/2) from fulcrum
r_arm = (L - h_b/2) * 1e-3;        % m
T_p   = F_max * r_arm;             % Nm    torque on paddle

% ── Motor spec calculator ─────────────────────────────────────────────────
W_stroke = p * 1e3 * (LVEV * 1e-6);                  % J    W = p × V
rpm_min  = CO_min / (LVEV * 1e-3);                   % rpm  Min CO / LVEV
rpm_max  = CO_max / (LVEV * 1e-3);                   % rpm  Max CO / LVEV
t_cyc    = 60 / rpm_max;                             % s    period at max rpm
t_ejLV   = b * t_cyc;                                % s    LV ejection period

% Gearbox output torque T_g — slotted-arm worst case (θ=0, coupler closest to fulcrum):
%   T_g = x·T_p / (a − x)
% Motor shaft torque:  T_m = T_g / (GR · η_mech · η_gb)
T_g     = T_p * x / (a - x);                         % Nm  gearbox output torque
T_motor = T_g / (GR * eta_mech * eta_gb);            % Nm  required motor torque

% Max power: P = W/t,  t = LV ejection time,  account for efficiencies
P_max = W_stroke / t_ejLV / (eta_gb * eta_mech);     % W

%% ===== PRINT TABLE ======================================================

SEP = repmat('─', 1, 58);
fprintf('\n%s\n', SEP);
fprintf('  Appendix D – Paddle Geometry and Output Calculator\n');
fprintf('%s\n\n', SEP);

fprintf('PADDLE ANGLE CALCULATOR\n');
fprintf('  Crank arm length              x     = %6.1f  mm\n',  x);
fprintf('  Motor midpoint – fulcrum      a     = %6.1f  mm\n',  a);
fprintf('  Max paddle angle              α     = %6.1f  deg\n', alpha_deg);

fprintf('\nPADDLE LENGTH CALCULATOR\n');
fprintf('  Paddle width                  w     = %6.1f  mm\n',   w);
fprintf('  Avg ventricle volume          V_avg = %6.1f  ml\n',   V_avg);
fprintf('  Ejection fraction             EF    = %6.2f\n',       EF);
fprintf('  Anatomical shunt              AS    = %6.2f\n',       AS);
fprintf('  Left ventricle volume         LV    = %6.1f  ml   [V*(1+AS/2)]\n', LV);
fprintf('  Right ventricle volume        RV    = %6.1f  ml   [V*(1-AS/2)]\n', RV);
fprintf('  LV ejection volume            LVEV  = %6.1f  ml   [LV×EF]\n',      LVEV);
fprintf('  RV ejection volume            RVEV  = %6.1f  ml   [RV×EF]\n',      RVEV);
fprintf('  Paddle length                 L     = %6.1f  mm\n',  L);

fprintf('\nVENTRICLE BAG\n');
fprintf('  Bag height                    h_b   = %6.1f  mm\n',  h_b);
fprintf('  Bag elasticity force          F_e   = %6.1f  N\n',   F_e);
fprintf('  Contact fraction (full cycle) b     = %6.2f\n',      b);

fprintf('\nMOTOR SPEC CALCULATOR\n');
fprintf('  Max LV pressure               p     = %6.1f  kPa\n', p);
fprintf('  Paddle contact area (w×h_b)         = %6.0f  mm²\n', A_pad*1e6);
fprintf('  Pressure force on paddle      F_p   = %6.1f  N\n',   F_p);
fprintf('  Max force (+ bag elasticity)  F_max = %6.1f  N\n',   F_max);
fprintf('  Torque on paddle              T_p   = %6.2f  Nm\n',  T_p);
fprintf('  Energy per LV stroke          W     = %6.2f  J\n',   W_stroke);
fprintf('  Min cardiac output                  = %6.1f  L/min\n', CO_min);
fprintf('  Max cardiac output                  = %6.1f  L/min\n', CO_max);
fprintf('  Min RPM                             = %6.1f  rpm\n', rpm_min);
fprintf('  Max RPM                             = %6.1f  rpm\n', rpm_max);
fprintf('  Cycle period (max rpm)        t     = %6.2f  s\n',   t_cyc);
fprintf('  LV ejection period            t_LV  = %6.2f  s\n',   t_ejLV);
fprintf('  Gearbox output torque (T_g)   T_g   = %6.2f  Nm   [x·T_p/(a−x)]\n', T_motor);
fprintf('  Mechanical efficiency               = %6.2f\n',      eta_mech);
fprintf('  Motor efficiency                    = %6.2f\n',      eta_motor);
fprintf('  Gearbox efficiency                  = %6.2f\n',      eta_gb);
fprintf('  Max power                     P     = %6.1f  W\n',   P_max);
fprintf('%s\n', SEP);

%% ===== MECHANISM SIMULATION =============================================
% One full cycle at max RPM (234 rpm).
% Scotch yoke: paddle angle  φ(t) = arcsin( (x/a)·sin(ωt) )
%   First half-cycle  (ωt ∈ [0, π])   → φ positive → LV compressed
%   Second half-cycle (ωt ∈ [π, 2π])  → φ negative → RV compressed
%
% Contact threshold φ_th:
%   Time above φ_th in full cycle = b·T
%   Exact solution for scotch-yoke geometry:
%     φ_th = arcsin( (x/a)·sin(π·(0.5 − b)) )

N      = 4000;
t_sim  = linspace(0, t_cyc, N);
dt     = t_sim(2) - t_sim(1);
omega  = 2*pi / t_cyc;
theta  = omega * t_sim;                         % crank angle [0, 2π]

phi      = asin((x/a) .* sin(theta));           % paddle angle (rad)
phi_deg  = rad2deg(phi);

% Contact threshold
phi_th     = asin((x/a) * sin(pi*(0.5 - b)));  % rad
phi_th_deg = rad2deg(phi_th);

fprintf('\nContact threshold:  phi_th = %.2f deg\n', phi_th_deg);
fprintf('LV contact window:  %.1f ms – %.1f ms\n', ...
    t_sim(find(phi >  phi_th, 1     ))*1e3, ...
    t_sim(find(phi >  phi_th, 1,'last'))*1e3);
fprintf('RV contact window:  %.1f ms – %.1f ms\n', ...
    t_sim(find(phi < -phi_th, 1     ))*1e3, ...
    t_sim(find(phi < -phi_th, 1,'last'))*1e3);

% Contact masks
LV_contact = phi >  phi_th;
RV_contact = phi < -phi_th;

% Paddle angular velocity (for determining compression vs release stroke)
dphi_dt = gradient(phi, dt);                    % rad/s

% Bag compression fraction (0 to 1), clipped below threshold
LV_comp = max(0, (phi      - phi_th) / (alpha - phi_th));
RV_comp = max(0, (-phi     - phi_th) / (alpha - phi_th));

% Cumulative ejected volume — increases only during compression stroke
% (valve prevents back-flow on release stroke)
V_LV = zeros(1, N);
V_RV = zeros(1, N);
for i = 2:N
    if LV_contact(i) && dphi_dt(i) > 0
        V_LV(i) = V_LV(i-1) + LVEV * max(0, LV_comp(i) - LV_comp(i-1));
    else
        V_LV(i) = V_LV(i-1);
    end
    if RV_contact(i) && dphi_dt(i) < 0
        V_RV(i) = V_RV(i-1) + RVEV * max(0, RV_comp(i) - RV_comp(i-1));
    else
        V_RV(i) = V_RV(i-1);
    end
end

% Instantaneous flow rates
Q_LV = max(0, gradient(V_LV, dt));             % ml/s
Q_RV = max(0, gradient(V_RV, dt));             % ml/s

t_ms = t_sim * 1e3;                             % ms — for all x-axes

%% ===== FIGURE 1: Three-panel mechanism overview =========================

c_LV = [0.82 0.10 0.10];     % red  (left ventricle)
c_RV = [0.08 0.35 0.80];     % blue (right ventricle)
c_th = [0.50 0.50 0.50];     % grey (threshold lines)

fig1 = figure('Name', 'Appendix D – LV / RV Ejection', ...
    'Position', [60 60 1120 840], 'Color', 'white');
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle(sprintf(['Appendix D  –  Paddle Mechanism Simulation\n', ...
    '%.0f rpm   |   Cycle period: %.0f ms   |   LV ejection: %.0f ms   |   RV ejection: %.0f ms'], ...
    rpm_max, t_cyc*1e3, t_ejLV*1e3, b*t_cyc*1e3), ...
    'FontSize', 12, 'FontWeight', 'bold');

% ── Panel 1: Paddle angle ─────────────────────────────────────────────────
ax1 = nexttile;
hold on;

% Shaded contact windows (drawn first, behind curve)
lv_idx = find(LV_contact);
rv_idx = find(RV_contact);
if ~isempty(lv_idx)
    patch([t_ms(lv_idx), fliplr(t_ms(lv_idx))], ...
          [alpha_deg*ones(1,numel(lv_idx)), -alpha_deg*ones(1,numel(lv_idx))], ...
          c_LV, 'FaceAlpha', 0.13, 'EdgeColor', 'none');
end
if ~isempty(rv_idx)
    patch([t_ms(rv_idx), fliplr(t_ms(rv_idx))], ...
          [alpha_deg*ones(1,numel(rv_idx)), -alpha_deg*ones(1,numel(rv_idx))], ...
          c_RV, 'FaceAlpha', 0.13, 'EdgeColor', 'none');
end

plot(t_ms, phi_deg, 'k-', 'LineWidth', 2);
yline( phi_th_deg, '--', 'Color', c_th, 'LineWidth', 1.3);
yline(-phi_th_deg, '--', 'Color', c_th, 'LineWidth', 1.3);
yline( alpha_deg,  ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 0.9);
yline(-alpha_deg,  ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 0.9);
text(2,  phi_th_deg+1.2, sprintf('+%.1f° contact threshold', phi_th_deg), ...
    'FontSize',8,'Color',c_th);
text(2, -phi_th_deg-2.2, sprintf('−%.1f° contact threshold', phi_th_deg), ...
    'FontSize',8,'Color',c_th);

legend({'LV contact region','RV contact region','Paddle angle'}, ...
    'Location','northeast','FontSize',8);
ylabel('Paddle Angle (°)');
title(sprintf('Paddle Angle  |  Max = ±%.1f°   Contact threshold = ±%.1f°', ...
    alpha_deg, phi_th_deg));
ylim([-alpha_deg*1.35, alpha_deg*1.35]);
grid on;

% ── Panel 2: Instantaneous flow rate ──────────────────────────────────────
ax2 = nexttile;
hold on;
area(t_ms, Q_LV, 'FaceColor', c_LV, 'FaceAlpha', 0.70, 'EdgeColor', 'none');
area(t_ms, Q_RV, 'FaceColor', c_RV, 'FaceAlpha', 0.70, 'EdgeColor', 'none');
legend({sprintf('LV  (LVEV = %.1f mL, EF = %.0f%%)', LVEV, EF*100), ...
        sprintf('RV  (RVEV = %.1f mL, EF = %.0f%%)', RVEV, EF*100)}, ...
    'Location','northeast','FontSize',9);
ylabel('Flow Rate (mL/s)');
title('Instantaneous Ejection Flow Rate');
grid on;

% ── Panel 3: Cumulative ejected volume ────────────────────────────────────
ax3 = nexttile;
hold on;
plot(t_ms, V_LV, '-', 'Color', c_LV, 'LineWidth', 2.5);
plot(t_ms, V_RV, '-', 'Color', c_RV, 'LineWidth', 2.5);
yline(LVEV, '--', 'Color', c_LV, 'LineWidth', 1.2, ...
    'Label', sprintf('LVEV = %.1f mL', LVEV), ...
    'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'top');
yline(RVEV, '--', 'Color', c_RV, 'LineWidth', 1.2, ...
    'Label', sprintf('RVEV = %.1f mL', RVEV), ...
    'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'bottom');
legend({sprintf('Left Ventricle  →  %.1f mL ejected', V_LV(end)), ...
        sprintf('Right Ventricle →  %.1f mL ejected', V_RV(end))}, ...
    'Location','east','FontSize',9);
ylabel('Volume Ejected (mL)');
xlabel('Time (ms)');
title('Cumulative Volume Ejected per Cycle');
ylim([0, max(LVEV, RVEV) * 1.30]);
grid on;

linkaxes([ax1, ax2, ax3], 'x');
xlim(ax1, [0, t_cyc*1e3]);

%% ===== FIGURE 2: Ejection timeline ======================================

t_LV1 = t_ms(find(LV_contact, 1));
t_LV2 = t_ms(find(LV_contact, 1, 'last'));
t_RV1 = t_ms(find(RV_contact, 1));
t_RV2 = t_ms(find(RV_contact, 1, 'last'));
t_end  = t_cyc * 1e3;

figure('Name', 'LV / RV Ejection Timeline', ...
    'Position', [60 930 1120 230], 'Color', 'white');
hold on;

% Background (full cycle)
patch([0, t_end, t_end, 0], [0, 0, 1, 1], ...
    [0.92 0.92 0.92], 'EdgeColor', [0.6 0.6 0.6], 'LineWidth', 1.5);

% LV ejection bar
patch([t_LV1, t_LV2, t_LV2, t_LV1], [0.08, 0.08, 0.92, 0.92], ...
    c_LV, 'FaceAlpha', 0.82, 'EdgeColor', c_LV*0.65, 'LineWidth', 1.5);
text((t_LV1+t_LV2)/2, 0.50, ...
    sprintf('LV  Ejection\n%.1f → %.1f ms\n(%.1f ms)', t_LV1, t_LV2, t_LV2-t_LV1), ...
    'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'FontSize',10,'FontWeight','bold','Color','white');

% RV ejection bar
patch([t_RV1, t_RV2, t_RV2, t_RV1], [0.08, 0.08, 0.92, 0.92], ...
    c_RV, 'FaceAlpha', 0.82, 'EdgeColor', c_RV*0.65, 'LineWidth', 1.5);
text((t_RV1+t_RV2)/2, 0.50, ...
    sprintf('RV  Ejection\n%.1f → %.1f ms\n(%.1f ms)', t_RV1, t_RV2, t_RV2-t_RV1), ...
    'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'FontSize',10,'FontWeight','bold','Color','white');

% Gap labels (no-contact windows)
if t_LV1 > 1
    text(t_LV1/2, 0.50, sprintf('gap\n%.1f ms', t_LV1), ...
        'HorizontalAlignment','center','FontSize',8,'Color',[0.4 0.4 0.4]);
end
if t_RV1 - t_LV2 > 1
    text((t_LV2+t_RV1)/2, 0.50, sprintf('gap\n%.1f ms', t_RV1-t_LV2), ...
        'HorizontalAlignment','center','FontSize',8,'Color',[0.4 0.4 0.4]);
end
if t_end - t_RV2 > 1
    text((t_RV2+t_end)/2, 0.50, sprintf('gap\n%.1f ms', t_end-t_RV2), ...
        'HorizontalAlignment','center','FontSize',8,'Color',[0.4 0.4 0.4]);
end

xlabel('Time (ms)', 'FontSize', 11);
title(sprintf(['LV and RV Ejection Timeline  |  One Cycle = %.0f ms (%.0f rpm)  ', ...
    '|  LV–RV delay = %.0f ms  |  Contact fraction b = %.2f'], ...
    t_end, rpm_max, t_RV1-t_LV1, b), ...
    'FontSize', 11, 'FontWeight', 'bold');
xlim([0, t_end]);
ylim([0, 1]);
set(gca, 'YTick', [], 'XGrid', 'on', 'GridLineStyle', '--', 'GridAlpha', 0.5);

%% ===== FIGURE 3: Parameter summary table ================================

figure('Name','Appendix D – Summary', 'Position',[1200 60 460 680],'Color','white');
axis off;
title('Appendix D  –  Parameter Summary', 'FontSize', 12, 'FontWeight', 'bold');

params = {
    'PADDLE ANGLE CALCULATOR', '', '';
    'Crank arm length',            sprintf('x = %.0f mm',    x),         '';
    'Motor midpoint–fulcrum',      sprintf('a = %.0f mm',    a),         '';
    'Max paddle angle',            sprintf('α = %.1f°',      alpha_deg), '← asin(x/a)';
    '', '', '';
    'PADDLE LENGTH CALCULATOR', '', '';
    'Avg ventricle volume',        sprintf('V = %.1f ml',    V_avg),     '';
    'Ejection fraction',           sprintf('EF = %.2f',      EF),        '';
    'Anatomical shunt',            sprintf('AS = %.2f',      AS),        '';
    'Left ventricle vol',          sprintf('LV = %.1f ml',   LV),        '← V(1+AS/2)';
    'Right ventricle vol',         sprintf('RV = %.1f ml',   RV),        '← V(1−AS/2)';
    'LV ejection volume',          sprintf('LVEV = %.1f ml', LVEV),      '← LV × EF';
    'RV ejection volume',          sprintf('RVEV = %.1f ml', RVEV),      '← RV × EF';
    'Paddle length',               sprintf('L = %.1f mm',    L),         '';
    '', '', '';
    'VENTRICLE BAG', '', '';
    'Bag height',                  sprintf('h_b = %.1f mm',  h_b),       '';
    'Bag elasticity force',        sprintf('F_e = %.1f N',   F_e),       '';
    'Contact fraction',            sprintf('b = %.2f',       b),         '';
    '', '', '';
    'MOTOR SPEC CALCULATOR', '', '';
    'LV pressure',                 sprintf('p = %.0f kPa',   p),         '';
    'Pressure force',              sprintf('F_p = %.1f N',   F_p),       '← p × w × h_b';
    'Max force',                   sprintf('F_max = %.1f N', F_max),     '← F_p + F_e';
    'Torque on paddle',            sprintf('T_p = %.2f Nm',  T_p),       '← F_max × (L−h_b/2)';
    'Energy per stroke',           sprintf('W = %.2f J',     W_stroke),  '← p × LVEV';
    'Min RPM',                     sprintf('%.1f rpm',       rpm_min),   '← CO_min / LVEV';
    'Max RPM',                     sprintf('%.1f rpm',       rpm_max),   '← CO_max / LVEV';
    'Cycle period',                sprintf('t = %.2f s',     t_cyc),     '← 60/rpm_max';
    'LV ejection period',          sprintf('t_LV = %.2f s',  t_ejLV),    '← b × t';
    'Gearbox output torque (T_g)', sprintf('T_g = %.2f Nm',  T_motor),   '← x·T_p/(a−x)';
    'Mechanical efficiency',       sprintf('η_m = %.2f',     eta_mech),  '';
    'Motor efficiency',            sprintf('η_mot = %.2f',   eta_motor), '';
    'Gearbox efficiency',          sprintf('η_gb = %.2f',    eta_gb),    '';
    'Max power',                   sprintf('P = %.1f W',     P_max),     '← W/t_LV/(η_m η_gb)';
};

n_rows = size(params, 1);
y_step = 1 / (n_rows + 2);
for k = 1:n_rows
    y = 1 - k * y_step;
    label = params{k, 1};
    value = params{k, 2};
    note  = params{k, 3};

    if isempty(value)
        % Section header
        text(0.02, y, label, 'FontSize', 9, 'FontWeight', 'bold', ...
            'Color', [0.2 0.2 0.6], 'Units', 'normalized');
    else
        text(0.02, y, label, 'FontSize', 8.5, 'Units', 'normalized');
        text(0.52, y, value, 'FontSize', 8.5, 'FontWeight', 'bold', ...
            'Color', [0.1 0.45 0.1], 'Units', 'normalized');
        if ~isempty(note)
            text(0.75, y, note, 'FontSize', 7.5, 'Color', [0.5 0.5 0.5], ...
                'Units', 'normalized');
        end
    end
end

%% ===== FIGURE 4: Pressure, Force & Motor Torque vs Crank Angle ==========
% Ventricular pressures from Table 3.
%   0°–90°   = LV systole    (paddle compresses LV bag)
%   90°–180° = LV diastole   (paddle returns, LV refills — pressure decays to 0)
%   180°–270° = RV systole   (paddle compresses RV bag)
%   270°–360° = RV diastole  (paddle returns, RV refills — pressure decays to 0)
%
% Systole profile: 0–10% IVC ramp, 10–40% rapid ejection, 40–100% reduced ejection
% Diastole profile: exponential decay to 0  (matches TBH27_archived convention)

mmHg  = 133.322;   % Pa per mmHg
k_dia = 15;        % fast exponential decay — matches Wiggers diagram (near-vertical isovolumic relaxation)

% ── Pressures from Table 3 ────────────────────────────────────────────────
P_LV_open  =  80 * mmHg;
P_LV_peak  = 120 * mmHg;
P_LV_close = 100 * mmHg;

P_RV_open  = 15 * mmHg;
P_RV_peak  = 30 * mmHg;
P_RV_close = 15 * mmHg;

% ── Crank angle axis ──────────────────────────────────────────────────────
theta_deg = rad2deg(theta);   % 0° to 360°

% Quarter-cycle masks
lv_sys_m = theta_deg >= 0   & theta_deg <= 90;
lv_dia_m = theta_deg > 90  & theta_deg <= 180;
rv_sys_m = theta_deg > 180 & theta_deg <= 270;
rv_dia_m = theta_deg > 270 & theta_deg <= 360;

% ── Pressure profiles ─────────────────────────────────────────────────────
P_LV_prof = zeros(1, N);
P_RV_prof = zeros(1, N);

% LV systole (0°–90°)
nu = theta_deg(lv_sys_m) / 90;
Plv = zeros(1, sum(lv_sys_m));
Plv(nu < 0.10) = P_LV_open .* nu(nu < 0.10) / 0.10;
s2 = nu >= 0.10 & nu < 0.40;
Plv(s2) = P_LV_open  + (P_LV_peak  - P_LV_open ) .* (nu(s2) - 0.10) / 0.30;
s3 = nu >= 0.40;
Plv(s3) = P_LV_peak  + (P_LV_close - P_LV_peak ) .* (nu(s3) - 0.40) / 0.60;
P_LV_prof(lv_sys_m) = Plv;

% LV diastole (90°–180°): exponential decay P_LV_close → 0
tau = (theta_deg(lv_dia_m) - 90) / 90;
P_LV_prof(lv_dia_m) = P_LV_close * (exp(-k_dia*tau) - exp(-k_dia)) / (1 - exp(-k_dia));

% RV systole (180°–270°)
nu = (theta_deg(rv_sys_m) - 180) / 90;
Prv = zeros(1, sum(rv_sys_m));
Prv(nu < 0.10) = P_RV_open .* nu(nu < 0.10) / 0.10;
s2 = nu >= 0.10 & nu < 0.40;
Prv(s2) = P_RV_open  + (P_RV_peak  - P_RV_open ) .* (nu(s2) - 0.10) / 0.30;
s3 = nu >= 0.40;
Prv(s3) = P_RV_peak  + (P_RV_close - P_RV_peak ) .* (nu(s3) - 0.40) / 0.60;
P_RV_prof(rv_sys_m) = Prv;

% RV diastole (270°–360°): exponential decay P_RV_close → 0
tau = (theta_deg(rv_dia_m) - 270) / 90;
P_RV_prof(rv_dia_m) = P_RV_close * (exp(-k_dia*tau) - exp(-k_dia)) / (1 - exp(-k_dia));

% ── Forces — active only during systole (matches TBH27: F = sys_idx .* P * A) ──
% F_LV and F_RV are pure pressure forces; F_e added once to total only
F_LV_prof  = P_LV_prof * A_pad .* double(lv_sys_m);
F_RV_prof  = P_RV_prof * A_pad .* double(rv_sys_m);
F_pad_prof = F_LV_prof + F_RV_prof + F_e * double(lv_sys_m | rv_sys_m);

% ── Gearbox output torque — slotted-arm kinematic formula (virtual work) ──
% Crank pin slides in radial slot along paddle arm.
% Virtual work: T_crank*dθ = T_paddle*dφ  →
%   T_g(θ) = T_pad(θ) * x*(a*cos(θ)-x) / (a²+x²-2ax*cos(θ))
% Positive (compression) for θ < arccos(x/a) ≈ 61.6°  (arm moving toward max deflection)
% Zero at θ = arccos(x/a) ≈ 61.6°  (arm at dead centre — max paddle deflection)
% Negative beyond 61.6° (arm returning) — clamped to zero as bag pressure assists return
% Worst case at θ=0: T_g = x*T_p/(a-x)  [matches report formula]
T_pad_prof   = F_pad_prof * r_arm;
kine_ratio   = max(0, x * (a * cos(theta) - x) ./ (a^2 + x^2 - 2*a*x*cos(theta)));
T_g_prof     = T_pad_prof .* kine_ratio;                      % Nm  gearbox output torque
T_motor_prof = T_g_prof / (GR * eta_mech * eta_gb);           % Nm  motor shaft torque  T_m = T_g/(GR·η_mech·η_gb)

fprintf('\nMotor torque T_m = T_g/(GR·η_mech·η_gb)  [GR=%g]:\n', GR);
fprintf('  LV peak T_m: %.3f Nm  (systole 0°–90°)\n',   max(T_motor_prof(lv_sys_m)));
fprintf('  RV peak T_m: %.3f Nm  (systole 180°–270°)\n', max(T_motor_prof(rv_sys_m)));
fprintf('  Worst-case static T_m (T_g worst / GR·η): %.3f Nm\n', T_motor);

% ── Figure ────────────────────────────────────────────────────────────────
figure('Name', 'Pressure, Force & Motor Torque vs Crank Angle', ...
    'Position', [60 60 1100 760], 'Color', 'white');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle(sprintf(['Paddle Forces & Motor Torque vs Crank Angle  |  %.0f rpm\n', ...
    '0°/180° = neutral   |   0°–90° = LV systole   |   90°–180° = LV diastole   |   180°–270° = RV systole   |   270°–360° = RV diastole'], ...
    rpm_max), 'FontSize', 10, 'FontWeight', 'bold');

crank_ticks  = 0:90:360;
crank_labels = {'0°  (neutral)','90°  (LV max)','180°  (neutral)','270°  (RV max)','360°'};

c_LV_dia = c_LV * 0.55 + [1 1 1] * 0.45;   % lighter red for diastole shading
c_RV_dia = c_RV * 0.55 + [1 1 1] * 0.45;   % lighter blue

% ── Panel 1: Pressure ────────────────────────────────────────────────────
nexttile;
plot(theta_deg, P_LV_prof/mmHg, '-', 'Color', c_LV, 'LineWidth', 2.2, ...
    'DisplayName', 'LV  (open 80 / peak 120 / close 100 mmHg)'); hold on;
plot(theta_deg, P_RV_prof/mmHg, '-', 'Color', c_RV, 'LineWidth', 2.2, ...
    'DisplayName', 'RV  (open 15 / peak 30 / close 15 mmHg)');
ylabel('Pressure (mmHg)');
title('Ventricular Pressure Profiles  (Table 3)  —  systole + diastolic exponential decay');
legend('Location','northeast','FontSize',9);
xlim([0 360]); xticks(crank_ticks); xticklabels(crank_labels); grid on;
yl = ylim;
patch([0   90  90   0],  [yl(1) yl(1) yl(2) yl(2)], c_LV,     'FaceAlpha',0.12,'EdgeColor','none','HandleVisibility','off');
patch([90  180 180  90], [yl(1) yl(1) yl(2) yl(2)], c_LV_dia,  'FaceAlpha',0.10,'EdgeColor','none','HandleVisibility','off');
patch([180 270 270 180], [yl(1) yl(1) yl(2) yl(2)], c_RV,     'FaceAlpha',0.12,'EdgeColor','none','HandleVisibility','off');
patch([270 360 360 270], [yl(1) yl(1) yl(2) yl(2)], c_RV_dia,  'FaceAlpha',0.10,'EdgeColor','none','HandleVisibility','off');
ylim(yl);
for dg = [0 90 180 270 360]
    xline(dg, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
end
uistack(findobj(gca,'Type','Patch'), 'bottom');
text(45,  max(P_LV_prof/mmHg)*0.55, 'LV systole',  'Color',c_LV*0.7,'FontSize',9,'FontWeight','bold','HorizontalAlignment','center');
text(135, max(P_LV_prof/mmHg)*0.55, 'LV diastole', 'Color',c_LV*0.7,'FontSize',9,'HorizontalAlignment','center');
text(225, max(P_LV_prof/mmHg)*0.55, 'RV systole',  'Color',c_RV*0.7,'FontSize',9,'FontWeight','bold','HorizontalAlignment','center');
text(315, max(P_LV_prof/mmHg)*0.55, 'RV diastole', 'Color',c_RV*0.7,'FontSize',9,'HorizontalAlignment','center');

% ── Panel 2: Force on paddle ─────────────────────────────────────────────
nexttile;
F_elast_prof = F_e * double(lv_sys_m | rv_sys_m);
plot(theta_deg, F_LV_prof,    '-',  'Color', c_LV,           'LineWidth', 2.0, 'DisplayName', 'F_{LV}  (pressure only)'); hold on;
plot(theta_deg, F_RV_prof,    '-',  'Color', c_RV,           'LineWidth', 2.0, 'DisplayName', 'F_{RV}  (pressure only)');
plot(theta_deg, F_elast_prof, '--', 'Color', [0.85 0.55 0.0], 'LineWidth', 1.8, 'DisplayName', sprintf('F_{bag elasticity} = %.0f N', F_e));
plot(theta_deg, F_pad_prof,   'k-', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('F_{total} = F_{LV} + F_{RV} + F_e (%.0f N)', F_e));
yline(F_max, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.2, ...
    'Label', sprintf('Appendix D F_{max} = %.1f N', F_max), ...
    'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');
ylabel('Force on Paddle (N)');
title('Force on Paddle vs Crank Angle  (zero during diastole)');
legend('Location','northeast','FontSize',9);
xlim([0 360]); xticks(crank_ticks); xticklabels(crank_labels); grid on;
yl = ylim;
patch([0   90  90   0],  [yl(1) yl(1) yl(2) yl(2)], c_LV,    'FaceAlpha',0.12,'EdgeColor','none','HandleVisibility','off');
patch([90  180 180  90], [yl(1) yl(1) yl(2) yl(2)], c_LV_dia, 'FaceAlpha',0.10,'EdgeColor','none','HandleVisibility','off');
patch([180 270 270 180], [yl(1) yl(1) yl(2) yl(2)], c_RV,    'FaceAlpha',0.12,'EdgeColor','none','HandleVisibility','off');
patch([270 360 360 270], [yl(1) yl(1) yl(2) yl(2)], c_RV_dia, 'FaceAlpha',0.10,'EdgeColor','none','HandleVisibility','off');
ylim(yl);
for dg = [0 90 180 270 360]
    xline(dg, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
end
uistack(findobj(gca,'Type','Patch'), 'bottom');

% ── Panel 3: Motor torque ─────────────────────────────────────────────────
nexttile;
plot(theta_deg, T_motor_prof * 1e3, '-', 'Color', [0.49 0.18 0.56], ...
    'LineWidth', 2.5, 'DisplayName', 'Required motor torque'); hold on;
yline(T_motor * 1e3, 'k--', 'LineWidth', 1.2, ...
    'Label', sprintf('Worst case T_m = %.2f Nm  [T_g/(GR·η_m·η_gb), GR=%g]', T_motor, GR), ...
    'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');
ylabel('Motor Torque (mNm)');
xlabel('Crank Angle  —  0°/180° = paddle neutral   |   90° = max LV deflection   |   270° = max RV deflection');
title(sprintf('Required Motor Torque  T_m = T_g/(GR·\\eta_m·\\eta_{gb})  |  GR=%g  |  LV peak %.0f mNm   RV peak %.0f mNm', ...
    GR, max(T_motor_prof(lv_sys_m))*1e3, max(T_motor_prof(rv_sys_m))*1e3));
legend('Location','northeast','FontSize',9);
xlim([0 360]); xticks(crank_ticks); xticklabels(crank_labels); grid on;
yl = ylim;
patch([0   90  90   0],  [yl(1) yl(1) yl(2) yl(2)], c_LV,    'FaceAlpha',0.12,'EdgeColor','none','HandleVisibility','off');
patch([90  180 180  90], [yl(1) yl(1) yl(2) yl(2)], c_LV_dia, 'FaceAlpha',0.10,'EdgeColor','none','HandleVisibility','off');
patch([180 270 270 180], [yl(1) yl(1) yl(2) yl(2)], c_RV,    'FaceAlpha',0.12,'EdgeColor','none','HandleVisibility','off');
patch([270 360 360 270], [yl(1) yl(1) yl(2) yl(2)], c_RV_dia, 'FaceAlpha',0.10,'EdgeColor','none','HandleVisibility','off');
ylim(yl);
for dg = [0 90 180 270 360]
    xline(dg, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
end
uistack(findobj(gca,'Type','Patch'), 'bottom');
