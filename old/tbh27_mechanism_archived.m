clear; clc; close all;

%% ── EXPORT OPTIONS ───────────────────────────────────────────────────────────
EXPORT_FIGS  = true;
EXPORT_DPI   = 300;
EXPORT_FMT   = 'png';
EXPORT_DIR   = fullfile(fileparts(mfilename('fullpath')), 'figures');

export_flags = struct(...
    'spring_sweep',  true,  ...
    'power_cycle',   true,  ...
    'kinematics',    true,  ...
    'forces',        true,  ...
    'disp_pres',     true,  ...
    'motor_spec',    true,  ...
    'bpm_sweep',     true  );

%% ── MOTOR DATASHEET INPUT ────────────────────────────────────────────────────
motor.name          = 'Maxon ECX FLAT 22 S 14.9mNm';
motor.V_nom         = 12;           % V      — nominal voltage
motor.RPM_noload    = 13300;        % rpm    — no-load speed
motor.RPM_nominal   = 10300;         % rpm    — nominal (rated) speed
motor.RPM_max       = 15000;        % rpm    — max permissible speed
motor.T_nominal     = 14.9e-3;      % N·m    — nominal (continuous) torque
motor.T_stall       = 64.1e-3;      % N·m    — stall torque
motor.P_rated       = 17;           % W      — assigned rated (continuous shaft) power
motor.eta           = 0.81;         % —      — motor efficiency at nominal point
motor.I_nominal     = 1.74;        % A      — nominal (max continuous) current
motor.I_stall       = 11.1;         % A      — stall current
motor.T_const       = 8.47e-3;      % N·m/A  — torque constant
motor.RPM_per_V     = 1130;          % rpm/V  — speed constant

%% ── GEARBOX DATASHEET INPUT ──────────────────────────────────────────────────
gearbox.name        = 'Maxon Planetary GPX 22 UP 3-stage';
gearbox.N           = 62;           % —      — gear reduction ratio
gearbox.T_cont      = 4.3;          % N·m    — max continuous output torque
gearbox.T_intermit  = 5.3;          % N·m    — max intermittent output torque
gearbox.P_cont      = 20;            % W      — max continuous transmittable power
gearbox.P_intermit  = 25;          % W      — max intermittent transmittable power
gearbox.eta         = 0.9;         % —      — gearbox efficiency

N_gb_check          = gearbox.N;    % gear ratio to evaluate

% Combined drivetrain efficiency — single source of truth for all power calcs
eta_drive = motor.eta * gearbox.eta;

%% ── PARAMETERS ───────────────────────────────────────────────────────────────
HR       = 120;
r_c      = 0.00945;
L_rod    = 0.022;
D_plate  = 0.075;
SF       = 0.5;

P_dia    = 80  * 133.322;
P_sys    = 120 * 133.322;
P_end    = 100 * 133.322;
dP_bag   = 10  * 133.322;
F_para   = 5;
m_mover  = 0.2;
WPT_lim  = 10;
k_spring = 3468;%From real spring LC035D 13 S316
P_budget = 15.6;

f       = HR/60; T = 1/f;
T_sys   = SF*T;  T_dia = (1-SF)*T;
stroke  = 2*r_c;
A_plate = pi/4 * D_plate^2;
lambda  = r_c / L_rod;
n_sc    = 1 / lambda;    % slider-crank ratio L_rod/r_c — used in torque formula

fprintf('=== TBH27 Piston/Crank Analysis ===\n');
fprintf('HR: %d bpm | SF: %.2f | stroke: %.1f mm | lambda: %.4f | n_sc: %.4f\n', ...
    HR, SF, stroke*1e3, lambda, n_sc);
fprintf('Motor: %s | eta_mot=%.2f\n', motor.name, motor.eta);
fprintf('Gearbox: %s N=%d | eta_gb=%.2f\n', gearbox.name, gearbox.N, gearbox.eta);
fprintf('Combined eta_drive=%.4f\n', eta_drive);

%% ── TIME, CRANK ANGLE & PHASE MASKS ─────────────────────────────────────────
N_pts   = 4000; dt = T/N_pts;
t       = [(0:N_pts-1)*dt, (0:N_pts-1)*dt + T];
t_cyc   = mod(t, T);
n_cyc   = floor(t / T);
sys_idx = t_cyc < T_sys;

omega_sys = pi/T_sys;
omega_dia = pi/T_dia;
theta = n_cyc*2*pi + sys_idx.*(omega_sys*t_cyc) + ~sys_idx.*(pi + omega_dia*(t_cyc - T_sys));

%% ── KINEMATICS ───────────────────────────────────────────────────────────────
omega_shaft           = zeros(size(t));
omega_shaft(sys_idx)  = omega_sys;
omega_shaft(~sys_idx) = omega_dia;

x_p     = r_c*(1 - cos(theta)) + L_rod*(1 - sqrt(1 - lambda^2*sin(theta).^2));
dxdth   = r_c*sin(theta) + r_c*lambda^2*sin(theta).*cos(theta) ./ sqrt(1 - lambda^2*sin(theta).^2);
v_p     = dxdth .* omega_shaft;
d2xdth2 = r_c*cos(theta) ...
        + r_c*lambda.*(cos(theta).^2 - sin(theta).^2) ./ sqrt(1 - lambda^2*sin(theta).^2) ...
        + r_c*lambda^3.*sin(theta).^2.*cos(theta).^2  ./ (1 - lambda^2*sin(theta).^2).^(3/2);
a_p     = d2xdth2 .* omega_shaft.^2;

%% ── PRESSURE PROFILE ─────────────────────────────────────────────────────────
P_phys = zeros(size(t));
t_ivc  = 0.10*T_sys;

P_phys(sys_idx & t_cyc < t_ivc) = P_dia * t_cyc(sys_idx & t_cyc < t_ivc) / t_ivc;
ej1 = sys_idx & t_cyc >= t_ivc & t_cyc < 0.40*T_sys;
P_phys(ej1) = P_dia + (P_sys-P_dia) * (t_cyc(ej1)-t_ivc) / (0.30*T_sys);
ej2 = sys_idx & t_cyc >= 0.40*T_sys;
P_phys(ej2) = P_sys + (P_end-P_sys) * (t_cyc(ej2)-0.40*T_sys) / (0.60*T_sys);
% Normalised exponential: starts at P_end, reaches exactly 0 at end of diastole
k_dia = 3;
tau_dia = (t_cyc(~sys_idx) - T_sys) / T_dia;
P_phys(~sys_idx) = P_end * (exp(-k_dia*tau_dia) - exp(-k_dia)) / (1 - exp(-k_dia));

%% ── FORCES ───────────────────────────────────────────────────────────────────
F_fluid    = sys_idx .* P_phys * A_plate;
F_bag_wall = sys_idx .* dP_bag * A_plate;
F_inert    = m_mover * abs(a_p);
F_total    = F_fluid + F_bag_wall + F_inert + F_para;

fprintf('\nPeak F_fluid: %.1f N | Peak F_inertia: %.1f N | Peak F_total: %.1f N\n', ...
    max(F_fluid), max(F_inert), max(F_total));

%% ── SPRING ───────────────────────────────────────────────────────────────────
sign_factor           = ones(size(t));
sign_factor(sys_idx)  = -1;

F_spring = k_spring * (stroke - x_p);
F_motor  = F_total + sign_factor .* F_spring;

if k_spring > 0
    E_spring = 0.5 * k_spring * stroke^2;
    fprintf('\nSpring: k=%.1f N/m | E_stored=%.2f mJ/cycle | F_max=%.1f N\n', ...
        k_spring, E_spring*1e3, k_spring*stroke);
end

%% ── TORQUE HELPER ────────────────────────────────────────────────────────────
% Velocity pole slider-crank torque formula — single definition used everywhere:
%   T = F * r_c * |sin(theta) + sin(2*theta) / (2*sqrt(n_sc^2 - sin^2(theta)))|
% n_sc = L_rod/r_c > 1 always, so sqrt argument is always positive.
crank_torque = @(F, th) F .* r_c .* abs( sin(th) + ...
    sin(2*th) ./ (2 * sqrt(n_sc^2 - sin(th).^2)) );

%% ── SHAFT TORQUE & ELECTRICAL POWER ─────────────────────────────────────────
T_shaft      = crank_torque(F_motor,  theta);
T_shaft_base = crank_torque(F_total,  theta);
P_elec       = T_shaft      .* omega_shaft / eta_drive;
P_elec_base  = T_shaft_base .* omega_shaft / eta_drive;

fprintf('\nPeak T_shaft: %.4f N·m | Peak P_elec: %.2f W | Mean P_elec: %.2f W | WPT %s\n', ...
    max(T_shaft), max(P_elec), mean(P_elec), pass_fail(mean(P_elec), WPT_lim));
if k_spring > 0
    fprintf('vs no spring:  Peak P_elec: %.2f W (%.0f%% change)\n', ...
        max(P_elec_base), 100*(max(P_elec)-max(P_elec_base))/max(P_elec_base));
end

%% ── OPTIMUM SPRING STIFFNESS SWEEP ──────────────────────────────────────────
k_max        = 2 * max(F_total) / stroke;
k_sweep      = linspace(0, k_max, 500);
P_peak_sweep = zeros(size(k_sweep));

for ki = 1:numel(k_sweep)
    F_sp_i            = k_sweep(ki) * (stroke - x_p);
    F_mot_i           = F_total + sign_factor .* F_sp_i;
    T_sh_i            = crank_torque(F_mot_i, theta);     % ← velocity pole formula
    P_peek_i          = T_sh_i .* omega_shaft / eta_drive;
    P_peak_sweep(ki)  = max(P_peek_i);
end

[P_peak_opt, idx_opt] = min(P_peak_sweep);
k_opt = k_sweep(idx_opt);

fprintf('\n--- Optimum spring ---\n');
fprintf('k_opt = %.1f N/m | Peak P_elec = %.2f W (vs %.2f W no spring, %.0f%% reduction)\n', ...
    k_opt, P_peak_opt, max(P_elec_base), 100*(max(P_elec_base)-P_peak_opt)/max(P_elec_base));
fprintf('Spring F_max at BDC = %.1f N | E_stored = %.2f mJ/cycle\n', ...
    k_opt*stroke, 0.5*k_opt*stroke^2*1e3);

fig_spring = figure('Position',[50 50 700 320],'Color','white');
plot(k_sweep, P_peak_sweep, 'b-', 'LineWidth', 2); hold on;
plot(k_opt, P_peak_opt, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
yline(max(P_elec_base), 'k--', sprintf('No spring: %.2f W', max(P_elec_base)), ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');
xline(k_opt, 'r--', sprintf('k_{opt} = %.1f N/m', k_opt), ...
    'LineWidth', 1.2, 'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
xlabel('Spring stiffness k (N/m)');
ylabel('Peak P_{elec} (W)');
title(sprintf('Peak electrical power vs spring stiffness — optimum k = %.1f N/m', k_opt));
legend('Peak P_{elec}', sprintf('Optimum (%.2f W)', P_peak_opt), 'Location', 'northeast');
grid on;

%% ── POWER PLOT ───────────────────────────────────────────────────────────────
theta_deg = rad2deg(mod(theta, 2*pi)) + (n_cyc * 360);

fig_power = figure('Position',[50 50 1000 380],'Color','white');
plot(theta_deg, P_elec, 'r-', 'LineWidth', 2, 'DisplayName', 'With spring'); hold on;
if k_spring > 0
    plot(theta_deg, P_elec_base, 'b--', 'LineWidth', 1.5, 'DisplayName', 'No spring');
    legend('Location','northeast','FontSize',8);
end
yline(max(P_elec),  'r--', sprintf('Peak: %.2f W',  max(P_elec)),  'LineWidth',1.2,'LabelHorizontalAlignment','right','HandleVisibility','off');
yline(mean(P_elec), 'k--', sprintf('Mean: %.2f W',  mean(P_elec)), 'LineWidth',1.2,'LabelHorizontalAlignment','right','HandleVisibility','off');
yline(WPT_lim,      'b--', sprintf('WPT: %g W', WPT_lim),          'LineWidth',1.5,'LabelHorizontalAlignment','right','HandleVisibility','off');
yline(0, 'k:', 'LineWidth',0.8,'HandleVisibility','off');

yl = ylim;
for n = 0:1
    patch([n*360 n*360 n*360+180 n*360+180],[yl(1) yl(2) yl(2) yl(1)], ...
        [0.9 0.85 0.85],'FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
end
for deg = [90 180 270 360 450 540 630]
    if mod(deg,360) == 0;      lbl = 'BDC';
    elseif deg==180||deg==540; lbl = 'TDC';
    else;                      lbl = '';
    end
    xline(deg,'k:',lbl,'LabelVerticalAlignment','bottom','HandleVisibility','off');
end

ylabel('P_{elec} (W)');
xlabel('Crank angle (°)  —  0°/360°=BDC, 180°=TDC');
title(sprintf('Motor electrical power — HR=%d bpm | \\eta_{drive}=%.3f | Peak=%.2f W | k_{spring}=%.0f N/m', ...
    HR, eta_drive, max(P_elec), k_spring));
xticks(0:90:720); grid on; xlim([0 720]);

%% ── PISTON KINEMATICS FIGURE ─────────────────────────────────────────────────
fig_kin = figure('Position', [50 50 1000 600], 'Color', 'white');

subplot(3,1,1);
plot(theta_deg, x_p*1e3, 'b-', 'LineWidth', 1.8);
ylabel('Displacement (mm)');
title(sprintf('Piston kinematics — HR=%d bpm | stroke=%.1f mm | \\lambda=%.4f | SF=%.2f', ...
    HR, stroke*1e3, lambda, SF));
xticks(0:90:720); xlim([0 720]); grid on;
for deg = [90 180 270 360 450 540 630]
    if mod(deg,360)==0; lbl='BDC'; elseif deg==180||deg==540; lbl='TDC'; else; lbl=''; end
    xline(deg,'k:',lbl,'LabelVerticalAlignment','bottom','HandleVisibility','off');
end
for n = 0:1
    yl = ylim;
    patch([n*360 n*360 n*360+180 n*360+180],[yl(1) yl(2) yl(2) yl(1)], ...
        [0.85 0.9 0.85],'FaceAlpha',0.25,'EdgeColor','none','HandleVisibility','off');
end
subplot(3,1,2);
plot(theta_deg, v_p*1e3, 'g-', 'LineWidth', 1.8);
ylabel('Velocity (mm/s)');
yline(0,'k:','LineWidth',0.8,'HandleVisibility','off');
xticks(0:90:720); xlim([0 720]); grid on;
for deg = [90 180 270 360 450 540 630]
    if mod(deg,360)==0; lbl='BDC'; elseif deg==180||deg==540; lbl='TDC'; else; lbl=''; end
    xline(deg,'k:',lbl,'LabelVerticalAlignment','bottom','HandleVisibility','off');
end
for n = 0:1
    yl = ylim;
    patch([n*360 n*360 n*360+180 n*360+180],[yl(1) yl(2) yl(2) yl(1)], ...
        [0.85 0.9 0.85],'FaceAlpha',0.25,'EdgeColor','none','HandleVisibility','off');
end

subplot(3,1,3);
plot(theta_deg, a_p, 'r-', 'LineWidth', 1.8);
ylabel('Acceleration (m/s²)');
xlabel('Crank angle (°)  —  0°/360°=BDC, 180°=TDC');
yline(0,'k:','LineWidth',0.8,'HandleVisibility','off');
xticks(0:90:720); xlim([0 720]); grid on;
for deg = [90 180 270 360 450 540 630]
    if mod(deg,360)==0; lbl='BDC'; elseif deg==180||deg==540; lbl='TDC'; else; lbl=''; end
    xline(deg,'k:',lbl,'LabelVerticalAlignment','bottom','HandleVisibility','off');
end
for n = 0:1
    yl = ylim;
    patch([n*360 n*360 n*360+180 n*360+180],[yl(1) yl(2) yl(2) yl(1)], ...
        [0.85 0.9 0.85],'FaceAlpha',0.25,'EdgeColor','none','HandleVisibility','off');
end

%% ── FORCES & PRESSURE FIGURE ─────────────────────────────────────────────────
fig_forces = figure('Position', [50 50 1000 420], 'Color', 'white');

plot(theta_deg, F_total,  'k-',  'LineWidth', 2,   'DisplayName', 'F_{total}');  hold on;
if k_spring > 0
    plot(theta_deg, F_spring, 'b--', 'LineWidth', 1.8, 'DisplayName', 'F_{spring}');
end
plot(theta_deg, F_motor,  'r-',  'LineWidth', 1.8, 'DisplayName', 'F_{motor}');
yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
ylabel('Force (N)');

yl = ylim;
for n = 0:1
    patch([n*360 n*360 n*360+180 n*360+180],[yl(1) yl(2) yl(2) yl(1)], ...
        [0.9 0.85 0.85],'FaceAlpha',0.25,'EdgeColor','none','HandleVisibility','off');
end
for deg = [90 180 270 360 450 540 630]
    if mod(deg,360)==0; lbl='BDC'; elseif deg==180||deg==540; lbl='TDC'; else; lbl=''; end
    xline(deg,'k:',lbl,'LabelVerticalAlignment','bottom','HandleVisibility','off');
end

xlabel('Crank angle (°)  —  0°/360°=BDC, 180°=TDC');
title(sprintf('Piston forces — HR=%d bpm | k_{spring}=%.0f N/m', HR, k_spring));
legend('Location', 'northeast', 'FontSize', 9);
xticks(0:90:720); xlim([0 720]); grid on;

%% ── DISPLACEMENT & PRESSURE OVERLAY FIGURE ──────────────────────────────────
fig_disp_pres = figure('Position', [50 50 1000 380], 'Color', 'white');

yyaxis left
plot(theta_deg, x_p*1e3, 'b-', 'LineWidth', 2, 'DisplayName', 'Displacement');
ylabel('Displacement (mm)');
ylim([0, max(x_p*1e3) * 1.12]);

yyaxis right
plot(theta_deg, P_phys/1e3, 'm-', 'LineWidth', 1.8, 'DisplayName', 'Pressure');
ylabel('Pressure (kPa)');
ylim([0, max(P_phys/1e3) * 1.12]);

for n = 0:1
    yyaxis left; yl = ylim;
    patch([n*360 n*360 n*360+180 n*360+180],[yl(1) yl(2) yl(2) yl(1)], ...
        [0.9 0.85 0.85],'FaceAlpha',0.25,'EdgeColor','none','HandleVisibility','off');
end
for deg = [90 180 270 360 450 540 630]
    if mod(deg,360)==0; lbl='BDC'; elseif deg==180||deg==540; lbl='TDC'; else; lbl=''; end
    xline(deg,'k:',lbl,'LabelVerticalAlignment','bottom','HandleVisibility','off');
end

xlabel('Crank angle (°)  —  0°/360°=BDC, 180°=TDC');
title('Piston displacement & pressure');
legend('Location', 'northeast', 'FontSize', 9);
xticks(0:90:720); xlim([0 720]); grid on;

%% ── MOTOR SPECCING ───────────────────────────────────────────────────────────
fprintf('\n=== MOTOR SPECCING ===\n');

RPM_shaft_sys  = omega_sys * 60 / (2*pi);
RPM_shaft_dia  = omega_dia * 60 / (2*pi);
RPM_shaft_peak = max(RPM_shaft_sys, RPM_shaft_dia);

fprintf('Shaft RPM — systolic: %.1f | diastolic: %.1f | peak: %.1f\n', ...
    RPM_shaft_sys, RPM_shaft_dia, RPM_shaft_peak);
fprintf('Peak shaft torque: %.4f N·m\n', max(T_shaft));
fprintf('Mean electrical power (gear-ratio independent): %.2f W\n\n', mean(P_elec));

N_gb_sweep   = 1:1:max(100, N_gb_check);
T_mot_peak   = zeros(size(N_gb_sweep));
T_mot_mean   = zeros(size(N_gb_sweep));
RPM_mot_peak = zeros(size(N_gb_sweep));

for i = 1:numel(N_gb_sweep)
    N               = N_gb_sweep(i);
    T_mot_inst      = T_shaft / (N * gearbox.eta);
    T_mot_peak(i)   = max(T_mot_inst);
    T_mot_mean(i)   = mean(T_mot_inst);
    RPM_mot_peak(i) = RPM_shaft_peak * N;
end

fprintf('%-8s %-14s %-14s %-16s\n', ...
    'N_gb', 'Peak T_mot (mN·m)', 'Mean T_mot (mN·m)', 'Peak RPM_mot');
fprintf('%s\n', repmat('-', 1, 55));

print_ratios = unique([1 2 3 5 10 15 20 25 30 40 50 60 N_gb_check]);
for i = 1:numel(print_ratios)
    N   = print_ratios(i);
    idx = find(N_gb_sweep == N, 1);
    if isempty(idx); continue; end
    fprintf('%-8d %-14.2f %-14.2f %-16.1f\n', ...
        N, T_mot_peak(idx)*1e3, T_mot_mean(idx)*1e3, RPM_mot_peak(idx));
end

RPM_motor_max = motor.RPM_max;
feasible      = RPM_mot_peak <= RPM_motor_max;

if any(feasible)
    [T_opt, opt_idx] = min(T_mot_peak(feasible));
    feas_N   = N_gb_sweep(feasible);
    feas_RPM = RPM_mot_peak(feasible);
    N_opt    = feas_N(opt_idx);
    RPM_opt  = feas_RPM(opt_idx);

    fprintf('\n--- Recommended gear ratio (min peak torque, RPM < %d) ---\n', RPM_motor_max);
    fprintf('N_gb = %d  |  Peak T_motor = %.2f mN·m  |  Peak RPM_motor = %.0f\n', ...
        N_opt, T_opt*1e3, RPM_opt);
    fprintf('Mean T_motor = %.2f mN·m  |  Mean P_elec = %.2f W\n', ...
        T_mot_mean(N_gb_sweep == N_opt)*1e3, mean(P_elec));
end

if any(feasible)
    fprintf('\n--- Minimum motor spec summary (use N_gb = %d) ---\n', N_opt);
    fprintf('  RMS electrical power    : %.2f W\n',  rms(P_elec));
    fprintf('  Mean electrical power   : %.2f W\n',  mean(P_elec));
    fprintf('  Peak electrical power   : %.2f W\n',  max(P_elec));
    fprintf('  Peak motor torque       : %.2f mN·m\n', T_opt*1e3);
    fprintf('  Peak motor speed        : %.0f RPM\n', RPM_opt);
    fprintf('  No-load speed margin    : add ≥20%% → %.0f RPM no-load\n', RPM_opt*1.2);
else
    fprintf('\nWARNING: No feasible gear ratio found within RPM_max=%d — motor spec summary skipped.\n', RPM_motor_max);
end

%% ── OPERATING POINT AT N_gb_check ───────────────────────────────────────────
% Single consolidated block — all viability check variables derived here once.
% Spring is already embedded in T_shaft via F_motor above.

% Gearbox output shaft = crank shaft
T_gb_out      = T_shaft;
T_gb_out_peak = max(T_gb_out);
T_gb_out_rms  = rms(T_gb_out);
P_gb_out      = T_shaft .* omega_shaft;
P_gb_out_peak = max(P_gb_out);
P_gb_out_rms  = rms(P_gb_out);

% Motor shaft = gearbox input
T_mot_inst_chk = T_shaft / (N_gb_check * gearbox.eta);
T_mot_pk_chk   = max(T_mot_inst_chk);
T_mot_rms_chk  = rms(T_mot_inst_chk);
RPM_chk        = RPM_shaft_peak * N_gb_check;
RPM_sys_chk    = RPM_shaft_sys  * N_gb_check;
RPM_dia_chk    = RPM_shaft_dia  * N_gb_check;
P_mech_peak    = T_mot_pk_chk   * (RPM_chk * 2*pi/60);
I_rms_req      = T_mot_rms_chk  / motor.T_const;

%% ── MOTOR VIABILITY CHECK ────────────────────────────────────────────────────
fprintf('\n%s\n', repmat('=', 1, 60));
fprintf('  MOTOR VIABILITY CHECK — %s\n', motor.name);
fprintf('  Gear ratio: N = %d\n', N_gb_check);
fprintf('%s\n', repmat('=', 1, 60));
fprintf('\n  %-38s %10s   %10s\n', 'Check', 'Value', 'Limit');
fprintf('  %s\n', repmat('-', 1, 62));

chk_stall    = T_mot_pk_chk < motor.T_stall;
fprintf('  %-38s %7.2f mNm   %7.2f mNm   %s\n', ...
    '1. Peak torque < stall torque', ...
    T_mot_pk_chk*1e3, motor.T_stall*1e3, verdict(chk_stall));

chk_nom_pk   = T_mot_pk_chk <= motor.T_nominal;
overload_pct = 100*(T_mot_pk_chk - motor.T_nominal)/motor.T_nominal;
if chk_nom_pk
    fprintf('  %-38s %7.2f mNm   %7.2f mNm   PASS\n', ...
        '2. Peak torque vs nominal', T_mot_pk_chk*1e3, motor.T_nominal*1e3);
else
    fprintf('  %-38s %7.2f mNm   %7.2f mNm   WARN (+%.0f%% over nominal, burst OK)\n', ...
        '2. Peak torque vs nominal', T_mot_pk_chk*1e3, motor.T_nominal*1e3, overload_pct);
end

chk_rms = T_mot_rms_chk <= motor.T_nominal;
fprintf('  %-38s %7.2f mNm   %7.2f mNm   %s\n', ...
    '3. RMS torque < nominal (thermal)', ...
    T_mot_rms_chk*1e3, motor.T_nominal*1e3, verdict(chk_rms));

RPM_available_at_peak = motor.RPM_noload * (1 - T_mot_pk_chk  / motor.T_stall);
RPM_available_at_rms  = motor.RPM_noload * (1 - T_mot_rms_chk / motor.T_stall);
chk_st_peak = RPM_chk <= RPM_available_at_peak;
chk_st_rms  = RPM_chk <= RPM_available_at_rms;
fprintf('  %-38s %7.0f rpm   %7.0f rpm   %s\n', ...
    '4a. RPM feasible at peak torque', ...
    RPM_chk, RPM_available_at_peak, verdict(chk_st_peak));
fprintf('  %-38s %7.0f rpm   %7.0f rpm   %s\n', ...
    '4b. RPM feasible at RMS torque', ...
    RPM_chk, RPM_available_at_rms, verdict(chk_st_rms));

rpm_dev = 100*(RPM_chk - motor.RPM_nominal)/motor.RPM_nominal;
fprintf('  %-38s %7.0f rpm   %7.0f rpm   (%.0f%% from nominal)\n', ...
    '5. Peak RPM vs nominal speed', ...
    RPM_chk, motor.RPM_nominal, rpm_dev);

chk_pmech = P_mech_peak <= motor.P_rated;
fprintf('  %-38s %7.2f W     %7.2f W     %s\n', ...
    '6. Peak mech power < rated power', ...
    P_mech_peak, motor.P_rated, verdict(chk_pmech));

chk_curr_rms = I_rms_req <= motor.I_nominal;
fprintf('  %-38s %7.3f A     %7.3f A     %s\n', ...
    '7. RMS current < nominal current', ...
    I_rms_req, motor.I_nominal, verdict(chk_curr_rms));

all_pass_motor = all([chk_stall, chk_rms, chk_st_peak, chk_st_rms, chk_pmech, chk_curr_rms]);
fprintf('\n  %s\n', repmat('-', 1, 62));
if all_pass_motor
    fprintf('  MOTOR OVERALL: VIABLE\n');
    if ~chk_nom_pk
        fprintf('  NOTE: peak torque exceeds nominal by %.0f%% — short-term burst only\n', overload_pct);
    end
else
    fprintf('  MOTOR OVERALL: NOT VIABLE — one or more hard limits exceeded\n');
end
fprintf('%s\n\n', repmat('=', 1, 60));

%% ── GEARBOX VIABILITY CHECK ──────────────────────────────────────────────────
fprintf('%s\n', repmat('=', 1, 60));
fprintf('  GEARBOX VIABILITY CHECK — %s\n', gearbox.name);
fprintf('  Reduction: %d:1\n', N_gb_check);
fprintf('%s\n', repmat('=', 1, 60));
fprintf('\n  %-38s %10s   %10s\n', 'Check', 'Value', 'Limit');
fprintf('  %s\n', repmat('-', 1, 62));

chk_gb_T_peak = T_gb_out_peak <= gearbox.T_intermit;
fprintf('  %-38s %7.3f Nm    %7.3f Nm    %s\n', ...
    '1. Peak torque < intermittent limit', ...
    T_gb_out_peak, gearbox.T_intermit, verdict(chk_gb_T_peak));

chk_gb_T_rms = T_gb_out_rms <= gearbox.T_cont;
fprintf('  %-38s %7.3f Nm    %7.3f Nm    %s\n', ...
    '2. RMS torque < continuous limit', ...
    T_gb_out_rms, gearbox.T_cont, verdict(chk_gb_T_rms));

chk_gb_P_peak = P_gb_out_peak <= gearbox.P_intermit;
fprintf('  %-38s %7.3f W     %7.3f W     %s\n', ...
    '3. Peak power < intermittent limit', ...
    P_gb_out_peak, gearbox.P_intermit, verdict(chk_gb_P_peak));

chk_gb_P_rms = P_gb_out_rms <= gearbox.P_cont;
fprintf('  %-38s %7.3f W     %7.3f W     %s\n', ...
    '4. RMS power < continuous limit', ...
    P_gb_out_rms, gearbox.P_cont, verdict(chk_gb_P_rms));

all_pass_gb = all([chk_gb_T_peak, chk_gb_T_rms, chk_gb_P_peak, chk_gb_P_rms]);
fprintf('\n  %s\n', repmat('-', 1, 62));
if all_pass_gb
    fprintf('  GEARBOX OVERALL: VIABLE\n');
else
    fprintf('  GEARBOX OVERALL: NOT VIABLE — one or more limits exceeded\n');
end

fprintf('\n  DRIVETRAIN (%s + %s @ N=%d): %s\n', ...
    motor.name, gearbox.name, N_gb_check, ...
    verdict(all_pass_motor && all_pass_gb));
fprintf('%s\n\n', repmat('=', 1, 60));

%% ── MOTOR SPEC FIGURE ────────────────────────────────────────────────────────
fig_motor = figure('Position', [50 400 900 420], 'Color', 'white');

subplot(1,2,1);
yyaxis left
plot(N_gb_sweep, T_mot_peak*1e3, 'b-', 'LineWidth', 2); hold on;
yline(motor.T_nominal*1e3, 'b:', sprintf('T_{nom}=%.1f mNm', motor.T_nominal*1e3), ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');
yline(motor.T_stall*1e3,   'b--', sprintf('T_{stall}=%.1f mNm', motor.T_stall*1e3), ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');
ylabel('Peak motor torque (mN·m)');
yyaxis right
plot(N_gb_sweep, RPM_mot_peak, 'r-', 'LineWidth', 2);
yline(motor.RPM_max,     'r--', sprintf('RPM_{max}=%d', motor.RPM_max), ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');
yline(motor.RPM_nominal, 'r:',  sprintf('RPM_{nom}=%d', motor.RPM_nominal), ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');
ylabel('Peak motor RPM');
if any(feasible)
    xline(N_opt, 'k--', sprintf('N_{opt}=%d', N_opt), ...
        'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
end
xline(N_gb_check, 'm-', sprintf('N_{check}=%d', N_gb_check), ...
    'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
xlabel('Gear ratio N_{gb}');
title('Motor torque & speed vs gear ratio');
legend('Peak T_{mot}', 'Peak RPM_{mot}', 'Location', 'east');
grid on;

subplot(1,2,2);
scatter(RPM_mot_peak, T_mot_peak*1e3, 40, N_gb_sweep, 'filled');
colorbar; clabel_h = colorbar; clabel_h.Label.String = 'Gear ratio N_{gb}';
hold on;
if any(feasible)
    scatter(RPM_opt, T_opt*1e3, 120, 'rp', 'filled', ...
        'DisplayName', sprintf('N_{opt}=%d', N_opt));
end
scatter(RPM_chk, T_mot_pk_chk*1e3, 150, 'm', 's', 'filled', ...
    'DisplayName', sprintf('N_{check}=%d', N_gb_check));
xline(motor.RPM_max,     'r--', sprintf('RPM_{max}=%d', motor.RPM_max), ...
    'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom', 'HandleVisibility','off');
xline(motor.RPM_nominal, 'r:',  sprintf('RPM_{nom}=%d', motor.RPM_nominal), ...
    'LineWidth', 1.2, 'LabelVerticalAlignment', 'bottom', 'HandleVisibility','off');
yline(motor.T_nominal*1e3, 'b:', sprintf('T_{nom}=%.1f mNm', motor.T_nominal*1e3), ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');
yline(motor.T_stall*1e3,   'b--', sprintf('T_{stall}=%.1f mNm', motor.T_stall*1e3), ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');
xlabel('Peak motor RPM');
ylabel('Peak motor torque (mN·m)');
title('Motor operating envelope — each point = one gear ratio');
legend('Location', 'northeast');
grid on;

sgtitle(sprintf('Motor speccing — HR=%d bpm | k_{spring}=%.0f N/m | %s + %s @ N=%d', ...
    HR, k_spring, motor.name, gearbox.name, N_gb_check), 'FontWeight', 'bold');

%% ── BPM SWEEP ────────────────────────────────────────────────────────────────
fprintf('\n=== BPM SWEEP ===\n');
fprintf('%-8s %-14s %-14s %-10s\n', 'HR (bpm)', 'Peak P_elec (W)', 'Mean P_elec (W)', 'Status');
fprintf('%s\n', repmat('-', 1, 48));

HR_sweep   = 120:5:195;
P_peak_bpm = zeros(size(HR_sweep));
P_mean_bpm = zeros(size(HR_sweep));

for hi = 1:numel(HR_sweep)
    hr_s    = HR_sweep(hi);
    T_s     = 60 / hr_s;
    T_sys_s = SF * T_s;
    T_dia_s = (1 - SF) * T_s;

    dt_s  = T_s / N_pts;
    t_s   = [(0:N_pts-1)*dt_s, (0:N_pts-1)*dt_s + T_s];
    tc_s  = mod(t_s, T_s);
    nc_s  = floor(t_s / T_s);
    si_s  = tc_s < T_sys_s;

    oms_s = pi / T_sys_s;
    omd_s = pi / T_dia_s;
    th_s  = nc_s*2*pi + si_s.*(oms_s.*tc_s) + ~si_s.*(pi + omd_s.*(tc_s - T_sys_s));

    ow_s          = zeros(size(t_s));
    ow_s(si_s)    = oms_s;
    ow_s(~si_s)   = omd_s;

    xp_s  = r_c*(1 - cos(th_s)) + L_rod*(1 - sqrt(1 - lambda^2*sin(th_s).^2));
    d2x_s = r_c*cos(th_s) ...
          + r_c*lambda.*(cos(th_s).^2 - sin(th_s).^2) ./ sqrt(1 - lambda^2*sin(th_s).^2) ...
          + r_c*lambda^3.*sin(th_s).^2.*cos(th_s).^2  ./ (1 - lambda^2*sin(th_s).^2).^(3/2);
    ap_s  = d2x_s .* ow_s.^2;

    Pp_s   = zeros(size(t_s));
    tivc_s = 0.10 * T_sys_s;
    Pp_s(si_s & tc_s < tivc_s) = P_dia * tc_s(si_s & tc_s < tivc_s) / tivc_s;
    ej1_s  = si_s & tc_s >= tivc_s & tc_s < 0.40*T_sys_s;
    Pp_s(ej1_s) = P_dia + (P_sys-P_dia) * (tc_s(ej1_s)-tivc_s) / (0.30*T_sys_s);
    ej2_s  = si_s & tc_s >= 0.40*T_sys_s;
    Pp_s(ej2_s) = P_sys + (P_end-P_sys) * (tc_s(ej2_s)-0.40*T_sys_s) / (0.60*T_sys_s);
    tau_s = (tc_s(~si_s) - T_sys_s) / T_dia_s;
    Pp_s(~si_s) = P_end * (exp(-k_dia*tau_s) - exp(-k_dia)) / (1 - exp(-k_dia));

    Ff_s = si_s .* Pp_s * A_plate;
    Fb_s = si_s .* dP_bag * A_plate;
    Fi_s = m_mover * abs(ap_s);
    Ft_s = Ff_s + Fb_s + Fi_s + F_para;

    sf_s       = ones(size(t_s));
    sf_s(si_s) = -1;
    Fsp_s      = k_spring * (stroke - xp_s);
    Fm_s       = Ft_s + sf_s .* Fsp_s;

    Tsh_s = crank_torque(Fm_s, th_s);               % ← velocity pole formula
    Pe_s  = Tsh_s .* ow_s / eta_drive;

    P_peak_bpm(hi) = max(Pe_s);
    P_mean_bpm(hi) = mean(Pe_s);

    status = pass_fail(P_peak_bpm(hi), P_budget);
    fprintf('%-8d %-14.2f %-14.2f %-10s\n', hr_s, P_peak_bpm(hi), P_mean_bpm(hi), status);
end

HR_fine     = linspace(HR_sweep(1), HR_sweep(end), 2000);
P_peak_fine = interp1(HR_sweep, P_peak_bpm, HR_fine, 'pchip');
cross_idx   = find(P_peak_fine >= P_budget, 1, 'first');

if ~isempty(cross_idx) && cross_idx > 1
    HR_max_safe = HR_fine(cross_idx - 1);
elseif cross_idx == 1
    HR_max_safe = NaN;
    fprintf('WARNING: Peak P_elec exceeds %.1f W budget at all heart rates in sweep.\n', P_budget);
else
    HR_max_safe = HR_sweep(end);
    fprintf('All heart rates in sweep are within %.1f W budget.\n', P_budget);
end

fprintf('\nMax safe HR (peak P_elec <= %.1f W): %s bpm\n', P_budget, ...
    ternary(isnan(HR_max_safe), 'NONE — already over budget at 120 bpm', ...
    sprintf('%.1f', HR_max_safe)));

fig_bpm = figure('Position', [50 750 780 400], 'Color', 'white');
hold on;

yl_top = max(P_peak_bpm) * 1.15;
patch([HR_sweep(1) HR_sweep(end) HR_sweep(end) HR_sweep(1)], ...
      [P_budget yl_top yl_top P_budget], ...
      [1 0.82 0.82], 'FaceAlpha', 0.45, 'EdgeColor', 'none', 'HandleVisibility', 'off');

plot(HR_fine, P_peak_fine, 'r-', 'LineWidth', 2.5, 'DisplayName', 'Peak P_{elec}');
plot(HR_fine, interp1(HR_sweep, P_mean_bpm, HR_fine, 'pchip'), ...
     'b--', 'LineWidth', 1.8, 'DisplayName', 'Mean P_{elec}');

yline(P_budget, 'k-', sprintf('Limit: %.1f W', P_budget), ...
      'LineWidth', 2, 'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');

if ~isempty(cross_idx) && cross_idx > 1 && ~isnan(HR_max_safe)
    xline(HR_max_safe, 'k--', sprintf('Max HR = %.0f bpm', HR_max_safe), ...
          'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
    plot(HR_max_safe, P_budget, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', ...
         'DisplayName', sprintf('Crossover (%.0f bpm)', HR_max_safe));
end

scatter(HR_sweep, P_peak_bpm, 50, 'r', 'filled', 'HandleVisibility', 'off');
scatter(HR_sweep, P_mean_bpm, 50, 'b', 'filled', 'HandleVisibility', 'off');

nom_idx = HR_sweep == 120;
plot(120, P_peak_bpm(nom_idx), 'r^', 'MarkerSize', 9, 'MarkerFaceColor', 'r', ...
     'DisplayName', sprintf('Nominal 120 bpm (%.2f W)', P_peak_bpm(nom_idx)));

xlabel('Heart rate (bpm)');
ylabel('Electrical power (W)');
title(sprintf('Peak & mean P_{elec} vs heart rate — k_{spring}=%.0f N/m | limit=%.1f W', ...
    k_spring, P_budget));
legend('Location', 'northwest', 'FontSize', 9);
ylim([0 yl_top]);
xlim([HR_sweep(1)-2 HR_sweep(end)+2]);
xticks(HR_sweep);
xtickangle(45);
grid on;

%% ── FIGURE EXPORT ────────────────────────────────────────────────────────────
if EXPORT_FIGS
    if ~exist(EXPORT_DIR, 'dir')
        [status, msg] = mkdir(EXPORT_DIR);
        if ~status
            error('Could not create export folder: %s\n%s', EXPORT_DIR, msg);
        end
        fprintf('\nCreated export folder: %s\n', EXPORT_DIR);
    end

    export_list = {
        'spring_sweep', fig_spring,     'fig1_spring_sweep';
        'power_cycle',  fig_power,      'fig2_power_cycle';
        'kinematics',   fig_kin,        'fig3_kinematics';
        'forces',       fig_forces,     'fig4_forces';
        'disp_pres',    fig_disp_pres,  'fig5_disp_pres';
        'motor_spec',   fig_motor,      'fig6_motor_spec';
        'bpm_sweep',    fig_bpm,        'fig7_bpm_sweep';
    };

    fprintf('\n=== FIGURE EXPORT ===\n');
    for ei = 1:size(export_list, 1)
        flag_name = export_list{ei, 1};
        fig_h     = export_list{ei, 2};
        fname     = export_list{ei, 3};

        if export_flags.(flag_name) && isvalid(fig_h)
            fpath = fullfile(EXPORT_DIR, sprintf('%s.%s', fname, EXPORT_FMT));
            exportgraphics(fig_h, fpath, 'Resolution', EXPORT_DPI);
            fprintf('  Exported: %s\n', fpath);
        else
            fprintf('  Skipped:  %s\n', fname);
        end
    end
    fprintf('Export complete.\n');
end

%% ── HELPER FUNCTIONS ─────────────────────────────────────────────────────────
function s = pass_fail(val, lim)
    if val <= lim; s = 'PASS'; else; s = 'FAIL'; end
end

function s = verdict(tf)
    if tf; s = 'PASS'; else; s = 'FAIL'; end
end

function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end