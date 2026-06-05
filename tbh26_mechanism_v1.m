%% TBH27 Actuation Subsystem — Four-Bar Linkage Kinematic Model
%  Models the crank-coupler-paddle (rocker) mechanism over two cardiac cycles.
%  Solves the Freudenstein equation for paddle angle, then differentiates
%  for angular velocity, acceleration, and torque transmission.
%
%  Link definitions:
%    r1 = ground link  (motor axis to paddle fulcrum, fixed)
%    r2 = crank        (driven by motor via gearbox)
%    r3 = coupler      (connects crank pin to paddle pin)
%    r4 = paddle/rocker (output link, rotates about fulcrum)
%
%  Coordinate convention:
%    Ground link along +x axis. Crank rotates CCW from x-axis.
%    All angles in radians unless stated.

clear; clc; close all;

%% ── PARAMETERS ──────────────────────────────────────────────────────────────

% Link lengths (mm) — edit these to match CAD measurements
r1 = 25;    % ground link:  motor axis → paddle fulcrum
r2 = 10;    % crank arm:    x = 10 mm (Appendix D)
r3 = 18;    % coupler:      estimate — measure from CAD
r4 = 40;    % paddle:       L = 40 mm (Appendix D)

% Operating parameters
bpm       = 73.1;            % cardiac rate (bpm) — use min speed (worst-case torque)
GR        = 62.1;            % nominal gearbox reduction (Appendix C)
eta_mech  = 0.70;            % mechanical efficiency (scotch yoke ref — adjust)
eta_gb    = 0.74;            % gearbox efficiency (Appendic C)
Tp        = 0.93;            % resistive torque on paddle (Nm) — from report

% Initial crank angle (deg) — set so paddle starts at mid-stroke
theta2_0_deg = 0;

% Resolution
n_cycles  = 2;
n_pts     = 2000;            % points per cycle

%% ── DERIVED QUANTITIES ───────────────────────────────────────────────────────

omega_bpm  = bpm / 60;                     % rev/s
omega2     = omega_bpm * 2 * pi;           % crank angular velocity (rad/s)
T_cycle    = 1 / omega_bpm;               % period of one cycle (s)
t          = linspace(0, n_cycles * T_cycle, n_cycles * n_pts);
theta2     = omega2 * t + deg2rad(theta2_0_deg);   % crank angle over time (rad)

%% ── GRASHOF CHECK ────────────────────────────────────────────────────────────
links = sort([r1 r2 r3 r4]);
S = links(1); L = links(4); P = links(2); Q = links(3);
if S + L <= P + Q
    disp('Grashof condition satisfied — crank can fully rotate.');
else
    warning('Grashof condition NOT satisfied — crank cannot fully rotate. Check link lengths.');
end

%% ── FREUDENSTEIN SOLUTION FOR PADDLE ANGLE θ4 ───────────────────────────────
% Freudenstein equation:
%   K1*cos(θ2) - K2*cos(θ4) + K3 = cos(θ2 - θ4)
% where:
%   K1 = r1/r4,  K2 = r1/r2,  K3 = (r1^2 + r2^2 - r3^2 + r4^2) / (2*r2*r4)
%
% Rearranged as:  A*cos(θ4) + B*sin(θ4) = C
%   A =  K2*cos(θ2) - K1 - K3 + cos(θ2)   [from half-angle substitution]
% Using standard substitution t = tan(θ4/2):
%   (1-t^2)*A + 2t*B = C*(1+t^2)  → quadratic in t

K1 = r1 / r4;
K2 = r1 / r2;
K3 = (r1^2 + r2^2 - r3^2 + r4^2) / (2 * r2 * r4);

% For each crank angle, solve for θ4 (two solutions — pick the physical one)
theta4 = zeros(size(theta2));
theta3 = zeros(size(theta2));

for i = 1:length(theta2)
    th2 = theta2(i);

    % Freudenstein coefficients
    A =  cos(th2) - K1 - K2*cos(th2) + K3;
    B = -2 * sin(th2);
    C =  K1 - (1 + K2)*cos(th2) + K3;

    % Quadratic in tan(θ4/2): (A+C)*t^2 + 2B*t + (C-A) = 0
    % Wait — standard form: A*cos(θ4) + B*sin(θ4) = C
    % Rewrite with Freudenstein properly:
    % K1*cos(θ2) - K2*cos(θ4) + K3 = cos(θ2 - θ4)
    %                                = cos(θ2)cos(θ4) + sin(θ2)sin(θ4)
    % → (K2 + cos(θ2))*cos(θ4) - sin(θ2)*sin(θ4) = K1*cos(θ2) + K3
    % → A_f*cos(θ4) + B_f*sin(θ4) = C_f
    A_f =  K2 + cos(th2);
    B_f = -sin(th2);
    C_f =  K1*cos(th2) + K3;

    % Half-angle substitution: t = tan(θ4/2)
    % cos(θ4) = (1-t²)/(1+t²), sin(θ4) = 2t/(1+t²)
    % → (A_f - C_f)*t² - 2*B_f*t + (-A_f - C_f) = 0  [multiply through by (1+t²)]
    % Wait: A_f*(1-t²) + B_f*2t = C_f*(1+t²)
    % → -(A_f+C_f)*t² + 2*B_f*t + (A_f-C_f) = 0
    a_q = -(A_f + C_f);
    b_q =  2 * B_f;
    c_q =  A_f - C_f;

    disc = b_q^2 - 4*a_q*c_q;
    if disc < 0
        % No real solution at this crank angle — linkage has reached a limit
        theta4(i) = theta4(max(i-1,1));
        continue
    end

    if abs(a_q) < 1e-10
        % Degenerate case
        if abs(b_q) > 1e-10
            t_sol = [-c_q / b_q];
        else
            t_sol = [0];
        end
    else
        t1 = (-b_q + sqrt(disc)) / (2*a_q);
        t2 = (-b_q - sqrt(disc)) / (2*a_q);
        t_sol = [t1, t2];
    end

    th4_candidates = 2 * atan(t_sol);

    % Pick the solution that keeps the paddle in the expected range
    % (positive y-side: open configuration)
    if i == 1
        % First point: pick solution closest to 90 deg (paddle roughly upright)
        [~, idx] = min(abs(th4_candidates - pi/2));
        theta4(i) = th4_candidates(idx);
    else
        % Subsequent points: pick solution closest to previous value (continuity)
        [~, idx] = min(abs(th4_candidates - theta4(i-1)));
        theta4(i) = th4_candidates(idx);
    end

    % Solve for coupler angle θ3 from vector loop
    % r2*sin(θ2) + r3*sin(θ3) = r4*sin(θ4)
    % → sin(θ3) = (r4*sin(θ4) - r2*sin(th2)) / r3
    sin_th3 = (r4*sin(theta4(i)) - r2*sin(th2)) / r3;
    cos_th3 = (r1 + r4*cos(theta4(i)) - r2*cos(th2)) / r3;
    theta3(i) = atan2(sin_th3, cos_th3);
end

%% ── VELOCITY ANALYSIS ────────────────────────────────────────────────────────
% From differentiated vector loop equations:
%   -r2*ω2*sin(θ2) - r3*ω3*sin(θ3) + r4*ω4*sin(θ4) = 0
%    r2*ω2*cos(θ2) + r3*ω3*cos(θ3) - r4*ω4*cos(θ4) = 0
%
% In matrix form: [A]{ω3,ω4}' = {b}

omega3 = zeros(size(theta2));
omega4 = zeros(size(theta2));

for i = 1:length(theta2)
    A_v = [-r3*sin(theta3(i)),  r4*sin(theta4(i));
            r3*cos(theta3(i)), -r4*cos(theta4(i))];
    b_v = [ r2*omega2*sin(theta2(i));
           -r2*omega2*cos(theta2(i))];

    if abs(det(A_v)) < 1e-10
        omega3(i) = 0; omega4(i) = 0;
    else
        sol = A_v \ b_v;
        omega3(i) = sol(1);
        omega4(i) = sol(2);
    end
end

%% ── ACCELERATION ANALYSIS ────────────────────────────────────────────────────
% Differentiate velocity equations (α2 = 0, constant crank speed):
%   -r3*α3*sin(θ3) - r3*ω3²*cos(θ3) + r4*α4*sin(θ4) + r4*ω4²*cos(θ4) = r2*ω2²*cos(θ2)
%    r3*α3*cos(θ3) - r3*ω3²*sin(θ3) - r4*α4*cos(θ4) + r4*ω4²*sin(θ4) = r2*ω2²*sin(θ2)

alpha3 = zeros(size(theta2));
alpha4 = zeros(size(theta2));

for i = 1:length(theta2)
    A_a = [-r3*sin(theta3(i)),  r4*sin(theta4(i));
            r3*cos(theta3(i)), -r4*cos(theta4(i))];
    b_a = [ r2*omega2^2*cos(theta2(i)) + r3*omega3(i)^2*cos(theta3(i)) - r4*omega4(i)^2*cos(theta4(i));
            r2*omega2^2*sin(theta2(i)) + r3*omega3(i)^2*sin(theta3(i)) - r4*omega4(i)^2*sin(theta4(i))];

    if abs(det(A_a)) < 1e-10
        alpha3(i) = 0; alpha4(i) = 0;
    else
        sol = A_a \ b_a;
        alpha3(i) = sol(1);
        alpha4(i) = sol(2);
    end
end

%% ── TRANSMISSION ANGLE ───────────────────────────────────────────────────────
% Angle between coupler (r3) and rocker (r4) — should stay 40°–140° for good force transmission
mu = abs(theta4 - theta3);           % transmission angle (rad)
mu = mod(mu, pi);                    % fold to [0, π]
mu_deg = rad2deg(mu);

%% ── TORQUE & MECHANICAL ADVANTAGE ───────────────────────────────────────────
% Instantaneous velocity ratio (mechanical advantage of four-bar):
%   MA = ω4 / ω2  — paddle angular velocity / crank angular velocity
% Torque at crank input required to produce Tp at paddle output:
%   T_crank = Tp * (ω4 / ω2)         [ignoring efficiency here]
%   T_motor  = T_crank / (GR * eta_mech * eta_gb)

MA        = omega4 / omega2;         % instantaneous mechanical advantage
T_crank   = Tp .* MA;                % torque required at crank (Nm)
T_motor   = T_crank / (GR * eta_mech * eta_gb);  % motor torque required (Nm)

% Paddle angle in degrees for plotting
theta4_deg = rad2deg(theta4);
theta4_deg = theta4_deg - mean(theta4_deg);   % centre about zero for clarity

% Convert time to cardiac cycle number
cycle_num = t / T_cycle;

%% ── PLOTS ────────────────────────────────────────────────────────────────────

figure('Name','TBH27 Four-Bar Kinematics','NumberTitle','off','Position',[100 100 1100 800]);
tl = tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact');
title(tl, sprintf('TBH27 Four-Bar Kinematics  |  %g bpm  |  r1=%.0f  r2=%.0f  r3=%.0f  r4=%.0f mm', ...
    bpm, r1, r2, r3, r4), 'FontSize', 11);

% ── 1. Paddle angle ──
nexttile;
plot(cycle_num, theta4_deg, 'b', 'LineWidth', 1.5);
xlabel('Cardiac cycle'); ylabel('Paddle angle (°)');
title('Paddle (rocker) angle'); grid on;
yline(0,'--k','LineWidth',0.8);
xline(1,'--','Color',[0.5 0.5 0.5],'LineWidth',0.8);

% ── 2. Crank angle (reference) ──
nexttile;
plot(cycle_num, rad2deg(mod(theta2, 2*pi)), 'Color',[0.5 0.5 0.5], 'LineWidth', 1);
xlabel('Cardiac cycle'); ylabel('Crank angle (°)');
title('Crank angle (input)'); grid on;
xline(1,'--','Color',[0.5 0.5 0.5],'LineWidth',0.8);

% ── 3. Paddle angular velocity ──
nexttile;
plot(cycle_num, omega4, 'r', 'LineWidth', 1.5);
xlabel('Cardiac cycle'); ylabel('\omega_4 (rad/s)');
title('Paddle angular velocity'); grid on;
yline(0,'--k','LineWidth',0.8);
xline(1,'--','Color',[0.5 0.5 0.5],'LineWidth',0.8);

% ── 4. Paddle angular acceleration ──
nexttile;
plot(cycle_num, alpha4, 'Color',[0.85 0.33 0.10], 'LineWidth', 1.5);
xlabel('Cardiac cycle'); ylabel('\alpha_4 (rad/s²)');
title('Paddle angular acceleration'); grid on;
yline(0,'--k','LineWidth',0.8);
xline(1,'--','Color',[0.5 0.5 0.5],'LineWidth',0.8);

% ── 5. Transmission angle ──
nexttile;
plot(cycle_num, mu_deg, 'Color',[0.13 0.55 0.13], 'LineWidth', 1.5);
xlabel('Cardiac cycle'); ylabel('\mu (°)');
title('Transmission angle'); grid on;
yline(40,'--r','LineWidth',1); yline(140,'--r','LineWidth',1);
text(0.02*max(cycle_num), 43, 'Min acceptable (40°)', 'Color','r','FontSize',8);
xline(1,'--','Color',[0.5 0.5 0.5],'LineWidth',0.8);
ylim([0 180]);

% ── 6. Required motor torque ──
nexttile;
plot(cycle_num, abs(T_motor)*1000, 'Color',[0.49 0.18 0.56], 'LineWidth', 1.5);
xlabel('Cardiac cycle'); ylabel('Motor torque (mNm)');
title('Required motor torque'); grid on;
xline(1,'--','Color',[0.5 0.5 0.5],'LineWidth',0.8);

%% ── SUMMARY STATS ────────────────────────────────────────────────────────────

fprintf('\n── TBH27 Four-Bar Linkage Summary ─────────────────────────────\n');
fprintf('  Link lengths: r1=%.1f  r2=%.1f  r3=%.1f  r4=%.1f mm\n', r1,r2,r3,r4);
fprintf('  Operating speed: %.1f bpm  (ω2 = %.2f rad/s)\n', bpm, omega2);
fprintf('\n  Paddle angle range:       %.2f° to %.2f°  (total swing: %.2f°)\n', ...
    min(theta4_deg), max(theta4_deg), max(theta4_deg)-min(theta4_deg));
fprintf('  Max paddle ω4:            %.3f rad/s\n', max(abs(omega4)));
fprintf('  Max paddle α4:            %.2f rad/s²\n', max(abs(alpha4)));
fprintf('\n  Transmission angle range: %.1f° to %.1f°\n', min(mu_deg), max(mu_deg));
if min(mu_deg) < 40 || max(mu_deg) > 140
    fprintf('  ⚠ Transmission angle outside 40°–140° — poor force transmission at some crank positions\n');
else
    fprintf('  ✓ Transmission angle within acceptable range (40°–140°)\n');
end
fprintf('\n  Peak required motor torque: %.2f mNm\n', max(abs(T_motor))*1000);
fprintf('  (vs Maxon ECX FLAT 22S nominal: 28.6 mNm from Appendix C)\n');
fprintf('───────────────────────────────────────────────────────────────\n');

%% ── LINKAGE ANIMATION (optional — comment out if slow) ──────────────────────

figure('Name','Linkage Animation','NumberTitle','off','Position',[200 200 600 500]);
ax = gca; axis equal; grid on; hold on;
title('Four-bar linkage — one cycle'); xlabel('x (mm)'); ylabel('y (mm)');

% Plot one full cycle of coupler curve
n_anim = n_pts;   % one cycle
Ox = 0; Oy = 0;                                      % crank origin (O2)
Dx = r1; Dy = 0;                                     % fulcrum (O4)

coupler_x = zeros(1, n_anim);
coupler_y = zeros(1, n_anim);

for i = 1:n_anim
    Ax = Ox + r2*cos(theta2(i));
    Ay = Oy + r2*sin(theta2(i));
    Bx = Dx + r4*cos(theta4(i));
    By = Dy + r4*sin(theta4(i));
    coupler_x(i) = Ax;
    coupler_y(i) = Ay;
end

plot(coupler_x, coupler_y, '--', 'Color',[0.7 0.7 0.7], 'LineWidth',0.8);  % coupler point trace

% Animate last few frames as static snapshot at 8 evenly spaced crank angles
angles_snap = round(linspace(1, n_anim, 8));
colors_snap = winter(8);

for k = 1:8
    i = angles_snap(k);
    Ax = Ox + r2*cos(theta2(i));
    Ay = Oy + r2*sin(theta2(i));
    Bx = Dx + r4*cos(theta4(i));
    By = Dy + r4*sin(theta4(i));

    col = colors_snap(k,:);
    plot([Ox Ax], [Oy Ay], '-', 'Color',col, 'LineWidth',2);   % crank
    plot([Ax Bx], [Ay By], '-', 'Color',col, 'LineWidth',1.5); % coupler
    plot([Dx Bx], [Dy By], '-', 'Color',col, 'LineWidth',2);   % rocker
end

% Fixed pivots
plot(Ox, Oy, 'ks', 'MarkerFaceColor','k', 'MarkerSize',8);
plot(Dx, Dy, 'ks', 'MarkerFaceColor','k', 'MarkerSize',8);
text(Ox-1, Oy-3, 'O_2 (motor)', 'FontSize',8);
text(Dx-1, Dy-3, 'O_4 (fulcrum)', 'FontSize',8);
legend('Coupler point trace','Location','best','FontSize',8);