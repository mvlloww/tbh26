%% Crank-and-Slotted-Arm LVAD — Biventricular Dynamics (Double-Radius Slot)
%
% Mirror of teardrop_plot.m for the double-radius slot variant.
% r1 arc at near-pivot end (reduces QR ratio); r2 arcs on the slot sides
% (smooth dtheta/dphi at the r1/wall junction, reducing power peaks).
%
% KEY PARAMETERS TO TUNE:
%   r1  — teardrop arc radius (mm); 0 → straight slot, must be < x
%   r2  — side-wall arc radius (mm); 0 → falls back to single teardrop
%   b   — geometric bag overlap [0, 0.5]
%   L, w, L_contact — paddle geometry (mm)

clear; clc; close all;

%% Mechanism parameters
x   = 5.4;    % Crank arm length, mm
a   = 32.2;   % Crank centre to fulcrum distance, mm
r1  = 1.6;    % Teardrop arc radius, mm  (= 0.2x)
r2  = 3.0;    % Side-wall arc radius, mm

rpm_max  = 145;
T        = 60 / rpm_max;
omega_gb = 2*pi / T;
b        = 0.5;

%% Paddle geometry
L         = 57.3;   % mm
w         = 75;     % mm
L_contact = 50;     % mm
K_geom    = w * (L^2 - (L - L_contact)^2) / 2;

%% Torque / power chain
p_bag   = 16e3;   % Pa
F_e     = 10;     % N
GR      = 62;
e_gb    = 0.90;
e_mech  = 0.72;
e_motor = 0.83;

%% Kinematics
phi_fine      = linspace(0, 360, 1441);
theta_fine    = teardrop_double_theta(phi_fine, x, a, r1, r2);
dth_dphi_fine = gradient(theta_fine, phi_fine);

[alpha, ipk] = max(theta_fine);
phi_pk       = phi_fine(ipk);
gamma        = alpha * b / (1-b);
qr_ratio     = max(dth_dphi_fine) / max(-dth_dphi_fine);

% Ejection windows
seg_up       = phi_fine <= phi_pk;
phi_a        = interp1(theta_fine(seg_up), phi_fine(seg_up), gamma);
phi_LV_start = 360 - phi_a;

seg_dn       = (phi_fine >= phi_pk) & (phi_fine <= 180);
phi_RV_start = interp1(fliplr(theta_fine(seg_dn)), fliplr(phi_fine(seg_dn)), gamma);

SV = K_geom * alpha * pi/180 / (1000 * (1-b));
CO = 2 * SV * rpm_max / 1000;

%% Time domain — 2 cycles
N         = 2000;
t         = linspace(0, 2*T, N);
phi_deg_t = mod(omega_gb * t * 180/pi, 360);

theta_t    = interp1(phi_fine, theta_fine,    phi_deg_t);
dth_dphi_t = interp1(phi_fine, dth_dphi_fine, phi_deg_t);

dtheta_dt_rad = dth_dphi_t * omega_gb;
dtheta_dt     = dtheta_dt_rad * (180/pi);

lv_mask = (phi_deg_t >= phi_LV_start) | (phi_deg_t <= phi_pk);
rv_mask = (phi_deg_t >= phi_RV_start) & (phi_deg_t <= 360 - phi_pk);

Q_LV    = max(0,  dtheta_dt_rad) .* double(lv_mask) * K_geom / 1000;
Q_RV    = max(0, -dtheta_dt_rad) .* double(rv_mask) * K_geom / 1000;
Q_total = Q_LV + Q_RV;

A_contact = w * L_contact * 1e-6;
F_total   = p_bag * A_contact + F_e;
r_moment  = (L - L_contact/2) * 1e-3;
Tp_mag    = F_total * r_moment;

Tp_LV    = Tp_mag * double(lv_mask);
Tp_RV    = Tp_mag * double(rv_mask);
Tp_total = Tp_LV - Tp_RV;

dth_dless = dtheta_dt_rad / omega_gb;
Tg_LV    =  Tp_mag .* dth_dless .* double(lv_mask);
Tg_RV    = -Tp_mag .* dth_dless .* double(rv_mask);
Tg       = Tg_LV + Tg_RV;

Tm          = Tg / (GR * e_gb * e_mech);
omega_motor = omega_gb * GR;
P_mech      = Tm .* omega_motor;
P_elec      = P_mech / e_motor;

%% Ejection shading
d_LV = phi_LV_start / 360;
d_RV = phi_RV_start / 360;
lv_segs = [0,          T * phi_pk/360;
           T * d_LV,   T + T * phi_pk/360;
           T + T*d_LV, 2*T              ] * 1000;
rv_segs = [T * d_RV,   T * (360 - phi_pk)/360;
           T + T*d_RV, T + T*(360 - phi_pk)/360] * 1000;
lv_col = [0.72 0.87 1.00];
rv_col = [1.00 0.78 0.78];

%% ================================================================
%  Figure 1: Kinematics
figure('Name','Biventricular Dynamics — Double-Radius Slot','Color','w','Position',[60 60 980 700]);

ax1 = subplot(3,1,1); hold on;
for i=1:size(lv_segs,1); xregion(lv_segs(i,1),lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75); end
for i=1:size(rv_segs,1); xregion(rv_segs(i,1),rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75); end
plot(t*1000, theta_t,'k-','LineWidth',2);
yline( alpha,'b--','LineWidth',1.2,'Label',sprintf('+\\alpha = %.1f°', alpha),'LabelHorizontalAlignment','left');
yline(-alpha,'r--','LineWidth',1.2,'Label',sprintf('-\\alpha = %.1f°',-alpha),'LabelHorizontalAlignment','left');
yline(0,'k:'); xline(T*1000,'k--','Label','Cycle 2','LabelVerticalAlignment','bottom');
hold off;
ylabel('Paddle Angle (°)'); title('Paddle Angle vs Time  [double-radius slot]');
legend({'LV ejection','RV ejection','Paddle angle'},'Location','northeast'); grid on; xlim([0 2*T*1000]);

ax2 = subplot(3,1,2); hold on;
for i=1:size(lv_segs,1); xregion(lv_segs(i,1),lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75); end
for i=1:size(rv_segs,1); xregion(rv_segs(i,1),rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75); end
plot(t*1000, dtheta_dt,'k-','LineWidth',2);
yline(0,'k:'); xline(T*1000,'k--');
hold off; ylabel('d\theta/dt (°/s)'); title('Paddle Angular Velocity vs Time');
grid on; xlim([0 2*T*1000]);

ax3 = subplot(3,1,3); hold on;
for i=1:size(lv_segs,1); xregion(lv_segs(i,1),lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75); end
for i=1:size(rv_segs,1); xregion(rv_segs(i,1),rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75); end
plot(t*1000, Q_LV,   'b-', 'LineWidth',2);
plot(t*1000, Q_RV,   'r-', 'LineWidth',2);
plot(t*1000, Q_total,'k--','LineWidth',1.2);
hold off; xlabel('Time (ms)'); ylabel('Flow Rate (mL/s)'); title('Instantaneous Flow Rate vs Time');
legend({'LV flow','RV flow','Total'},'Location','northeast'); grid on; xlim([0 2*T*1000]);

annotation('textbox',[0.72 0.35 0.26 0.28],'String',{
    sprintf('x = %g mm  |  a = %g mm', x, a),
    sprintf('r1 = %g mm (%.0f%% of x)  |  r2 = %g mm', r1, 100*r1/x, r2),
    sprintf('\\phi_{peak} = %.1f°  (straight: %.1f°)', phi_pk, acosd(x/a)),
    sprintf('\\alpha = %.1f°  (straight: %.1f°)', alpha, asind(x/a)),
    sprintf('QR ratio = %.3f  (straight: %.3f)', qr_ratio, (a+x)/(a-x)),
    sprintf('Overlap b = %.2f  |  \\gamma = %.1f°', b, gamma),
    sprintf('─────────────────────'),
    sprintf('L = %g mm  |  w = %g mm', L, w),
    sprintf('L_{contact} = %g mm from tip', L_contact),
    sprintf('SV = %.1f mL / ventricle', SV),
    sprintf('CO = %.2f L/min combined', CO)}, ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor',[.5 .5 .5],'FontSize',8.5);

sgtitle('Crank-and-Slotted-Arm LVAD — Kinematics (Double-Radius Slot)', ...
    'FontSize',12,'FontWeight','bold');

%% ================================================================
%  Figure 2: Forces & Torques
figure('Name','LVAD Forces & Torques — Double-Radius','Color','w','Position',[100 40 980 1050]);

ax4 = subplot(5,1,1); hold on;
for i=1:size(lv_segs,1); xregion(lv_segs(i,1),lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75); end
for i=1:size(rv_segs,1); xregion(rv_segs(i,1),rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75); end
plot(t*1000, F_total*double(lv_mask),'b-','LineWidth',2);
plot(t*1000, F_total*double(rv_mask),'r-','LineWidth',2);
yline(F_total,'k:','Label',sprintf('F_{total} = %.1f N',F_total),'LabelHorizontalAlignment','left');
hold off; ylabel('F_{total} (N)'); title('F_{total} Applied to Paddle vs Time');
legend({'LV','RV'},'Location','northeast'); grid on; xlim([0 2*T*1000]);

ax5 = subplot(5,1,2); hold on;
for i=1:size(lv_segs,1); xregion(lv_segs(i,1),lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75); end
for i=1:size(rv_segs,1); xregion(rv_segs(i,1),rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75); end
plot(t*1000,  Tp_LV,   'b-', 'LineWidth',2);
plot(t*1000, -Tp_RV,   'r-', 'LineWidth',2);
plot(t*1000,  Tp_total,'k--','LineWidth',1.2);
hold off; ylabel('T_p (N·m)'); title('Paddle Torque vs Time');
legend({'LV load','RV load','Net'},'Location','northeast'); grid on; xlim([0 2*T*1000]);

ax6 = subplot(5,1,3); hold on;
for i=1:size(lv_segs,1); xregion(lv_segs(i,1),lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75); end
for i=1:size(rv_segs,1); xregion(rv_segs(i,1),rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75); end
plot(t*1000, Tg_LV,'b-','LineWidth',2); plot(t*1000, Tg_RV,'r-','LineWidth',2);
plot(t*1000, Tg,   'k--','LineWidth',1.2); yline(0,'k:');
hold off; ylabel('T_g (N·m)'); title('Gearbox Output Torque vs Time');
legend({'LV','RV','Total'},'Location','northeast'); grid on; xlim([0 2*T*1000]);

ax7 = subplot(5,1,4); hold on;
for i=1:size(lv_segs,1); xregion(lv_segs(i,1),lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75); end
for i=1:size(rv_segs,1); xregion(rv_segs(i,1),rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75); end
plot(t*1000, Tm,'k-','LineWidth',2); yline(0,'k:');
hold off; ylabel('T_m (N·m)'); title('Motor Torque vs Time');
grid on; xlim([0 2*T*1000]); ylim([0, max(Tm)*1.25]);

ax8 = subplot(5,1,5); hold on;
for i=1:size(lv_segs,1); xregion(lv_segs(i,1),lv_segs(i,2),'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75); end
for i=1:size(rv_segs,1); xregion(rv_segs(i,1),rv_segs(i,2),'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75); end
plot(t*1000, P_elec,'k-','LineWidth',2); yline(0,'k:');
hold off; xlabel('Time (ms)'); ylabel('P_{elec} (W)'); title('Electrical Input Power vs Time');
grid on; xlim([0 2*T*1000]); ylim([0, max(P_elec)*1.25]);

annotation('textbox',[0.72 0.01 0.26 0.32],'String',{
    sprintf('p_{bag} = %g kPa  |  F_e = %g N', p_bag/1e3, F_e),
    sprintf('A_{contact} = %.0f mm²', w*L_contact),
    sprintf('F_{total} = %.1f N', F_total),
    sprintf('r_{moment} = %.0f mm', r_moment*1000),
    sprintf('T_{p} = %.3f N·m', Tp_mag),
    sprintf('─────────────────────'),
    sprintf('T_{g,LV} = %.4f N·m  (peak)', max(Tg_LV)),
    sprintf('T_{g,RV} = %.4f N·m  (peak)', max(Tg_RV)),
    sprintf('─────────────────────'),
    sprintf('GR = %g  |  \\eta_{gb}=%.2f  \\eta_{mech}=%.2f', GR, e_gb, e_mech),
    sprintf('\\eta_{motor}=%.2f  |  T_{m,peak}=%.5f N·m', e_motor, max(Tm)),
    sprintf('P_{mech,peak} = %.3f W', max(P_mech)),
    sprintf('P_{elec,peak} = %.3f W', max(P_elec))}, ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor',[.5 .5 .5],'FontSize',8.5);

sgtitle('Crank-and-Slotted-Arm LVAD — Forces & Torques (Double-Radius Slot)', ...
    'FontSize',12,'FontWeight','bold');

%% ================================================================
%  Figure 3: Crank vs Paddle angle
figure('Name','Crank vs Paddle — Double-Radius','Color','w','Position',[120 120 820 520]);
hold on;
xregion(0,            phi_pk,      'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_LV_start, 360,         'FaceColor',lv_col,'EdgeColor','none','FaceAlpha',0.75);
xregion(phi_RV_start, 360-phi_pk,  'FaceColor',rv_col,'EdgeColor','none','FaceAlpha',0.75);
plot(phi_fine, theta_fine,'k-','LineWidth',2);
yline( alpha,'b--','LineWidth',1.2,'Label',sprintf('+\\alpha = %.1f°',alpha),'LabelHorizontalAlignment','left');
yline(-alpha,'r--','LineWidth',1.2,'Label',sprintf('-\\alpha = %.1f°',-alpha),'LabelHorizontalAlignment','left');
xline(phi_pk,      'b:','LineWidth',1,'Label',sprintf('%.1f°',phi_pk),     'LabelVerticalAlignment','bottom');
xline(360-phi_pk,  'r:','LineWidth',1,'Label',sprintf('%.1f°',360-phi_pk), 'LabelVerticalAlignment','bottom');
xline(phi_RV_start,'r--','LineWidth',1,'Label',sprintf('RV start %.1f°',phi_RV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');
xline(phi_LV_start,'b--','LineWidth',1,'Label',sprintf('LV start %.1f°',phi_LV_start),'LabelVerticalAlignment','top','LabelHorizontalAlignment','right');
plot(phi_pk,      alpha, 'bs','MarkerFaceColor','b','MarkerSize',8);
plot(360-phi_pk, -alpha, 'rs','MarkerFaceColor','r','MarkerSize',8);
hold off;
xlabel('Crank Angle (°)'); ylabel('Paddle Angle (°)');
title('Crank Angle vs Paddle Angle — Double-Radius Slot','FontSize',12);
legend({'LV ejection','RV ejection','Paddle angle'},'Location','northeast');
grid on; xlim([0 360]); ylim([-alpha*1.3, alpha*1.3]); xticks(0:45:360);

%% Console summary
fprintf('=== Crank-and-Slotted-Arm LVAD — Double-Radius Slot ===\n');
fprintf('  r1 = %.2f mm  r2 = %.2f mm\n', r1, r2);
fprintf('  alpha = %.2f deg  (straight slot: %.2f)\n', alpha, asind(x/a));
fprintf('  QR ratio = %.3f  (straight: %.3f)\n', qr_ratio, (a+x)/(a-x));
fprintf('  CO = %.2f L/min combined\n', CO);
fprintf('  P_elec_peak = %.3f W\n', max(P_elec));

%% ================================================================
function theta = teardrop_double_theta(phi_deg, x, a, r1, r2)
    theta_o = atand(x*sind(phi_deg) ./ (a - x*cosd(phi_deg)));
    if r1 < 1e-9; theta = theta_o; return; end
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
