function paddle_mechanism_teardrop_viz
% PADDLE_MECHANISM_TEARDROP_VIZ  Interactive animation of the teardrop-slot
% LVAD paddle mechanism (geometry mirrors paddle_angle_teardrop.m).
%
% The teardrop replaces the sharp pivot-end of the radial slot with an arc
% of radius r2, reducing the quick-return ratio. Drag sliders to change
% parameters; press Play to animate through a full crank revolution.
%
% r1 = crank pin radius (fixed, physical property of the pin).
% r2 = teardrop radius (the actual slot's rounded near-pivot wall).
% The pin's CENTRE does not ride on the wall itself — it stays a constant
% distance r1 inside it. So the kinematics (theta(phi)) are driven by the
% effective arc radius r_eff = r2 - r1 (the pin centre's own path), while
% the drawn slot wall is that same shape scaled outward by r2/r_eff about
% the arc centre. This is why the wall's sharp tip sits beyond a+x — a
% finite pin can never get its centre all the way into a mathematical point.
%
% Frame: fulcrum F at origin, crank centre C at (a,0).
% Crank pin P = (a - x*cos(phi), x*sin(phi))  [unchanged by slot shape].
% Arm angle theta is teardrop-modified; it differs from atan2(Py,Px) when
% the pin rides the arc (near phi = 0 / 360).

%% Default parameters  (match paddle_angle_teardrop.m)
r1    = 2;      % Crank pin radius, mm — fixed physical property of the pin
x0    = 5.4;
a0    = 32;
r2_0  = 2*r1;   % teardrop (wall) radius; gives r_eff = r1 at startup
L0    = 57;
Lc0   = 50;
b0    = 0.5;
t0    = 4;     % paddle thickness, mm
phi   = 0;

%% Physics for power overlay (w has no slider — fixed at analytical optimum)
w_fixed   = 100;    % mm
rpm_max   = 145;
p_LV_mmHg = 120;    % LV peak bag pressure, mmHg
p_RV_mmHg = 25;     % RV peak bag pressure, mmHg
mmHg2Pa   = 133.322;
p_LV      = p_LV_mmHg * mmHg2Pa;   % Pa
p_RV      = p_RV_mmHg * mmHg2Pa;   % Pa
lv_fast   = true;   % true → LV on quick-return stroke; false → LV on slow stroke
F_e       = 10;     % N
e_gb      = 0.90;
e_mech    = 0.72;
e_motor   = 0.83;
omega_gb  = 2*pi*rpm_max/60;
P_max     = 15.6;   % W budget

%% UI
fig = uifigure('Name','Teardrop Slot Mechanism','Position',[100 100 1280 580]);
ax  = uiaxes(fig,'Position',[30 50 530 500]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'mm'); ylabel(ax,'mm');
title(ax,'Crank-and-Slotted-Arm — Teardrop Slot Variant');

ax2 = uiaxes(fig,'Position',[905 35 345 250]);
hold(ax2,'on'); grid(ax2,'on');
xlabel(ax2,'Time (ms)'); ylabel(ax2,'P_{elec} (W)');
title(ax2,'Electrical Input Power');

%% Sliders  (7 rows, 60 px apart)
[lbl_x,   sld_x]   = mkSlider(580, 445, 'x',   [2   15],   x0);
[lbl_a,   sld_a]   = mkSlider(580, 385, 'a',   [15  40],   a0);
[lbl_r2,  sld_r2]  = mkSlider(580, 325, 'r2',  [1.3*r1  8],   r2_0);
[lbl_L,   sld_L]   = mkSlider(580, 265, 'L',   [10  80],   L0);
[lbl_Lc,  sld_Lc]  = mkSlider(580, 205, 'Lc',  [1   60],   Lc0);
[lbl_b,   sld_b]   = mkSlider(580, 145, 'b',   [0  0.5],   b0);
[lbl_phi, sld_phi] = mkSlider(580,  85, 'phi', [0  360],   phi);
[lbl_t,   sld_t]  = mkSlider(580,  25, 't',  [1   20],   t0);

btn = uibutton(fig,'Position',[740 15 120 35],'Text','Play', ...
    'ButtonPushedFcn',@(~,~) togglePlay());

%% Timer
tmr = timer('ExecutionMode','fixedRate','Period',0.04, ...
    'TimerFcn',@(~,~) stepAnim());
fig.CloseRequestFcn = @(~,~) closeFig();

%% Graphics handles (created once, updated each frame)
ang60 = linspace(0, 2*pi, 60)';

hArm    = plot(ax, NaN, NaN, '-',  'Color',[0.75 0.75 0.75],'LineWidth',7);
hSlot   = patch(ax, NaN, NaN, [0.35 0.75 0.35], ...
                'FaceAlpha',0.35,'EdgeColor',[0.15 0.55 0.15],'LineWidth',1.5);
hBagLV  = patch(ax, NaN(60,1), NaN(60,1), [0.18 0.44 0.85], ...
                'FaceAlpha',0.55,'EdgeColor',[0.10 0.30 0.70],'LineWidth',1.2);
hBagRV  = patch(ax, NaN(60,1), NaN(60,1), [0.85 0.22 0.22], ...
                'FaceAlpha',0.55,'EdgeColor',[0.65 0.10 0.10],'LineWidth',1.2);
hTxtLV  = text(ax,NaN,NaN,'','Color','w','FontSize',9,'FontWeight','bold', ...
               'HorizontalAlignment','center');
hTxtRV  = text(ax,NaN,NaN,'','Color','w','FontSize',9,'FontWeight','bold', ...
               'HorizontalAlignment','center');
hPath   = plot(ax, NaN, NaN, 'b:',  'LineWidth',1);
hCrank  = plot(ax, NaN, NaN, 'b-',  'LineWidth',2);
hNeut   = plot(ax, NaN, NaN, 'k:',  'LineWidth',1);
hPaddle = patch(ax, NaN(4,1), NaN(4,1), [0.15 0.15 0.15], ...
                'FaceAlpha',0.85,'EdgeColor','k','LineWidth',1.5);
hCtact  = plot(ax, NaN, NaN, 'r-',  'LineWidth',5);
hFulc   = plot(ax, NaN, NaN, 'ko',  'MarkerFaceColor','k','MarkerSize',8);
hCC     = plot(ax, NaN, NaN, 'bo',  'MarkerFaceColor','b','MarkerSize',8);
hPin    = plot(ax, NaN, NaN, 'ys',  'MarkerFaceColor','y','MarkerSize',8, ...
               'MarkerEdgeColor','b');
hPinCirc = patch(ax, NaN(60,1), NaN(60,1), [1.00 0.85 0.10], ...
                'FaceAlpha',0.7,'EdgeColor',[0.55 0.40 0],'LineWidth',1.2);

legend(ax, [hPaddle, hCtact, hSlot, hCrank, hBagLV, hBagRV, hPinCirc], ...
    {'Paddle (pivot\rightarrowtip)','Contact zone','Teardrop slot', ...
     'Crank arm','LV bag','RV bag','Crank pin (actual size)'}, ...
    'Location','southoutside');

T_pd0 = 60/rpm_max*1000;
plot(ax2, [0 T_pd0], [P_max P_max], 'r--', 'LineWidth',1);
text(ax2, T_pd0*0.03, P_max*1.08, sprintf('%.1f W budget',P_max), ...
     'Color','r','FontSize',8);
hPow    = plot(ax2, NaN, NaN, 'k-', 'LineWidth',1.5);
hPowDot = plot(ax2, NaN, NaN, 'ro', 'MarkerFaceColor','r','MarkerSize',10, ...
               'MarkerEdgeColor','k');

redraw();

%% ---- nested functions ----------------------------------------

    function [lbl, sld] = mkSlider(xp, yp, name, lims, val)
        lbl = uilabel(fig, 'Position',[xp yp+20 315 22]);
        sld = uislider(fig, 'Position',[xp yp 305 3], ...
            'Limits',lims, 'Value',val, ...
            'ValueChangingFcn',@(~,e) onSlide(name, e.Value));
        updLabel(lbl, name, val);
    end

    function updLabel(lbl, name, val)
        switch name
            case 'x',   lbl.Text = sprintf('Crank arm        x   = %.1f mm',  val);
            case 'a',   lbl.Text = sprintf('Crank\x2013pivot  a   = %.1f mm',  val);
            case 'r2',  lbl.Text = sprintf('Teardrop radius  r2  = %.2f mm  (pin r1=%.1f, clearance r_{eff}=%.2f)', ...
                                           val, r1, val-r1);
            case 'L',   lbl.Text = sprintf('Paddle length    L   = %.1f mm',  val);
            case 'Lc',  lbl.Text = sprintf('Contact length   Lc  = %.1f mm',  val);
            case 'b',   lbl.Text = sprintf('Bag overlap      b   = %.2f  (fill %.0f%%)', ...
                                           val, (1-val)*100);
            case 'phi', lbl.Text = sprintf('Crank angle      \x03c6   = %.0f\x00b0', val);
            case 't',   lbl.Text = sprintf('Paddle thickness t   = %.1f mm', val);
        end
    end

    function onSlide(name, val)
        if strcmp(name,'phi'), phi = val; end
        redraw();
    end

    function togglePlay()
        if strcmp(tmr.Running,'on')
            stop(tmr);  btn.Text = 'Play';
        else
            start(tmr); btn.Text = 'Pause';
        end
    end

    function stepAnim()
        phi = mod(phi + 3, 360);
        sld_phi.Value = phi;
        redraw();
    end

    function closeFig()
        stop(tmr); delete(tmr); delete(fig);
    end

    function redraw()
        xv    = sld_x.Value;
        av    = sld_a.Value;
        r2v   = max(1.3*r1, min(sld_r2.Value, r1 + 0.95*xv));   % r2 (wall) in [1.3*r1, r1+0.95x]
        r_eff = r2v - r1;   % pin-centre's effective arc radius (drives kinematics)
        Lv  = sld_L.Value;
        Lcv = min(sld_Lc.Value, Lv);
        bv  = sld_b.Value;
        tv  = sld_t.Value;

        updLabel(lbl_x,   'x',   xv);
        updLabel(lbl_a,   'a',   av);
        updLabel(lbl_r2,  'r2',  r2v);
        updLabel(lbl_L,   'L',   Lv);
        updLabel(lbl_Lc,  'Lc',  Lcv);
        updLabel(lbl_b,   'b',   bv);
        updLabel(lbl_phi, 'phi', phi);
        updLabel(lbl_t,   't',   tv);

        %% Crank kinematics (unchanged by slot shape)
        F    = [0, 0];
        C    = [av, 0];
        phir = deg2rad(phi);
        P    = [av - xv*cos(phir),  xv*sin(phir)];

        %% Teardrop arm angle — driven by the pin CENTRE's path (r_eff), not the wall (r2v)
        theta_d = td_theta(phi, xv, av, r_eff);
        theta_r = deg2rad(theta_d);
        arm_dir = [cos(theta_r),  sin(theta_r)];   % F → slot / crank side
        pad_dir = -arm_dir;                         % F → paddle tip

        tip    = F + Lv  * pad_dir;
        cInner = F + (Lv - Lcv) * pad_dir;

        %% Arm centreline: from paddle tip through F to slot apex
        apex = F + (av + xv) * arm_dir;
        set(hArm, 'XData',[tip(1) apex(1)], 'YData',[tip(2) apex(2)]);

        %% Paddle rectangle (thickness tv perpendicular to arm)
        perp = [sin(theta_r), -cos(theta_r)];
        h    = tv / 2;
        px   = [F(1)+h*perp(1), tip(1)+h*perp(1), tip(1)-h*perp(1), F(1)-h*perp(1)];
        py   = [F(2)+h*perp(2), tip(2)+h*perp(2), tip(2)-h*perp(2), F(2)-h*perp(2)];
        set(hPaddle, 'XData',px, 'YData',py);

        %% Actual slot wall outline in body frame → world frame
        % The pin-centre path (radius r_eff, apex at a+x) is first built exactly as
        % before, then scaled outward by k=r2v/r_eff about the arc centre (0,ci) to
        % get the real wall the pin (radius r1) rides inside — concentric r2v arc,
        % same tangent angles, apex pushed out past a+x to leave clearance for r1.
        % The far tip is NOT left as a sharp point either: at phi=180 the pin
        % centre sits exactly at (0,a+x), tangent to both wall lines at once (a
        % ball wedged in a V) — so the tip is capped with its own r1 fillet.
        ys = av - xv;   % bottom of pin-centre arc (body Y' = a-x)
        if r_eff < 1e-4
            % Degenerate: essentially straight slot; fall back to centreline
            sx = [ys*(cos(theta_r)),  apex(1)];
            sy = [ys*(sin(theta_r)),  apex(2)];
        else
            ci  = ys + r_eff;
            Di  = 2*xv - r_eff;
            Txi = r_eff * sqrt(max(0, Di^2 - r_eff^2)) / Di;
            Tyi = ci + r_eff^2 / Di;
            thR = atan2(Tyi - ci, Txi);

            k      = r2v / r_eff;            % pin-centre path -> actual wall
            Txw    = Txi * k;
            Tyw    = ci + (Tyi - ci) * k;
            apex_w = ci + (av + xv - ci) * k;

            % Tip fillet: circle of radius r1, centred on the axis, tangent to
            % both wall lines (same constant line-normal (Txw,Tyw-ci)/r2v used
            % at the near-pivot tangent point applies everywhere on the line)
            Yc  = apex_w - r1 * r2v / (Tyw - ci);
            Txf = r1 * Txw / r2v;
            Tyf = Yc + r1 * (Tyw - ci) / r2v;

            aa_big = linspace(thR, -pi - thR, 120);   % near-pivot r2v arc
            aa_tip = linspace(pi - thR, thR, 40);     % far-tip r1 fillet

            % Body-frame wall outline: near-pivot arc, (implicit line), tip fillet, (implicit line, closes)
            bX  = [r2v*cos(aa_big),    r1*cos(aa_tip)   ];
            bY  = [ci+r2v*sin(aa_big), Yc+r1*sin(aa_tip)];
            % Rotate to world frame
            sx  = bY*cos(theta_r) - bX*sin(theta_r);
            sy  = bY*sin(theta_r) + bX*cos(theta_r);
        end
        set(hSlot, 'XData', sx, 'YData', sy);

        %% Alpha (max paddle angle) via phi sweep — used for bag fill
        ps   = 0:5:360;
        ts   = arrayfun(@(p) td_theta(p, xv, av, r_eff), ps);
        ad   = max(ts);
        gd   = ad * bv / max(1e-9, 1-bv);
        den  = max(eps, ad + gd);
        fLV  = min(1, max(0, (ad - theta_d) / den));
        fRV  = min(1, max(0, (ad + theta_d) / den));

        %% Crank circle
        cang = linspace(0, 2*pi, 80);
        set(hPath,   'XData', C(1)+xv*cos(cang), 'YData', C(2)+xv*sin(cang));
        set(hCrank,  'XData', [C(1) P(1)], 'YData', [C(2) P(2)]);
        set(hNeut,   'XData', [F(1) C(1)], 'YData', [F(2) C(2)]);
        set(hCtact,  'XData', [cInner(1) tip(1)], 'YData', [cInner(2) tip(2)]);
        set(hFulc,   'XData', F(1),  'YData', F(2));
        set(hCC,     'XData', C(1),  'YData', C(2));
        set(hPin,    'XData', P(1),  'YData', P(2));
        set(hPinCirc,'XData', P(1)+r1*cos(ang60), 'YData', P(2)+r1*sin(ang60));

        %% Bags
        ar    = deg2rad(ad);
        rmb   = Lv * 0.4;
        bdist = Lv + rmb * 1.2;
        lcen  = bdist * [-cos(ar), -sin(ar)];
        rcen  = bdist * [-cos(ar),  sin(ar)];
        rlv   = max(rmb*0.08, fLV*rmb);
        rrv   = max(rmb*0.08, fRV*rmb);
        asp   = 1.3;
        set(hBagLV, 'XData', lcen(1)+rlv*cos(ang60), 'YData', lcen(2)+rlv*asp*sin(ang60));
        set(hBagRV, 'XData', rcen(1)+rrv*cos(ang60), 'YData', rcen(2)+rrv*asp*sin(ang60));
        set(hTxtLV, 'Position',[lcen(1) lcen(2) 0], 'String',sprintf('V = %.0f%%',fLV*100));
        set(hTxtRV, 'Position',[rcen(1) rcen(2) 0], 'String',sprintf('V = %.0f%%',fRV*100));

        %% Power curve
        phi_v   = (0:2:360)';
        th_v    = arrayfun(@(p) td_theta(p, xv, av, r_eff), phi_v);
        dth_v   = gradient(th_v, phi_v) * omega_gb;     % rad/s
        fast_ej = (th_v > -gd) & (dth_v > 0);          % fast stroke (phi~0, dtheta>0)
        slow_ej = (th_v <  gd) & (dth_v < 0);          % slow stroke (phi~180, dtheta<0)
        if lv_fast
            LV_ej = fast_ej;  RV_ej = slow_ej;
        else
            LV_ej = slow_ej;  RV_ej = fast_ej;
        end
        A_c    = w_fixed * Lcv * 1e-6;
        rm     = (Lv - Lcv/2) * 1e-3;
        Tp_LV  = (p_LV * A_c + F_e) * rm;
        Tp_RV  = (p_RV * A_c + F_e) * rm;
        P_v    = (Tp_LV * abs(dth_v) .* double(LV_ej) + ...
                  Tp_RV * abs(dth_v) .* double(RV_ej)) / (e_gb*e_mech*e_motor);
        T_pd   = 60/rpm_max*1000;
        t_v    = phi_v / 360 * T_pd;
        t_now  = phi   / 360 * T_pd;
        P_now  = interp1(phi_v, P_v, phi, 'linear', 0);
        set(hPow,    'XData',t_v,   'YData',P_v);
        set(hPowDot, 'XData',t_now, 'YData',P_now);
        xlim(ax2, [0, T_pd]);
        ylim(ax2, [0, max(max(P_v)*1.2, P_max*1.1)]);

        m = max([av+xv, Lv, norm(tip), bdist+rmb]) * 1.15;
        xlim(ax, [-m m]);  ylim(ax, [-m m]);
    end

    function theta = td_theta(phi_d, xv, av, r1v)
    % Teardrop arm angle for a single phi value (scalar).
        theta_o = atan2d(xv*sind(phi_d), av - xv*cosd(phi_d));
        if r1v < 1e-6
            theta = theta_o;  return;
        end
        ys  = av - xv;
        ci  = ys + r1v;
        Di  = 2*xv - r1v;
        if Di <= 0
            theta = theta_o;  return;
        end
        Txi = r1v * sqrt(max(0, Di^2 - r1v^2)) / Di;
        Tyi = ci + r1v^2 / Di;
        R_T = hypot(Txi, Tyi);
        R   = sqrt(av^2 + xv^2 - 2*av*xv*cosd(phi_d));
        sgn = 1 - 2*(phi_d > 180);   % +1 for phi<=180, -1 for phi>180

        if R <= R_T
            % Pin rides the arc
            Yp   = (R^2 - r1v^2 + ci^2) / (2*ci);
            Xp   = sqrt(max(0, R^2 - Yp^2));
            beta = atan2d(sgn*Xp, Yp);
        else
            % Pin rides the tangent line
            Ay  = av + xv;
            Ac  = Txi^2 + (Ay-Tyi)^2;
            Bc  = -2*Txi^2 + 2*Tyi*(Ay-Tyi);
            Cc  = R_T^2 - R^2;
            dsc = max(0, Bc^2 - 4*Ac*Cc);
            t1  = (-Bc + sqrt(dsc)) / (2*Ac);
            t2  = (-Bc - sqrt(dsc)) / (2*Ac);
            t  = t1;
            if t < 0 || t > 1
                d1 = max(0, t1-1) + max(0, -t1);
                d2 = max(0, t2-1) + max(0, -t2);
                if d2 < d1, t = t2; end
            end
            t   = max(0, min(1, t));
            Xp  = Txi*(1-t);
            Yp  = Tyi + t*(Ay-Tyi);
            beta = atan2d(sgn*Xp, Yp);
        end
        theta = theta_o - beta;
    end

end
