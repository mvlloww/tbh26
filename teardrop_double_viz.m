function teardrop_double_viz
% TEARDROP_DOUBLE_VIZ  Interactive animation of the double-radius slot LVAD
% mechanism. Extends teardrop_viz.m by adding a r2 slider.
%
% Sliders: x, a, r1, r2, L, Lc, b, phi   |  Play button for animation
% Power overlay: real-time P_elec vs time with moving dot at current phi.
%
% Double-radius slot geometry:
%   r1 arc at near-pivot end (reduces QR ratio).
%   r2 arcs replace straight slot sides (smooth dtheta/dphi, reduce power peaks).

%% Default parameters
x0   = 5.4;
a0   = 32.0;
r1_0 = 1.6;
r2_0 = 5.0;   % must be >= x0 - r1_0 = 3.8 mm
L0   = 57;
Lc0  = 50;
b0   = 0.5;
phi  = 0;

%% Physics for power overlay
w_fixed  = 100;
rpm_max  = 145;
p_bag    = 16e3;
F_e      = 10;
e_gb     = 0.90;
e_mech   = 0.72;
e_motor  = 0.83;
omega_gb = 2*pi*rpm_max/60;
P_max    = 15.6;

%% UI
fig = uifigure('Name','Double-Radius Slot Mechanism','Position',[100 100 1320 600]);
ax  = uiaxes(fig,'Position',[30 50 530 520]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'mm'); ylabel(ax,'mm');
title(ax,'Crank-and-Slotted-Arm — Double-Radius Slot');

ax2 = uiaxes(fig,'Position',[930 35 360 260]);
hold(ax2,'on'); grid(ax2,'on');
xlabel(ax2,'Time (ms)'); ylabel(ax2,'P_{elec} (W)');
title(ax2,'Electrical Input Power');

%% Sliders (8 rows, 55 px apart from y=490 down)
[lbl_x,   sld_x]   = mkSlider(580, 490, 'x',   [ 2   15],   x0);
[lbl_a,   sld_a]   = mkSlider(580, 435, 'a',   [15   40],   a0);
[lbl_r1,  sld_r1]  = mkSlider(580, 380, 'r1',  [ 0    6],   r1_0);
[lbl_r2,  sld_r2]  = mkSlider(580, 325, 'r2',  [ 0   30],   r2_0);
[lbl_L,   sld_L]   = mkSlider(580, 270, 'L',   [10   80],   L0);
[lbl_Lc,  sld_Lc]  = mkSlider(580, 215, 'Lc',  [ 1   60],   Lc0);
[lbl_b,   sld_b]   = mkSlider(580, 160, 'b',   [ 0  0.5],   b0);
[lbl_phi, sld_phi] = mkSlider(580, 105, 'phi', [ 0  360],   phi);

btn = uibutton(fig,'Position',[580 45 120 40],'Text','Play', ...
    'ButtonPushedFcn',@(~,~) togglePlay());

%% Timer
tmr = timer('ExecutionMode','fixedRate','Period',0.04, ...
    'TimerFcn',@(~,~) stepAnim());
fig.CloseRequestFcn = @(~,~) closeFig();

%% Graphics handles
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
hPath   = plot(ax, NaN, NaN, 'b:', 'LineWidth',1);
hCrank  = plot(ax, NaN, NaN, 'b-', 'LineWidth',2);
hNeut   = plot(ax, NaN, NaN, 'k:', 'LineWidth',1);
hPaddle = plot(ax, NaN, NaN, 'k-', 'LineWidth',3);
hCtact  = plot(ax, NaN, NaN, 'r-', 'LineWidth',5);
hFulc   = plot(ax, NaN, NaN, 'ko', 'MarkerFaceColor','k','MarkerSize',8);
hCC     = plot(ax, NaN, NaN, 'bo', 'MarkerFaceColor','b','MarkerSize',8);
hPin    = plot(ax, NaN, NaN, 'ys', 'MarkerFaceColor','y','MarkerSize',8,'MarkerEdgeColor','b');

legend(ax, [hPaddle, hCtact, hSlot, hCrank, hBagLV, hBagRV], ...
    {'Paddle','Contact zone','Double-radius slot','Crank arm','LV bag','RV bag'}, ...
    'Location','southoutside');

T_pd0 = 60/rpm_max*1000;
plot(ax2, [0 T_pd0], [P_max P_max], 'r--', 'LineWidth',1);
text(ax2, T_pd0*0.03, P_max*1.08, sprintf('%.1f W budget',P_max), 'Color','r','FontSize',8);
hPow    = plot(ax2, NaN, NaN, 'k-',  'LineWidth',1.5);
hPowDot = plot(ax2, NaN, NaN, 'ro',  'MarkerFaceColor','r','MarkerSize',10,'MarkerEdgeColor','k');

redraw();

%% ---- nested functions ----------------------------------------

    function [lbl, sld] = mkSlider(xp, yp, name, lims, val)
        lbl = uilabel(fig, 'Position',[xp yp+20 320 22]);
        sld = uislider(fig, 'Position',[xp yp 310 3], ...
            'Limits',lims, 'Value',val, ...
            'ValueChangingFcn',@(~,e) onSlide(name, e.Value));
        updLabel(lbl, name, val);
    end

    function updLabel(lbl, name, val)
        switch name
            case 'x',   lbl.Text = sprintf('Crank arm        x   = %.1f mm', val);
            case 'a',   lbl.Text = sprintf('Crank\x2013pivot  a   = %.1f mm', val);
            case 'r1',  lbl.Text = sprintf('r1 (teardrop)    r1  = %.2f mm  (%.0f%% of x)', ...
                                           val, 100*val/max(eps, sld_x.Value));
            case 'r2',  lbl.Text = sprintf('r2 (side arcs)   r2  = %.2f mm  (%.0f%% of x)', ...
                                           val, 100*val/max(eps, sld_x.Value));
            case 'L',   lbl.Text = sprintf('Paddle length    L   = %.1f mm', val);
            case 'Lc',  lbl.Text = sprintf('Contact length   Lc  = %.1f mm', val);
            case 'b',   lbl.Text = sprintf('Bag overlap      b   = %.2f  (fill %.0f%%)', ...
                                           val, (1-val)*100);
            case 'phi', lbl.Text = sprintf('Crank angle      \x03c6   = %.0f\x00b0', val);
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
        xv  = sld_x.Value;
        av  = sld_a.Value;
        r1v = min(sld_r1.Value, 0.95*xv);
        r2v = max(sld_r2.Value, xv - r1v + 0.1);  % r2 must be >= x-r1
        Lv  = sld_L.Value;
        Lcv = min(sld_Lc.Value, Lv);
        bv  = sld_b.Value;

        updLabel(lbl_x,   'x',   xv);
        updLabel(lbl_a,   'a',   av);
        updLabel(lbl_r1,  'r1',  r1v);
        updLabel(lbl_r2,  'r2',  r2v);
        updLabel(lbl_L,   'L',   Lv);
        updLabel(lbl_Lc,  'Lc',  Lcv);
        updLabel(lbl_b,   'b',   bv);
        updLabel(lbl_phi, 'phi', phi);

        %% Crank kinematics
        F    = [0, 0];
        C    = [av, 0];
        phir = deg2rad(phi);
        P    = [av - xv*cos(phir),  xv*sin(phir)];

        theta_d = td_double_theta(phi, xv, av, r1v, r2v);
        theta_r = deg2rad(theta_d);
        pad_dir = -[cos(theta_r), sin(theta_r)];

        tip    = F + Lv  * pad_dir;
        cInner = F + (Lv - Lcv) * pad_dir;
        arm_dir = -pad_dir;
        apex    = F + (av + xv) * arm_dir;

        set(hArm, 'XData',[tip(1) apex(1)], 'YData',[tip(2) apex(2)]);

        %% Slot outline in body frame -> world frame
        ys = av - xv;
        if r1v < 1e-4
            bX = [0 0]; bY = [ys, av+xv];
        else
            ci  = ys + r1v;
            Di  = 2*xv - r1v;
            if r2v < 1e-4 || Di <= 0
                % Single teardrop outline
                Txi = r1v*sqrt(max(0,Di^2-r1v^2))/Di;
                Tyi = ci + r1v^2/Di;
                thR = atan2(Tyi-ci, Txi);
                aa  = linspace(thR, -pi-thR, 120);
                r1X = r1v*cos(aa); r1Y = ci+r1v*sin(aa);
                bX  = [0,   Txi, r1X, -Txi, 0    ];
                bY  = [av+xv, Tyi, r1Y,  Tyi, av+xv];
            else
                % Double-radius outline
                dy   = (r1v*(2*xv + r2v) - 2*xv^2) / Di;
                Y2   = (av+xv) + dy;
                X2   = sqrt(max(r2v^2 - dy^2, 0));
                r_sm = r1v + r2v;
                Tp_x = r1v * X2 / r_sm;
                Tp_y = (ci*r2v + r1v*Y2) / r_sm;

                % r1 arc (bottom)
                thR  = atan2(Tp_y-ci, Tp_x);
                aa   = linspace(thR, -pi-thR, 120);
                r1X  = r1v*cos(aa); r1Y = ci+r1v*sin(aa);

                % r2 arc right side: apex -> tangent point, going through the
                % left side of the circle (clockwise through X'<0 region).
                % linspace(a_apex, a_Tp+2pi) stays in [a_apex, a_Tp+2pi]
                % which crosses pi and traces the concave inside wall.
                a_apex = atan2((av+xv)-Y2, -X2);
                a_Tp   = atan2(Tp_y-Y2,    Tp_x-X2);
                aa2  = linspace(a_apex, a_Tp + 2*pi, 80);
                r2X  = X2 + r2v*cos(aa2);   % body X' (apex->Tp)
                r2Y  = Y2 + r2v*sin(aa2);   % body Y'

                % outline: apex -> right r2 arc -> r1 arc -> left r2 (mirrored) -> apex
                bX = [0,     r2X,  r1X,  -fliplr(r2X), 0    ];
                bY = [av+xv, r2Y,  r1Y,   fliplr(r2Y), av+xv];
            end
        end
        % Rotate to world frame
        sx = bY*cos(theta_r) - bX*sin(theta_r);
        sy = bY*sin(theta_r) + bX*cos(theta_r);
        set(hSlot, 'XData',sx, 'YData',sy);

        %% Alpha and bag fill
        ps  = 0:5:360;
        ts  = arrayfun(@(p) td_double_theta(p, xv, av, r1v, r2v), ps);
        ad  = max(ts);
        gd  = ad * bv / max(1e-9, 1-bv);
        den = max(eps, ad + gd);
        fLV = min(1, max(0, (ad - theta_d) / den));
        fRV = min(1, max(0, (ad + theta_d) / den));

        %% Crank circle
        cang = linspace(0, 2*pi, 80);
        set(hPath,   'XData',C(1)+xv*cos(cang), 'YData',C(2)+xv*sin(cang));
        set(hCrank,  'XData',[C(1) P(1)],         'YData',[C(2) P(2)]);
        set(hNeut,   'XData',[F(1) C(1)],         'YData',[F(2) C(2)]);
        set(hPaddle, 'XData',[F(1) tip(1)],       'YData',[F(2) tip(2)]);
        set(hCtact,  'XData',[cInner(1) tip(1)],  'YData',[cInner(2) tip(2)]);
        set(hFulc,   'XData',F(1), 'YData',F(2));
        set(hCC,     'XData',C(1), 'YData',C(2));
        set(hPin,    'XData',P(1), 'YData',P(2));

        %% Bags
        ar    = deg2rad(ad);
        rmb   = Lv * 0.4;
        bdist = Lv + rmb * 1.2;
        lcen  = bdist * [-cos(ar), -sin(ar)];
        rcen  = bdist * [-cos(ar),  sin(ar)];
        rlv   = max(rmb*0.08, fLV*rmb);
        rrv   = max(rmb*0.08, fRV*rmb);
        set(hBagLV, 'XData',lcen(1)+rlv*cos(ang60), 'YData',lcen(2)+rlv*1.3*sin(ang60));
        set(hBagRV, 'XData',rcen(1)+rrv*cos(ang60), 'YData',rcen(2)+rrv*1.3*sin(ang60));
        set(hTxtLV, 'Position',[lcen(1) lcen(2) 0], 'String',sprintf('V=%.0f%%',fLV*100));
        set(hTxtRV, 'Position',[rcen(1) rcen(2) 0], 'String',sprintf('V=%.0f%%',fRV*100));

        %% Power curve
        phi_v = (0:2:360)';
        th_v  = arrayfun(@(p) td_double_theta(p, xv, av, r1v, r2v), phi_v);
        dth_v = gradient(th_v, phi_v) * omega_gb;
        LV_ej = (th_v > -gd) & (dth_v > 0);
        RV_ej = (th_v <  gd) & (dth_v < 0);
        Tp_v  = (p_bag * w_fixed*Lcv*1e-6 + F_e) * (Lv - Lcv/2)*1e-3;
        P_v   = Tp_v * abs(dth_v) .* double(LV_ej | RV_ej) / (e_gb*e_mech*e_motor);
        T_pd  = 60/rpm_max*1000;
        t_v   = phi_v / 360 * T_pd;
        t_now = phi   / 360 * T_pd;
        P_now = interp1(phi_v, P_v, phi, 'linear', 0);
        set(hPow,    'XData',t_v,   'YData',P_v);
        set(hPowDot, 'XData',t_now, 'YData',P_now);
        xlim(ax2, [0, T_pd]);
        ylim(ax2, [0, max(max(P_v)*1.2, P_max*1.1)]);

        m = max([av+xv, Lv, norm(tip), bdist+rmb]) * 1.15;
        xlim(ax, [-m m]);  ylim(ax, [-m m]);
    end

    function theta = td_double_theta(phi_d, xv, av, r1v, r2v)
        theta_o = atan2d(xv*sind(phi_d), av - xv*cosd(phi_d));
        if r1v < 1e-6; theta = theta_o; return; end
        if r2v > 0 && r2v < xv - r1v; r2v = 0; end  % invalid r2 → single teardrop
        ci  = (av - xv) + r1v;
        Di  = 2*xv - r1v;
        if Di <= 0; theta = theta_o; return; end
        R   = sqrt(av^2 + xv^2 - 2*av*xv*cosd(phi_d));
        sgn = 1 - 2*(phi_d > 180);

        if r2v < 1e-6
            Txi = r1v*sqrt(max(0,Di^2-r1v^2))/Di;
            Tyi = ci + r1v^2/Di;
            R_T = hypot(Txi,Tyi);
            if R <= R_T
                Yp = (R^2 - r1v^2 + ci^2)/(2*ci);
                Xp = sqrt(max(0, R^2 - Yp^2));
                beta = atan2d(sgn*Xp, Yp);
            else
                Ay=av+xv; Ac=Txi^2+(Ay-Tyi)^2; Bc=-2*Txi^2+2*Tyi*(Ay-Tyi); Cc=R_T^2-R^2;
                dsc=max(0,Bc^2-4*Ac*Cc);
                t1=(-Bc+sqrt(dsc))/(2*Ac); t2=(-Bc-sqrt(dsc))/(2*Ac); t=t1;
                if t<0||t>1
                    d1=max(0,t1-1)+max(0,-t1); d2=max(0,t2-1)+max(0,-t2);
                    if d2<d1; t=t2; end
                end
                t=max(0,min(1,t));
                beta=atan2d(sgn*Txi*(1-t), Tyi+t*(Ay-Tyi));
            end
        else
            dy   = (r1v*(2*xv+r2v) - 2*xv^2)/Di;
            Y2   = (av+xv) + dy;
            X2   = sqrt(max(r2v^2 - dy^2, 0));
            r_sm = r1v + r2v;
            Tp_x = r1v*X2/r_sm;
            Tp_y = (ci*r2v + r1v*Y2)/r_sm;
            R_T  = hypot(Tp_x, Tp_y);
            if R <= R_T
                Yp = (R^2 - r1v^2 + ci^2)/(2*ci);
                Xp = sqrt(max(0, R^2 - Yp^2));
                beta = atan2d(sgn*Xp, Yp);
            else
                if X2 > 1e-12
                    D2=X2^2+Y2^2; K=(D2+R^2-r2v^2)/2;
                    disc=max(D2*R^2-K^2,0);
                    Yp=(K*Y2+X2*sqrt(disc))/D2; Xp=(K-Y2*Yp)/X2;
                    beta=atan2d(sgn*Xp,Yp);
                else
                    beta=0;
                end
            end
        end
        theta = theta_o - beta;
    end

end
