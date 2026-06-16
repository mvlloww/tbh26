function paddle_mechanism_teardrop_viz
% PADDLE_MECHANISM_TEARDROP_VIZ  Interactive animation of the teardrop-slot
% LVAD paddle mechanism (geometry mirrors paddle_angle_teardrop.m).
%
% The teardrop replaces the sharp pivot-end of the radial slot with an arc
% of radius r1, reducing the quick-return ratio. Drag sliders to change
% parameters; press Play to animate through a full crank revolution.
%
% Frame: fulcrum F at origin, crank centre C at (a,0).
% Crank pin P = (a - x*cos(phi), x*sin(phi))  [unchanged by slot shape].
% Arm angle theta is teardrop-modified; it differs from atan2(Py,Px) when
% the pin rides the arc (near phi = 0 / 360).

%% Default parameters  (match paddle_angle_teardrop.m)
x0   = 9.9;
a0   = 28;
r1_0 = round(0.2 * x0, 2);   % 1.52 mm
L0   = 40;
Lc0  = 35;
b0   = 0.5;
phi  = 0;

%% UI
fig = uifigure('Name','Teardrop Slot Mechanism','Position',[100 100 920 580]);
ax  = uiaxes(fig,'Position',[30 50 530 500]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'mm'); ylabel(ax,'mm');
title(ax,'Crank-and-Slotted-Arm — Teardrop Slot Variant');

%% Sliders  (7 rows, 60 px apart)
[lbl_x,   sld_x]   = mkSlider(580, 445, 'x',   [2   15],   x0);
[lbl_a,   sld_a]   = mkSlider(580, 385, 'a',   [15  40],   a0);
[lbl_r1,  sld_r1]  = mkSlider(580, 325, 'r1',  [0    6],   r1_0);
[lbl_L,   sld_L]   = mkSlider(580, 265, 'L',   [10  80],   L0);
[lbl_Lc,  sld_Lc]  = mkSlider(580, 205, 'Lc',  [1   60],   Lc0);
[lbl_b,   sld_b]   = mkSlider(580, 145, 'b',   [0  0.5],   b0);
[lbl_phi, sld_phi] = mkSlider(580,  85, 'phi', [0  360],   phi);

btn = uibutton(fig,'Position',[580 35 120 35],'Text','Play', ...
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
hPaddle = plot(ax, NaN, NaN, 'k-',  'LineWidth',3);
hCtact  = plot(ax, NaN, NaN, 'r-',  'LineWidth',5);
hFulc   = plot(ax, NaN, NaN, 'ko',  'MarkerFaceColor','k','MarkerSize',8);
hCC     = plot(ax, NaN, NaN, 'bo',  'MarkerFaceColor','b','MarkerSize',8);
hPin    = plot(ax, NaN, NaN, 'ys',  'MarkerFaceColor','y','MarkerSize',8, ...
               'MarkerEdgeColor','b');

legend(ax, [hPaddle, hCtact, hSlot, hCrank, hBagLV, hBagRV], ...
    {'Paddle (pivot\rightarrowtip)','Contact zone','Teardrop slot', ...
     'Crank arm','LV bag','RV bag'}, ...
    'Location','southoutside');

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
            case 'r1',  lbl.Text = sprintf('Teardrop radius  r1  = %.2f mm  (%.0f%% of x)', ...
                                           val, 100*val/max(eps, sld_x.Value));
            case 'L',   lbl.Text = sprintf('Paddle length    L   = %.1f mm',  val);
            case 'Lc',  lbl.Text = sprintf('Contact length   Lc  = %.1f mm',  val);
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
        r1v = min(sld_r1.Value, 0.95*xv);   % r1 must be < x
        Lv  = sld_L.Value;
        Lcv = min(sld_Lc.Value, Lv);
        bv  = sld_b.Value;

        updLabel(lbl_x,   'x',   xv);
        updLabel(lbl_a,   'a',   av);
        updLabel(lbl_r1,  'r1',  r1v);
        updLabel(lbl_L,   'L',   Lv);
        updLabel(lbl_Lc,  'Lc',  Lcv);
        updLabel(lbl_b,   'b',   bv);
        updLabel(lbl_phi, 'phi', phi);

        %% Crank kinematics (unchanged by slot shape)
        F    = [0, 0];
        C    = [av, 0];
        phir = deg2rad(phi);
        P    = [av - xv*cos(phir),  xv*sin(phir)];

        %% Teardrop arm angle
        theta_d = td_theta(phi, xv, av, r1v);
        theta_r = deg2rad(theta_d);
        arm_dir = [cos(theta_r),  sin(theta_r)];   % F → slot / crank side
        pad_dir = -arm_dir;                         % F → paddle tip

        tip    = F + Lv  * pad_dir;
        cInner = F + (Lv - Lcv) * pad_dir;

        %% Arm centreline: from paddle tip through F to slot apex
        apex = F + (av + xv) * arm_dir;
        set(hArm, 'XData',[tip(1) apex(1)], 'YData',[tip(2) apex(2)]);

        %% Teardrop slot outline in body frame → world frame
        ys = av - xv;   % bottom of teardrop (body Y' = a-x)
        if r1v < 1e-4
            % Degenerate straight slot: draw centreline as 2-pt patch (line)
            sx = [ys*(cos(theta_r)),  apex(1)];
            sy = [ys*(sin(theta_r)),  apex(2)];
        else
            ci  = ys + r1v;
            Di  = 2*xv - r1v;
            Txi = r1v * sqrt(max(0, Di^2 - r1v^2)) / Di;
            Tyi = ci + r1v^2 / Di;
            thR = atan2(Tyi - ci, Txi);
            aa  = linspace(thR, -pi - thR, 120);
            % Body-frame outline (X' perpendicular, Y' along arm from F)
            bX  = [0,    Txi, r1v*cos(aa),    -Txi, 0    ];
            bY  = [av+xv, Tyi, ci+r1v*sin(aa), Tyi, av+xv];
            % Rotate to world frame
            sx  = bY*cos(theta_r) - bX*sin(theta_r);
            sy  = bY*sin(theta_r) + bX*cos(theta_r);
        end
        set(hSlot, 'XData', sx, 'YData', sy);

        %% Alpha (max paddle angle) via phi sweep — used for bag fill
        ps   = 0:5:360;
        ts   = arrayfun(@(p) td_theta(p, xv, av, r1v), ps);
        ad   = max(ts);
        gd   = ad * bv / max(1e-9, 1-bv);
        den  = max(eps, ad + gd);
        fLV  = min(1, max(0, (ad - theta_d) / den));
        fRV  = min(1, max(0, (ad + theta_d) / den));

        %% Crank circle
        cang = linspace(0, 2*pi, 80);
        set(hPath,   'XData', C(1)+xv*cos(cang), 'YData', C(2)+xv*sin(cang));
        set(hCrank,  'XData', [C(1) P(1)],        'YData', [C(2) P(2)]);
        set(hNeut,   'XData', [F(1) C(1)],        'YData', [F(2) C(2)]);
        set(hPaddle, 'XData', [F(1) tip(1)],      'YData', [F(2) tip(2)]);
        set(hCtact,  'XData', [cInner(1) tip(1)], 'YData', [cInner(2) tip(2)]);
        set(hFulc,   'XData', F(1),  'YData', F(2));
        set(hCC,     'XData', C(1),  'YData', C(2));
        set(hPin,    'XData', P(1),  'YData', P(2));

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
            t   = t1;
            if t < 0 || t > 1,  t = t2;  end
            Xp  = Txi*(1-t);
            Yp  = Tyi + t*(Ay-Tyi);
            beta = atan2d(sgn*Xp, Yp);
        end
        theta = theta_o - beta;
    end

end
