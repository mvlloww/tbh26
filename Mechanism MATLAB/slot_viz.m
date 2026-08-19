function paddle_mechanism_viz
% PADDLE_MECHANISM_VIZ  Interactive view of the crank-and-slotted-arm LVAD
% paddle mechanism (geometry/formulas mirror paddle_angle_plot.m).
%
% Drag the sliders to change the mechanism dimensions (x, a, L, L_contact)
% and press Play to watch the paddle move through a full crank revolution.
%
% Geometry: fulcrum F fixed at origin, crank centre C fixed at (a,0).
% Crank pin P orbits C at radius x with crank angle phi:
%   P = (a - x*cos(phi), x*sin(phi))
% The paddle is the radial line F->P extended to length L (slot is along
% the arm), so its angle is theta = atan2(P_y, P_x) — identical to the
% theta(phi) formula in paddle_angle_plot.m.

%% Initial parameter values
x0  = 10;   % crank arm length, mm
a0  = 40;   % crank-centre to fulcrum distance, mm
L0  = 40;   % paddle length (radial extent from pivot), mm
Lc0 = 20;   % contact length from tip of paddle, mm
b0  = 0;    % bag overlap [0, 0.5]

phi = 0;    % current crank angle, deg

%% Figure & axes
fig = uifigure('Name','Paddle Mechanism Visualiser','Position',[100 100 900 560]);
ax  = uiaxes(fig,'Position',[40 40 540 480]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel(ax,'mm'); ylabel(ax,'mm');
title(ax,'Crank-and-Slotted-Arm Paddle Mechanism');

%% Controls
[lbl_x,   sld_x]   = addSlider(600, 480, 'x',   [2 30],  x0);
[lbl_a,   sld_a]   = addSlider(600, 410, 'a',   [20 80], a0);
[lbl_L,   sld_L]   = addSlider(600, 340, 'L',   [10 80], L0);
[lbl_Lc,  sld_Lc]  = addSlider(600, 270, 'Lc',  [1 80],  Lc0);
[lbl_b,   sld_b]   = addSlider(600, 200, 'b',   [0 0.5], b0);
[lbl_phi, sld_phi] = addSlider(600, 130, 'phi', [0 360], phi);

btn_play = uibutton(fig,'Position',[600 60 100 30],'Text','Play', ...
    'ButtonPushedFcn',@(~,~) togglePlay());

%% Animation timer (advances phi at a fixed visual rate)
animTimer = timer('ExecutionMode','fixedRate','Period',0.04, ...
    'TimerFcn',@(~,~) stepAnimation());
fig.CloseRequestFcn = @(~,~) closeFig();

%% Graphics objects (created once, updated in place each frame)
ang60 = linspace(0, 2*pi, 60)';

hBagLV       = patch(ax, NaN(60,1), NaN(60,1), [0.18 0.44 0.85], ...
                   'FaceAlpha',0.55, 'EdgeColor',[0.10 0.30 0.70], 'LineWidth',1.2);
hBagRV       = patch(ax, NaN(60,1), NaN(60,1), [0.85 0.22 0.22], ...
                   'FaceAlpha',0.55, 'EdgeColor',[0.65 0.10 0.10], 'LineWidth',1.2);
hTxtLV       = text(ax, NaN, NaN, '', 'Color','w', 'FontSize',9, ...
                   'FontWeight','bold', 'HorizontalAlignment','center');
hTxtRV       = text(ax, NaN, NaN, '', 'Color','w', 'FontSize',9, ...
                   'FontWeight','bold', 'HorizontalAlignment','center');
hCrankPath   = plot(ax, NaN, NaN, 'b:', 'LineWidth',1);
hCrankArm    = plot(ax, NaN, NaN, 'b-', 'LineWidth',2);
hNeutral     = plot(ax, NaN, NaN, 'k:', 'LineWidth',1);
hSlot        = plot(ax, NaN, NaN, '-',  'Color',[0.4 0.8 0.4],'LineWidth',2);
hPaddle      = plot(ax, NaN, NaN, 'k-', 'LineWidth',3);
hContact     = plot(ax, NaN, NaN, 'r-', 'LineWidth',5);
hFulcrum     = plot(ax, NaN, NaN, 'ko', 'MarkerFaceColor','k','MarkerSize',8);
hCrankCentre = plot(ax, NaN, NaN, 'bo', 'MarkerFaceColor','b','MarkerSize',8);
hPin         = plot(ax, NaN, NaN, 'ys', 'MarkerFaceColor','y','MarkerSize',8,'MarkerEdgeColor','b');

legend(ax, [hPaddle, hContact, hSlot, hCrankArm, hBagLV, hBagRV], ...
    {'Paddle (pivot -> tip)','Bag contact zone','Pin slot','Crank arm','LV bag','RV bag'}, ...
    'Location','southoutside');

redraw();

%% ---------------------------------------------------------------
%  Nested helpers (share this function's workspace)
%% ---------------------------------------------------------------
    function [lbl, sld] = addSlider(x, y, name, limits, value)
        lbl = uilabel(fig,'Position',[x y+18 270 22]);
        sld = uislider(fig,'Position',[x y 260 3], ...
            'Limits',limits,'Value',value, ...
            'ValueChangingFcn',@(~,e) onSlide(name, e.Value));
        setLabel(lbl, name, value);
    end

    function setLabel(lbl, name, value)
        switch name
            case 'x',   lbl.Text = sprintf('Crank arm length        x  = %.0f mm',  value);
            case 'a',   lbl.Text = sprintf('Crank-fulcrum distance  a  = %.0f mm',  value);
            case 'L',   lbl.Text = sprintf('Paddle length           L  = %.0f mm',  value);
            case 'Lc',  lbl.Text = sprintf('Contact length (tip)    Lc = %.0f mm',  value);
            case 'b',   lbl.Text = sprintf('Bag overlap             b  = %.2f  (fill %.0f%%)', value, (1-value)*100);
            case 'phi', lbl.Text = sprintf('Crank angle             phi = %.0f deg', value);
        end
    end

    function onSlide(name, value)
        if strcmp(name, 'phi')
            phi = value;
        end
        redraw();
    end

    function togglePlay()
        if strcmp(animTimer.Running, 'on')
            stop(animTimer);
            btn_play.Text = 'Play';
        else
            start(animTimer);
            btn_play.Text = 'Pause';
        end
    end

    function stepAnimation()
        phi = mod(phi + 3, 360);
        sld_phi.Value = phi;
        redraw();
    end

    function closeFig()
        stop(animTimer);
        delete(animTimer);
        delete(fig);
    end

    function redraw()
        x  = sld_x.Value;
        a  = sld_a.Value;
        L  = sld_L.Value;
        Lc = min(sld_Lc.Value, L);

        setLabel(lbl_x,   'x',   x);
        setLabel(lbl_a,   'a',   a);
        setLabel(lbl_L,   'L',   L);
        setLabel(lbl_Lc,  'Lc',  Lc);
        setLabel(lbl_b,   'b',   sld_b.Value);
        setLabel(lbl_phi, 'phi', phi);

        F = [0 0];                            % fulcrum (paddle pivot), fixed
        C = [a 0];                            % crank centre, fixed
        phir = deg2rad(phi);
        P = [a - x*cos(phir), x*sin(phir)];   % crank pin, slides in radial slot

        theta  = atan2(P(2), P(1));           % paddle angle, rad — matches theta(phi)
        slotv  = [cos(theta), sin(theta)];    % direction F -> P (slot side)
        paddlev = -slotv;                     % paddle extends opposite to pin slot
        tip    = F + L*paddlev;
        zoneIn = F + (L - Lc)*paddlev;

        ang = linspace(0, 2*pi, 80);
        set(hCrankPath,   'XData', C(1)+x*cos(ang),   'YData', C(2)+x*sin(ang));
        set(hCrankArm,    'XData', [C(1) P(1)],       'YData', [C(2) P(2)]);
        set(hNeutral,     'XData', [F(1) C(1)],       'YData', [F(2) C(2)]);
        set(hSlot,        'XData', [F(1) P(1)],       'YData', [F(2) P(2)]);
        set(hPaddle,      'XData', [F(1) tip(1)],     'YData', [F(2) tip(2)]);
        set(hContact,     'XData', [zoneIn(1) tip(1)],'YData', [zoneIn(2) tip(2)]);
        set(hFulcrum,     'XData', F(1), 'YData', F(2));
        set(hCrankCentre, 'XData', C(1), 'YData', C(2));
        set(hPin,         'XData', P(1), 'YData', P(2));

        % Bag fill calculation — mirrors corrected model in paddle_angle_plot.m
        b_val    = sld_b.Value;
        alpha_r  = asin(min(1, x/a));
        alpha_d  = rad2deg(alpha_r);
        gamma_d  = alpha_d * b_val / max(1e-9, 1 - b_val);
        theta_d  = rad2deg(theta);
        denom    = max(eps, alpha_d + gamma_d);
        fill_LV  = min(1, max(0, (alpha_d - theta_d) / denom));
        fill_RV  = min(1, max(0, (alpha_d + theta_d) / denom));

        lv_dir   = [-cos(alpha_r), -sin(alpha_r)];
        rv_dir   = [-cos(alpha_r),  sin(alpha_r)];
        r_max_bag = L * 0.40;
        bag_dist  = L + r_max_bag * 1.2;
        lv_cen   = bag_dist * lv_dir;
        rv_cen   = bag_dist * rv_dir;

        r_lv = max(r_max_bag * 0.08, fill_LV * r_max_bag);
        r_rv = max(r_max_bag * 0.08, fill_RV * r_max_bag);
        asp  = 1.3;
        set(hBagLV, 'XData', lv_cen(1) + r_lv*cos(ang60), ...
                    'YData', lv_cen(2) + r_lv*asp*sin(ang60));
        set(hBagRV, 'XData', rv_cen(1) + r_rv*cos(ang60), ...
                    'YData', rv_cen(2) + r_rv*asp*sin(ang60));
        set(hTxtLV, 'Position', [lv_cen(1), lv_cen(2), 0], ...
                    'String', sprintf('V = %.0f%%', fill_LV*100));
        set(hTxtRV, 'Position', [rv_cen(1), rv_cen(2), 0], ...
                    'String', sprintf('V = %.0f%%', fill_RV*100));

        m = max([a + x, L, abs(tip(1)), abs(tip(2)), bag_dist + r_max_bag]) * 1.15;
        xlim(ax, [-m m]);
        ylim(ax, [-m m]);
    end
end
