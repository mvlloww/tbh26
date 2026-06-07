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
[lbl_phi, sld_phi] = addSlider(600, 200, 'phi', [0 360], phi);

btn_play = uibutton(fig,'Position',[600 130 100 30],'Text','Play', ...
    'ButtonPushedFcn',@(~,~) togglePlay());

%% Animation timer (advances phi at a fixed visual rate)
animTimer = timer('ExecutionMode','fixedRate','Period',0.04, ...
    'TimerFcn',@(~,~) stepAnimation());
fig.CloseRequestFcn = @(~,~) closeFig();

%% Graphics objects (created once, updated in place each frame)
hCrankPath   = plot(ax, NaN, NaN, 'b:', 'LineWidth',1);
hCrankArm    = plot(ax, NaN, NaN, 'b-', 'LineWidth',2);
hNeutral     = plot(ax, NaN, NaN, 'k:', 'LineWidth',1);
hPaddle      = plot(ax, NaN, NaN, 'k-', 'LineWidth',3);
hContact     = plot(ax, NaN, NaN, 'r-', 'LineWidth',5);
hFulcrum     = plot(ax, NaN, NaN, 'ko', 'MarkerFaceColor','k','MarkerSize',8);
hCrankCentre = plot(ax, NaN, NaN, 'bo', 'MarkerFaceColor','b','MarkerSize',8);
hPin         = plot(ax, NaN, NaN, 'ys', 'MarkerFaceColor','y','MarkerSize',8,'MarkerEdgeColor','b');

legend(ax, [hPaddle, hContact, hCrankArm], ...
    {'Paddle (pivot -> tip)','Bag contact zone','Crank arm'}, 'Location','southoutside');

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
        setLabel(lbl_phi, 'phi', phi);

        F = [0 0];                            % fulcrum (paddle pivot), fixed
        C = [a 0];                            % crank centre, fixed
        phir = deg2rad(phi);
        P = [a - x*cos(phir), x*sin(phir)];   % crank pin, slides in radial slot

        theta  = atan2(P(2), P(1));           % paddle angle, rad — matches theta(phi)
        dirv   = [cos(theta), sin(theta)];
        tip    = F + L*dirv;
        zoneIn = F + (L - Lc)*dirv;

        ang = linspace(0, 2*pi, 80);
        set(hCrankPath,   'XData', C(1)+x*cos(ang),   'YData', C(2)+x*sin(ang));
        set(hCrankArm,    'XData', [C(1) P(1)],       'YData', [C(2) P(2)]);
        set(hNeutral,     'XData', [F(1) C(1)],       'YData', [F(2) C(2)]);
        set(hPaddle,      'XData', [F(1) tip(1)],     'YData', [F(2) tip(2)]);
        set(hContact,     'XData', [zoneIn(1) tip(1)],'YData', [zoneIn(2) tip(2)]);
        set(hFulcrum,     'XData', F(1), 'YData', F(2));
        set(hCrankCentre, 'XData', C(1), 'YData', C(2));
        set(hPin,         'XData', P(1), 'YData', P(2));

        m = max(a + x, L) * 1.15;
        xlim(ax, [-m m]);
        ylim(ax, [-m m]);
    end
end
