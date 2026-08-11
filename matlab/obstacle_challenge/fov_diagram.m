function fov_diagram()
% fov_diagram  Top-down camera field-of-view diagram for the engineering
%   journal / GitHub documentation (Criterion 2: sensor placement
%   justified using field geometry).
%
%   Shows: the robot footprint, the camera mounting point, the FOV cone
%   (half-angle = CAMERA_H_FOV/2, matches render_frame.m / config.m),
%   the detection range, and worked examples of a pillar INSIDE vs
%   OUTSIDE the cone at a realistic corridor distance.
%
%   Values marked [MEASURE] are placeholders — replace with the camera's
%   actual datasheet FOV and its measured mounting offset on the chassis.

% ── Robot + camera geometry ────────────────────────────────────────────────
robot_L   = 0.180;     % m, chassis length
robot_W   = 0.150;     % m, chassis width
cam_off_x = 0.075;     % m, [MEASURE] camera forward of the robot's centre
cam_off_y = 0.0;       % m, [MEASURE] camera lateral offset (0 = centreline)
cam_h_fov = 120.0;     % deg, [MEASURE] horizontal FOV — matches CFG.CAMERA_H_FOV in config.m
range_m   = 2.0;       % m, detection range used in render_frame.m (range_px = 2.0/scale)

half_fov = deg2rad(cam_h_fov/2);
cam_x = cam_off_x;  cam_y = cam_off_y;     % camera position, robot frame (+x = forward)

% ── Example pillars (robot-frame coordinates, metres) ─────────────────────
% One clearly inside the cone, one clearly outside, at a realistic
% straight-section spacing (WRO signs sit ~0.4-1.0 m apart laterally).
pillars = struct( ...
    'x',     {1.20,        0.30}, ...
    'y',     {0.15,        0.65}, ...
    'color', {[0.85 0.1 0.1], [0.5 0.5 0.5]}, ...
    'label', {'Sign A - DETECTED', 'Sign B - OUTSIDE FOV (angle)'});

% ── Figure ──────────────────────────────────────────────────────────────
fig = figure('Color','w','Position',[80 80 1000 850]);
try, fig.Theme = 'light'; catch, end
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); box(ax,'on');
xlim(ax,[-0.3 2.2]); ylim(ax,[-1.1 1.1]);
xlabel(ax,'Forward distance from robot centre (m)');
ylabel(ax,'Lateral distance from robot centreline (m)');
title(ax, sprintf('Camera Field of View — %.0f\\circ horizontal, %.1f m range', cam_h_fov, range_m));

% FOV cone (filled wedge)
theta = linspace(-half_fov, half_fov, 60);
fx = cam_x + range_m*cos(theta);
fy = cam_y + range_m*sin(theta);
fill(ax, [cam_x, fx, cam_x], [cam_y, fy, cam_y], [1 0.85 0.55], ...
     'FaceAlpha',0.45, 'EdgeColor',[0.9 0.5 0], 'LineWidth',1.5);

% Range arc + boundary rays, labelled
plot(ax, cam_x + range_m*cos(theta), cam_y + range_m*sin(theta), '--', ...
     'Color',[0.9 0.5 0], 'LineWidth',1);
for s = [-1 1]
    plot(ax, [cam_x, cam_x+range_m*cos(s*half_fov)], ...
             [cam_y, cam_y+range_m*sin(s*half_fov)], ...
             '-', 'Color',[0.9 0.4 0], 'LineWidth',1.5);
end
text(ax, cam_x+range_m*cos(half_fov)*0.55, cam_y+range_m*sin(half_fov)*0.55+0.06, ...
     sprintf('%.0f\\circ', cam_h_fov/2), 'FontSize',10, 'Color',[0.7 0.35 0]);
text(ax, cam_x+range_m*1.01, cam_y-0.05, sprintf('%.1f m range', range_m), ...
     'FontSize',9, 'Color',[0.7 0.35 0]);

% Robot chassis (rectangle centred at origin, +x = forward)
rx = [-robot_L/2 robot_L/2 robot_L/2 -robot_L/2 -robot_L/2];
ry = [-robot_W/2 -robot_W/2 robot_W/2 robot_W/2 -robot_W/2];
fill(ax, rx, ry, [0.75 0.8 0.9], 'EdgeColor',[0.1 0.2 0.4], 'LineWidth',1.5);
quiver(ax, 0, 0, robot_L*0.55, 0, 0, 'Color',[0.1 0.2 0.4], 'LineWidth',2, ...
       'MaxHeadSize',0.8);
text(ax, 0, -robot_W/2-0.08, 'Robot chassis', 'HorizontalAlignment','center', 'FontSize',9);

% Camera mount marker
plot(ax, cam_x, cam_y, 'o', 'MarkerSize',9, 'MarkerFaceColor',[0.1 0.4 0.9], ...
     'MarkerEdgeColor','k', 'LineWidth',1);
text(ax, cam_x+0.03, cam_y+0.07, sprintf('Camera mount\n(%.0f mm fwd of centre)', cam_off_x*1000), ...
     'FontSize',8.5, 'Color',[0.1 0.3 0.7]);

% Pillars: show which are inside/outside the cone
for k = 1:numel(pillars)
    px = pillars(k).x; py = pillars(k).y;
    ang = atan2(py-cam_y, px-cam_x);
    dist = sqrt((px-cam_x)^2 + (py-cam_y)^2);
    inside = abs(ang) <= half_fov && dist <= range_m;
    if inside, mk = 's'; mfc = [0.85 0.1 0.1]; else, mk = 'x'; mfc = [0.4 0.4 0.4]; end
    plot(ax, px, py, mk, 'MarkerSize',12, 'MarkerFaceColor',mfc, ...
         'MarkerEdgeColor','k', 'LineWidth',1.5);
    text(ax, px+0.04, py, pillars(k).label, 'FontSize',8.5, 'Color',mfc*0.8);
end

legend(ax, {'FOV cone','range arc'}, 'Location','southoutside', 'Orientation','horizontal', ...
       'Box','off');

% ── Save ────────────────────────────────────────────────────────────────
here = fileparts(mfilename('fullpath'));
res  = fullfile(here,'results');
if ~exist(res,'dir'), mkdir(res); end
png = fullfile(res, 'fov_diagram.png');
saveas(fig, png);
fprintf('Saved: %s\n', png);
fprintf('NOTE: cam_off_x, cam_off_y and cam_h_fov are marked [MEASURE] in this\n');
fprintf('      file — replace with your camera''s real mount offset and datasheet FOV.\n');
end
