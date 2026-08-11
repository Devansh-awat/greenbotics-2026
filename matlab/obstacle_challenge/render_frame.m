function frame = render_frame(rx, ry, rtheta, obstacles, cx_px, cy_px, ri_px, ro_px, scale, CFG)
% render_frame  Simulated first-person camera — WRO FE competition view.
%   Optimized: static parts precomputed once (persistent), walls drawn on
%   a single 2D grey image, colour channels replicated once, pillars
%   written by direct indexed assignment. No full-frame temporaries.

FW = CFG.FRAME_WIDTH;    % 640
FH = CFG.FRAME_HEIGHT;   % 360

persistent gray_base cols_row persp
if isempty(gray_base) || size(gray_base,1) ~= FH || size(gray_base,2) ~= FW
    rows      = (1:FH)';
    floor_vec = uint8(185 + round(25 * rows / FH));   % floor gradient
    gray_base = repmat(floor_vec, 1, FW);
    cols_row  = 1:FW;
    persp     = 0.25 + 0.75 * (rows / FH);            % perspective taper
end

% ── Wall distances ────────────────────────────────────────────────────────
dist_c  = sqrt((rx - cx_px)^2 + (ry - cy_px)^2);
d_inner = max(0, dist_c - ri_px);
d_outer = max(0, ro_px - dist_c);
track_w = ro_px - ri_px;

ang_to_centre  = atan2(cy_px - ry, cx_px - rx);
rel_centre     = atan2(sin(ang_to_centre - rtheta), cos(ang_to_centre - rtheta));
centre_is_left = (rel_centre > 0);

inner_w = round(FW * 0.50 * exp(-1.8 * d_inner / track_w));
outer_w = round(FW * 0.50 * exp(-1.8 * d_outer / track_w));
inner_w = max(25, min(inner_w, round(FW * 0.48)));
outer_w = max(25, min(outer_w, round(FW * 0.48)));

if centre_is_left
    lw = min(round(inner_w * persp), FW-2);   % per-row left wall width
    rw = min(round(outer_w * persp), FW-2);
else
    lw = min(round(outer_w * persp), FW-2);
    rw = min(round(inner_w * persp), FW-2);
end

% ── Draw walls on one 2D grey image (implicit expansion, no repmat) ──────
gray = gray_base;
left_mask  = cols_row <= lw;              % FH x FW logical
right_mask = cols_row >= (FW - rw + 1);
gray(left_mask)  = 12;
gray(right_mask) = 12;

ew = 6;   % thin lighter edge where wall meets floor
gray((cols_row > lw - ew)             & left_mask)  = 60;
gray((cols_row < (FW - rw + 1 + ew))  & right_mask) = 60;

hor = max(2, min(round(FH * CFG.CROP_TOP_FRAC), FH-1));
gray(hor-1:hor+1, :) = 90;

frame = repmat(gray, 1, 1, 3);            % single 3-channel replication

% ── Pillars (direct indexed assignment — no channel copies) ──────────────
fov_rad  = deg2rad(CFG.CAMERA_H_FOV);
range_px = 2.0 / scale;

for k = 1:numel(obstacles)
    dx   = obstacles(k).x - rx;
    dy   = obstacles(k).y - ry;
    dist = sqrt(dx^2 + dy^2);
    if dist > range_px || dist < 8, continue; end

    ang_rel = atan2(sin(atan2(dy,dx) - rtheta), cos(atan2(dy,dx) - rtheta));
    if abs(ang_rel) > fov_rad/2, continue; end

    norm_x = ang_rel / (fov_rad/2);
    pcx    = round((norm_x + 1)/2 * FW);

    prox = (range_px - dist) / range_px;
    pw   = max(18, round(160 * prox));
    ph   = max(40, round(pw * 2.4));
    pw   = min(round(FW/2.2), pw);
    ph   = min(round(FH * 0.90), ph);

    x1 = max(1,  round(pcx - pw/2));
    x2 = min(FW, round(pcx + pw/2));
    y1 = max(1,  FH - ph);
    y2 = FH;

    switch obstacles(k).color
        case 'red',     rgb = [215,  30,  30];
        case 'green',   rgb = [ 25, 185,  40];
        otherwise,      rgb = [225,  25, 225];   % magenta parking barriers
    end

    frame(y1:y2, x1:x2, 1) = rgb(1);
    frame(y1:y2, x1:x2, 2) = rgb(2);
    frame(y1:y2, x1:x2, 3) = rgb(3);

    cap = y1 : min(y1 + round(ph*0.12), y2);
    frame(cap, x1:x2, 1) = min(255, rgb(1) + 70);
    frame(cap, x1:x2, 2) = min(255, rgb(2) + 70);
    frame(cap, x1:x2, 3) = min(255, rgb(3) + 70);
end
end
