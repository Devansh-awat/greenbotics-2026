function [P, CFG, track_img] = sim_setup(direction)
% sim_setup  Shared setup: config values, track geometry, sim parameters.
%   Used by run.m (interactive) and evaluate.m / simulate_run.m (batch).
%   direction: 'ccw' (default) or 'cw' — see set_direction.m.
if nargin < 1, direction = 'ccw'; end

here = fileparts(mfilename('fullpath'));

% ── Vision config (mirrors config.py / camera.py) ─────────────────────────
config;
CFG = struct();
CFG.FRAME_WIDTH               = FRAME_WIDTH;
CFG.FRAME_HEIGHT              = FRAME_HEIGHT;
CFG.CROP_TOP_FRAC             = CROP_TOP_FRAC;
CFG.CROP_BOTTOM_FRAC          = CROP_BOTTOM_FRAC;
CFG.MIN_CONTOUR_AREA          = MIN_CONTOUR_AREA;
CFG.MIN_BLOCK_AREA_FOR_ACTION = MIN_BLOCK_AREA_FOR_ACTION;
CFG.KP_BLOCK                  = KP_BLOCK;
CFG.RED_TARGET_X              = RED_TARGET_X;
CFG.GREEN_TARGET_X            = GREEN_TARGET_X;
CFG.CLEARANCE_FRAMES          = CLEARANCE_FRAMES;
CFG.GYRO_KP                   = GYRO_KP;
CFG.HEADING_LOCK_TOLERANCE    = HEADING_LOCK_TOLERANCE;
CFG.CAMERA_H_FOV              = 120.0;
CFG.LOWER_RED_1=LOWER_RED_1; CFG.UPPER_RED_1=UPPER_RED_1;
CFG.LOWER_RED_2=LOWER_RED_2; CFG.UPPER_RED_2=UPPER_RED_2;
CFG.LOWER_GREEN=LOWER_GREEN; CFG.UPPER_GREEN=UPPER_GREEN;

% ── Simulation parameters ─────────────────────────────────────────────────
P = struct();
P.dt         = 1/30;
P.total_time = 260.0;    % 3 laps + parking maneuver
P.nominal_v  = 0.18;
P.max_omega  = 2.2;
P.robot_L    = 0.180;
P.robot_W    = 0.150;
P.inner_r    = 0.50;
P.outer_r    = 1.35;
P.verbose    = true;
% Navigation mode:
%   'field' — vector-field navigation (validated: 10/10 random layouts)
%   'gyro'  — experimental port of the real robot's gyro/corner logic
%             (structure complete; completes laps on ~half the layouts,
%             tuning tracked in the engineering journal)
P.nav_mode    = 'field';
P.corner_dist = 0.72;     % front ToF distance (m) that triggers a corner turn

% ── Track image + geometry ────────────────────────────────────────────────
matfile = fullfile(here, 'mat.png');
if exist(matfile, 'file')
    track_img = imread(matfile);
    if size(track_img,3)==4, track_img = track_img(:,:,1:3); end
else
    track_img = make_track_image(800,800);
end
[IMG_H, IMG_W, ~] = size(track_img);
P.scale      = 3.0 / max(IMG_H, IMG_W);
P.cx         = IMG_W / 2;
P.cy         = IMG_H / 2;
P.ri_px      = P.inner_r / P.scale;
P.ro_px      = P.outer_r / P.scale;
P.rm_px      = (P.ri_px + P.ro_px) / 2;
P.half_in_w  = 0.5 / P.scale;
P.half_out_w = 1.5 / P.scale;

% Direction-dependent config: start pose, flow sign, parking lot placement
P = set_direction(P, direction);

P.n_steps = ceil(P.total_time / P.dt);
end
