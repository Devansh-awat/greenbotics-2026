function walls = find_wall_black(frame, CFG)
% find_wall_black  Black-wall detection in the simulated camera frame.
%   DETECTION ONLY — not wired into any steering decision. The real robot
%   uses camera-based black-wall detection in two places (src/vision/
%   wall_base.py's mat-relative darkness scan during parking; rules.py's
%   _gap_to_wall during pillar passing), while this simulator's steering so
%   far only ever used geometric ray casts (ray_wall_distance.m). This gives
%   the simulated camera the same black-wall awareness at a coarse,
%   left/right pixel-coverage level, as a basis for a future control use
%   (and for telemetry/overlay), without changing current behaviour.
%
%   Same ROI crop and 2x downsample as find_biggest_block.m, so the areas
%   returned are directly comparable to a block's .area field.
%
%   walls.left_area / .right_area / .total_area — black-wall pixel count
%   (full-resolution units) left and right of the frame's vertical centre.

[H, ~, ~] = size(frame);

roi_top = round(H * CFG.CROP_TOP_FRAC) + 1;
roi_bot = round(H * (1 - CFG.CROP_BOTTOM_FRAC));
roi = frame(roi_top:2:roi_bot, 1:2:end, :);    % 2x downsample, matches find_biggest_block

hsv = rgb2hsv(roi);
Hc  = hsv(:,:,1) * 180;
Sc  = hsv(:,:,2) * 255;
Vc  = hsv(:,:,3) * 255;

black_mask = Hc>=CFG.LOWER_BLACK(1) & Hc<=CFG.UPPER_BLACK(1) & ...
             Sc>=CFG.LOWER_BLACK(2) & Sc<=CFG.UPPER_BLACK(2) & ...
             Vc>=CFG.LOWER_BLACK(3) & Vc<=CFG.UPPER_BLACK(3);

% Columns were subsampled 1:2:end over the full frame width, so splitting
% this downsampled mask at its own midpoint is the same frame-centre split.
mid_ds = round(size(black_mask,2) / 2);
walls.left_area  = sum(black_mask(:,1:mid_ds),     'all') * 4;   % x4: undo 2x2 downsample
walls.right_area = sum(black_mask(:,mid_ds+1:end), 'all') * 4;
walls.total_area = walls.left_area + walls.right_area;
end
