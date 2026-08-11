function block = find_biggest_block(frame, CFG)
% mirrors find_biggest_block() in camera.py
% Optimized: analyses the ROI at half resolution (2x downsample) — 4x less
% work in rgb2hsv/regionprops. Areas and coordinates are scaled back so
% thresholds and outputs stay in full-resolution units.

block = [];
[H, ~, ~] = size(frame);

roi_top = round(H * CFG.CROP_TOP_FRAC) + 1;
roi_bot = round(H * (1 - CFG.CROP_BOTTOM_FRAC));
roi = frame(roi_top:2:roi_bot, 1:2:end, :);    % 2x downsample

hsv = rgb2hsv(roi);
Hc  = hsv(:,:,1) * 180;
Sc  = hsv(:,:,2) * 255;
Vc  = hsv(:,:,3) * 255;

red_mask = ...
    ((Hc>=CFG.LOWER_RED_1(1) & Hc<=CFG.UPPER_RED_1(1) & ...
    Sc>=CFG.LOWER_RED_1(2) & Sc<=CFG.UPPER_RED_1(2) & ...
    Vc>=CFG.LOWER_RED_1(3) & Vc<=CFG.UPPER_RED_1(3)) | ...
    (Hc>=CFG.LOWER_RED_2(1) & Hc<=CFG.UPPER_RED_2(1) & ...
    Sc>=CFG.LOWER_RED_2(2) & Sc<=CFG.UPPER_RED_2(2) & ...
    Vc>=CFG.LOWER_RED_2(3) & Vc<=CFG.UPPER_RED_2(3)));

green_mask = ...
    Hc>=CFG.LOWER_GREEN(1) & Hc<=CFG.UPPER_GREEN(1) & ...
    Sc>=CFG.LOWER_GREEN(2) & Sc<=CFG.UPPER_GREEN(2) & ...
    Vc>=CFG.LOWER_GREEN(3) & Vc<=CFG.UPPER_GREEN(3);

best_area = 0;
min_area_ds = CFG.MIN_CONTOUR_AREA / 4;   % threshold in downsampled units

for pair = {red_mask, 'red'; green_mask, 'green'}'
    mask  = pair{1};
    cname = pair{2};
    stats = regionprops(logical(mask), 'Area', 'BoundingBox');
    for s = stats'
        if s.Area < min_area_ds, continue; end
        bw = s.BoundingBox(3);
        bh = s.BoundingBox(4);
        if bh <= bw, continue; end          % portrait check (camera.py line)
        area_full = s.Area * 4;             % scale back to full resolution
        if area_full > best_area
            best_area      = area_full;
            block.color    = cname;
            block.area     = area_full;
            block.bbox     = [s.BoundingBox(1)*2, ...
                s.BoundingBox(2)*2 + roi_top - 1, bw*2, bh*2];
            block.cx       = (s.BoundingBox(1) + bw/2) * 2;
            block.cy       = (s.BoundingBox(2) + bh/2)*2 + roi_top - 1;
        end
    end
end
end
