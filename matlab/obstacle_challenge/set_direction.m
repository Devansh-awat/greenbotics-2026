function P = set_direction(P, direction)
% set_direction  Configure everything that depends on the driving direction
%   ('ccw' or 'cw' — randomized per round in the real competition).
%   Sets: start pose (mirrored start zone), flow sign, lane heading, and
%   the parking lot placement (opposite half of the start section, against
%   the outer wall, per the official field description).

P.direction = direction;
if strcmp(direction, 'ccw')
    P.dir_sign = +1;
    P.start_x  = P.cx + 0.25/P.scale;    % zone Z3 (right half)
    P.start_th = pi;                     % heading west
    P.lane_th  = pi;                     % lane heading in the start section
else
    P.dir_sign = -1;
    P.start_x  = P.cx - 0.25/P.scale;    % zone Z4 (left half), mirrored
    P.start_th = 0;                      % heading east
    P.lane_th  = 0;
end
P.start_y = P.cy + 1.00/P.scale;

% ── Parking lot: start section, outer wall, opposite half from the start ──
% Two magenta barriers 200 x 20 x 100 mm, gap = 1.5 x robot length.
P.lot_len = 1.5 * P.robot_L;                          % 0.27 m
P.lot_x   = P.cx - P.dir_sign*(0.50/P.scale);         % lot centre (px)
hl        = (P.lot_len/2) / P.scale;
bt        = 0.02 / P.scale;                            % barrier thickness
by1       = P.cy + (1.5 - 0.20)/P.scale;               % 0.20 m into track
by2       = P.cy + 1.5/P.scale;                        % outer wall face

% AABBs [xmin xmax ymin ymax] in px
P.bars = [ P.lot_x - hl - bt,  P.lot_x - hl,      by1, by2 ;
           P.lot_x + hl,       P.lot_x + hl + bt, by1, by2 ];

% Pseudo-obstacles so the camera renders the barriers (magenta is outside
% the red/green HSV masks, so detection is unaffected)
P.barrier_obs = struct( ...
    'color', {'magenta','magenta'}, ...
    'x',     {P.lot_x - hl - bt/2, P.lot_x + hl + bt/2}, ...
    'y',     {(by1+by2)/2, (by1+by2)/2});

% Parking maneuver waypoints (world px)
P.stage_pt = [P.lot_x + P.dir_sign*(0.50/P.scale), P.cy + 1.20/P.scale];
P.align_pt = [P.lot_x,                             P.cy + 1.20/P.scale];
P.enter_pt = [P.lot_x,                             P.cy + 1.375/P.scale];
end
