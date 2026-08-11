function P = mat_block_positions(cx, cy, scale)
% mat_block_positions  All 24 marked block squares on the WRO FE mat.
%   4 straight sections (N, E, S, W) × 3 tangential slots × 2 radial rows.
%   Measured from the official mat image:
%     radial rows      : 0.92 m and 1.12 m from mat centre
%     tangential slots : -0.5 m, 0, +0.5 m along the section
%   Returns struct array with fields x, y (px), section (1-4), slot (1-3).

t_off = [-0.5, 0, 0.5] / scale;    % tangential offsets in px
r_off = [0.90, 1.10]  / scale;     % official: 600/400 mm from outer wall

P   = struct('x',{},'y',{},'section',{},'slot',{});
idx = 0;
for s = 1:4
    for ti = 1:3
        for ri = 1:2
            idx = idx + 1;
            t = t_off(ti);
            r = r_off(ri);
            switch s
                case 1, x = cx + t; y = cy - r;   % North (top)
                case 2, x = cx + r; y = cy + t;   % East  (right)
                case 3, x = cx + t; y = cy + r;   % South (bottom)
                case 4, x = cx - r; y = cy + t;   % West  (left)
            end
            P(idx).x       = x;
            P(idx).y       = y;
            P(idx).section = s;
            P(idx).slot    = ti;
        end
    end
end
end
