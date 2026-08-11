function [x, y, touched] = barrier_pushout(x, y, P, r_m)
% barrier_pushout  Solid collision between the robot (circle, radius r_m
%   metres) and the two magenta parking barriers (AABBs in P.bars).
%   Returns corrected position and whether a barrier was touched this step
%   (rule 9.24.7: touching a parking lot limitation ends the real round —
%   the sim counts touches instead of aborting so we can tune them to zero).

touched = false;
r = r_m / P.scale;
for b = 1:size(P.bars,1)
    bx1 = P.bars(b,1); bx2 = P.bars(b,2);
    by1 = P.bars(b,3); by2 = P.bars(b,4);
    qx = min(max(x, bx1), bx2);
    qy = min(max(y, by1), by2);
    dx = x - qx;  dy = y - qy;
    d  = sqrt(dx^2 + dy^2);
    if d < r
        touched = true;
        if d > 1e-9
            x = qx + dx/d*r;
            y = qy + dy/d*r;
        else
            y = by1 - r;   % degenerate: push into the track
        end
    end
end
end
