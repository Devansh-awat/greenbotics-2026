function obs = place_obstacles(cx, cy, inner_r, outer_r, scale, n_red, n_green)
% Places obstacles evenly around the track so the robot will encounter them.
% Uses fixed positions at 60° intervals so they are spread around the ring.

ri = inner_r / scale;
ro = outer_r / scale;
r_mid = (ri + ro) / 2;   % place on the centreline of the track

colors = [repmat({'red'},1,n_red), repmat({'green'},1,n_green)];
n = numel(colors);

% Spread evenly around the track with a fixed seed offset
angles = linspace(0, 2*pi, n+1);
angles = angles(1:end-1) + pi/6;   % small offset from cardinal directions

obs = struct('color',{},'x',{},'y',{});
for k = 1:n
    obs(k).color = colors{k};
    obs(k).x     = cx + r_mid * cos(angles(k));
    obs(k).y     = cy + r_mid * sin(angles(k));
end

fprintf('Placed %d obstacles on track ring (r=%.0f px)\n', n, r_mid);
end
