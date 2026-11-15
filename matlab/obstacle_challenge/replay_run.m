function replay_run(seed, which, mode, logfile)
% replay_run  Re-create run `which` of evaluate(n, seed) and simulate it
%   with verbose output, to see exactly where it goes wrong.
close all force; clear functions; %#ok<CLFUNC>
rng(seed);
[P, CFG] = sim_setup();
P.nav_mode = mode;
for i = 1:which
    if rand < 0.5, P = set_direction(P, 'ccw'); else, P = set_direction(P, 'cw'); end
    obstacles = place_obstacles_random(P.cx, P.cy, 0, 0, P.scale, 0, 0);
end
if exist(logfile,'file'), delete(logfile); end
diary(logfile);
fprintf('Replay seed %d run %d, %s, %s\n', seed, which, upper(P.direction), mode);
for k = 1:numel(obstacles)
    fprintf('pillar %d %s at (%.2f, %.2f) m\n', k, obstacles(k).color, ...
        (obstacles(k).x-P.cx)*P.scale, (obstacles(k).y-P.cy)*P.scale);
end
P.verbose = true;
S = init_run_state(P);
for step = 1:P.n_steps
    prev = S.state;
    S = control_step(S, obstacles, step, P, CFG);
    if ~strcmp(prev, S.state) || mod(step, 150) == 0
        fprintf('  t=%6.1f  %-18s pos (%.2f, %.2f) yaw %5.0f goal %5.0f turns %2d laps %d front %.2f\n', ...
            step*P.dt, S.state, (S.x-P.cx)*P.scale, (S.y-P.cy)*P.scale, ...
            S.current_yaw, S.goal_yaw, S.turns_done, S.laps_done, S.d_front);
    end
    if S.mission_complete, break; end
end
diary off;
end
