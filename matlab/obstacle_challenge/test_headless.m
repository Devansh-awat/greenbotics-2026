% test_headless — quick validation of the shared control loop (no graphics)
addpath(fileparts(mfilename('fullpath')));
[P, CFG, ~] = sim_setup();
P.verbose = false;
rng(7);
obs = place_obstacles_random(P.cx, P.cy, 0, 0, P.scale, 0, 0);
tic; log = simulate_run(obs, P, CFG); el = toc;
fprintf('RESULT: complete=%d laps=%d park=%s touches=%d time=%.1fs minBlk=%.2fm revs=%d dir=%s (wall %.0fs)\n', ...
    log.mission_complete, log.laps_done, log.park_result, log.barrier_touches, ...
    log.total_time, log.min_block_clear, log.n_reverses, log.direction, el);
