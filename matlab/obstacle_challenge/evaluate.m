function results = evaluate(n_runs, seed)
% evaluate  Batch evaluation: run the robot on N official randomized
%   layouts (headless) and report success rate + statistics.
%
%   results = evaluate(20)       → 20 random layouts
%   results = evaluate(20, 42)   → reproducible with RNG seed 42
%
%   Saves a summary figure + results table into wro/results/.

if nargin < 1, n_runs = 10; end
if nargin < 2, seed = []; end
if ~isempty(seed), rng(seed); else, rng('shuffle'); end

here = fileparts(mfilename('fullpath'));
addpath(here);
[P, CFG, track_img] = sim_setup();
P.verbose = false;

results = struct('success',{},'full_park',{},'laps',{},'time',{}, ...
                 'min_block',{},'min_wall',{},'reverses',{},'n_obs',{}, ...
                 'touches',{},'dir',{});
logs = cell(1, n_runs);

fprintf('Evaluating %d randomized layouts (official WRO procedure, random direction)...\n', n_runs);
t0 = tic;
for i = 1:n_runs
    if rand < 0.5, P = set_direction(P, 'ccw'); else, P = set_direction(P, 'cw'); end
    obstacles = place_obstacles_random(P.cx, P.cy, 0, 0, P.scale, 0, 0);
    log = simulate_run(obstacles, P, CFG);
    logs{i} = log;
    results(i).success   = log.mission_complete;
    results(i).full_park = strcmp(log.park_result,'full');
    results(i).laps      = log.laps_done;
    results(i).time      = log.total_time;
    results(i).min_block = log.min_block_clear;
    results(i).min_wall  = log.min_wall_clear;
    results(i).reverses  = log.n_reverses;
    results(i).n_obs     = numel(obstacles);
    results(i).touches   = log.barrier_touches;
    results(i).dir       = log.direction;
    fprintf('  Run %2d/%d: %-4s %-5s laps %d/3  park=%-7s touch=%d  t=%5.1fs  revs=%d\n', ...
            i, n_runs, ternary(log.mission_complete,'PASS','FAIL'), ...
            upper(log.direction), log.laps_done, log.park_result, ...
            log.barrier_touches, log.total_time, log.n_reverses);
end
elapsed = toc(t0);

n_pass = sum([results.success]);
n_full = sum([results.full_park]);
fprintf('\n=====================================================\n');
fprintf('  RESULT: %d / %d missions complete (3 laps + parked)  (%.0f%%)\n', ...
        n_pass, n_runs, 100*n_pass/n_runs);
fprintf('  Full parallel parks (15 pts): %d / %d   Barrier touches: %d total\n', ...
        n_full, n_runs, sum([results.touches]));
fprintf('  Avg completion time (passes): %.1f s\n', ...
        mean([results([results.success]).time]));
fprintf('  Avg emergency reverses      : %.1f per run\n', mean([results.reverses]));
fprintf('  Worst block clearance       : %.2f m\n', min([results.min_block]));
fprintf('  (evaluation took %.0f s wall time)\n', elapsed);
fprintf('=====================================================\n');

% ── Summary figure ─────────────────────────────────────────────────────────
res_dir = fullfile(here, 'results');
if ~exist(res_dir,'dir'), mkdir(res_dir); end
stamp = datestr(now,'yyyy-mm-dd_HHMMSS'); %#ok<TNOW1,DATST>

fig = figure('Visible','off','Color','w','Position',[50 50 1500 600]);
try, fig.Theme = 'light'; catch, end   % avoid dark theme in batch mode

subplot(1,3,1);
imshow(track_img); hold on;
for i = 1:n_runs
    if results(i).success, c = [0 0.75 0.15 0.5]; else, c = [0.9 0.1 0.1 0.6]; end
    plot(logs{i}.x, logs{i}.y, '-', 'Color', c, 'LineWidth', 0.8);
end
title(sprintf('All %d trajectories (green=pass, red=fail)', n_runs));

subplot(1,3,2);
bar(1:n_runs, [results.laps], 'FaceColor',[0.2 0.5 0.9]); hold on;
yline(3,'g--','target');
xlabel('run'); ylabel('laps completed'); ylim([0 3.5]); grid on;
title(sprintf('Laps per run — %d/%d passed', n_pass, n_runs));

subplot(1,3,3);
histogram([results.min_block], 0:0.05:1, 'FaceColor',[0.85 0.3 0.2]); hold on;
xline(0.14,'k--','contact limit');
xlabel('min distance to any block (m)'); ylabel('runs'); grid on;
title('Closest approach to blocks');

out_png = fullfile(res_dir, sprintf('evaluation_%druns_%s.png', n_runs, stamp));
saveas(fig, out_png);
close(fig);
save(fullfile(res_dir, sprintf('evaluation_%druns_%s.mat', n_runs, stamp)), ...
     'results', 'logs');
fprintf('Saved: %s\n', out_png);
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end
