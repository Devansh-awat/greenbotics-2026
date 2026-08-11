function best = sweep_kp()
% sweep_kp  Closes the simulation → hardware loop for KP_BLOCK.
%
%   KP_BLOCK is the camera steering gain on the REAL robot
%   (main_v3.py: servo = KP_BLOCK * (cx - target_x), currently 0.09).
%   This sweep runs each candidate gain on the SAME six officially
%   randomized layouts (paired comparison) and measures pass rate,
%   completion time and worst block clearance.
%
%   The winning value is the sim's recommendation for config.py on the
%   physical robot — deploy it and verify with a stopwatch lap.

kp_values = [0.05 0.07 0.09 0.12 0.15];
n_layouts = 6;
seed0     = 100;

here = fileparts(mfilename('fullpath'));
addpath(here);
[P, CFG0, ~] = sim_setup();
P.verbose  = false;
P.nav_mode = 'field';

nk   = numel(kp_values);
pass = zeros(nk, n_layouts);
tsec = nan(nk, n_layouts);
mblk = zeros(nk, n_layouts);
revs = zeros(nk, n_layouts);

fprintf('Sweeping KP_BLOCK over %d values x %d layouts (paired seeds)...\n', nk, n_layouts);
t0 = tic;
for i = 1:nk
    CFG = CFG0;
    CFG.KP_BLOCK = kp_values(i);
    for j = 1:n_layouts
        rng(seed0 + j);   % same layout j for every KP value
        obstacles = place_obstacles_random(P.cx, P.cy, 0, 0, P.scale, 0, 0);
        log = simulate_run(obstacles, P, CFG);
        pass(i,j) = log.mission_complete;
        if log.mission_complete, tsec(i,j) = log.total_time; end
        mblk(i,j) = log.min_block_clear;
        revs(i,j) = log.n_reverses;
    end
    fprintf('  KP=%.2f : %d/%d pass  avg t=%5.1f s  worst clr=%.2f m  revs=%.1f\n', ...
        kp_values(i), sum(pass(i,:)), n_layouts, mean(tsec(i,:),'omitnan'), ...
        min(mblk(i,:)), mean(revs(i,:)));
end
fprintf('(sweep took %.0f s)\n', toc(t0));

% ── Pick the winner: max pass rate, then min avg time ─────────────────────
pr    = sum(pass,2);
tavg  = mean(tsec,2,'omitnan');
score = pr*1000 - tavg;                 % pass rate dominates, time breaks ties
[~, ib] = max(score);
best  = kp_values(ib);

fprintf('\nRECOMMENDATION: KP_BLOCK = %.2f', best);
if abs(best - 0.09) < 1e-9
    fprintf('  (current robot value 0.09 CONFIRMED by simulation)\n');
else
    fprintf('  (change from current 0.09 in src/obstacle_challenge/config.py)\n');
end

% ── Figure ─────────────────────────────────────────────────────────────────
res = fullfile(here,'results');
if ~exist(res,'dir'), mkdir(res); end
fig = figure('Visible','off','Color','w','Position',[60 60 1350 420]);
try, fig.Theme = 'light'; catch, end

labels = arrayfun(@(k) sprintf('%.2f',k), kp_values, 'UniformOutput', false);

subplot(1,3,1);
b = bar(categorical(labels, labels), pr/n_layouts*100, 'FaceColor',[0.25 0.55 0.9]);
b.FaceColor = 'flat'; b.CData(ib,:) = [0.15 0.75 0.3];
ylabel('pass rate (%)'); ylim([0 105]); grid on;
title(sprintf('3-lap pass rate (%d layouts each)', n_layouts));

subplot(1,3,2);
errorbar(kp_values, tavg, std(tsec,0,2,'omitnan'), '-o', 'LineWidth',1.5); hold on;
plot(kp_values(ib), tavg(ib), 'p', 'MarkerSize',15, ...
     'MarkerFaceColor',[0.15 0.75 0.3], 'MarkerEdgeColor','k');
xlabel('KP\_BLOCK'); ylabel('mean completion time (s)'); grid on;
title('Completion time (passed runs)');

subplot(1,3,3);
plot(kp_values, min(mblk,[],2), '-s', 'LineWidth',1.5); hold on;
yline(0.14, 'r--', 'contact limit');
xlabel('KP\_BLOCK'); ylabel('worst block clearance (m)'); grid on;
title('Safety margin vs gain');

png = fullfile(res, 'kp_sweep.png');
saveas(fig, png); close(fig);
save(fullfile(res,'kp_sweep.mat'), 'kp_values','pass','tsec','mblk','revs','best');
fprintf('Saved: %s\n', png);
end
