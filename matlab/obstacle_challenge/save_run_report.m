function out_png = save_run_report(log, P, track_img, label)
% save_run_report  Renders a 4-panel report figure for one run and saves
%   PNG + MAT into wro/results/. Returns the PNG path.

here    = fileparts(mfilename('fullpath'));
res_dir = fullfile(here, 'results');
if ~exist(res_dir, 'dir'), mkdir(res_dir); end
stamp   = datestr(now, 'yyyy-mm-dd_HHMMSS'); %#ok<TNOW1,DATST>
if nargin < 4, label = 'run'; end

state_cols = [0 0.7 0; 1 0.55 0; 0.85 0.75 0; 0.9 0.1 0.1; 0.85 0.15 0.85];  % D/A/C/E/Park

fig = figure('Visible','off', 'Color','w', 'Position',[50 50 1400 900]);
try, fig.Theme = 'light'; catch, end   % avoid dark theme in batch mode

% ── A. Trajectory over the mat, colored by state ──────────────────────────
subplot(2,2,1);
imshow(track_img); hold on;
for b = 1:size(P.bars,1)
    rectangle('Position',[P.bars(b,1), P.bars(b,3), ...
        P.bars(b,2)-P.bars(b,1), P.bars(b,4)-P.bars(b,3)], ...
        'FaceColor',[1 0 1], 'EdgeColor',[0.4 0 0.4]);
end
for sid = 1:5
    m = (log.state == sid);
    if any(m)
        plot(log.x(m), log.y(m), '.', 'Color', state_cols(sid,:), 'MarkerSize', 5);
    end
end
for k = 1:numel(log.obstacles)
    if strcmp(log.obstacles(k).color,'red'), oc=[1 0 0]; else, oc=[0 0.7 0]; end
    plot(log.obstacles(k).x, log.obstacles(k).y, 's', 'MarkerSize',11, ...
         'MarkerFaceColor',oc, 'MarkerEdgeColor','k');
end
plot(P.start_x, P.start_y, 'o', 'MarkerSize',13, 'MarkerEdgeColor',[0 0.6 0.9], 'LineWidth',2);
plot(log.x(end), log.y(end), 'p', 'MarkerSize',16, ...
     'MarkerFaceColor',[0.2 0.9 0.4], 'MarkerEdgeColor','k');
title(sprintf('Trajectory (%s) — green=driving, orange=avoiding, yellow=clearing, red=reverse', label));

% ── B. Speed + omega vs time ──────────────────────────────────────────────
subplot(2,2,2);
yyaxis left;  plot(log.t, log.v, 'LineWidth',1); ylabel('Speed (m/s)');
yyaxis right; plot(log.t, log.omega, 'LineWidth',0.8); ylabel('\omega (rad/s)');
xlabel('t (s)'); grid on; title('Speed and steering');

% ── C. Clearances vs time ─────────────────────────────────────────────────
subplot(2,2,3);
plot(log.t, log.d_blk,  'r-', 'LineWidth',1); hold on;
plot(log.t, log.d_wall, 'b-', 'LineWidth',1);
yline(0.14, 'r--', 'block contact');
yline(0.0,  'b--', 'wall contact');
ylim([0 1.5]); grid on; xlabel('t (s)'); ylabel('distance (m)');
legend('nearest block','nearest wall','Location','northeast');
title(sprintf('Clearances  (min block %.2f m, min wall %.2f m)', ...
      log.min_block_clear, log.min_wall_clear));

% ── D. Summary text ───────────────────────────────────────────────────────
subplot(2,2,4); axis off;
if log.mission_complete, verdict = 'MISSION COMPLETE'; else, verdict = 'TIME UP'; end
lap_str = sprintf('%.1f  ', log.lap_times);
n_red   = sum(strcmp({log.obstacles.color},'red'));
n_green = sum(strcmp({log.obstacles.color},'green'));
txt = { sprintf('RESULT          : %s', verdict), ...
        sprintf('Direction       : %s', upper(log.direction)), ...
        sprintf('Laps completed  : %d / 3', log.laps_done), ...
        sprintf('Lap times (s)   : %s', lap_str), ...
        sprintf('Parking         : %s  (touches: %d)', upper(log.park_result), log.barrier_touches), ...
        sprintf('Total time      : %.1f s', log.total_time), ...
        sprintf('Avg speed       : %.3f m/s', mean(abs(log.v))), ...
        sprintf('Min block clear : %.2f m', log.min_block_clear), ...
        sprintf('Min wall clear  : %.2f m', log.min_wall_clear), ...
        sprintf('Emergency revs  : %d', log.n_reverses), ...
        sprintf('Obstacles       : %d red, %d green', n_red, n_green) };
text(0.05, 0.9, txt, 'FontName','Courier', 'FontSize',13, 'VerticalAlignment','top');
title('Run summary');

out_png = fullfile(res_dir, sprintf('%s_%s.png', label, stamp));
saveas(fig, out_png);
save(fullfile(res_dir, sprintf('%s_%s.mat', label, stamp)), 'log');
close(fig);
end
