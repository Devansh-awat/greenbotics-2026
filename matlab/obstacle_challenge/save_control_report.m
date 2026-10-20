function out_png = save_control_report(log, P, CFG, label)
% save_control_report  Control-system analysis of one run (6 panels).
%   Greenbotics steers with two loops, both shown here:
%     1. Camera P-controller  (same law as main_v3.py on the robot)
%          servo = KP_BLOCK * (cx - target_x), clamped to +/-45 deg
%          red pillar -> target_x = 20, green pillar -> target_x = 620
%     2. Heading loop          omega = 3 * heading error, clamped
%   The figure stays open after the run and a PNG is saved in results/.

if nargin < 4, label = 'run'; end
here    = fileparts(mfilename('fullpath'));
res_dir = fullfile(here, 'results');
if ~exist(res_dir, 'dir'), mkdir(res_dir); end
stamp   = datestr(now, 'yyyy-mm-dd_HHMMSS'); %#ok<TNOW1,DATST>

t      = log.t(:)';
servo  = log.servo(:)';
herr   = rad2deg(log.head_err(:)');
lane   = log.lane_err(:)' * 100;          % cm
avoid  = (log.state(:)' == 2);            % AVOIDING_BLOCK
SAT    = 45;
RED    = [0.85 0.10 0.10];
GREEN  = [0.00 0.60 0.00];
BLUE   = [0.00 0.45 0.80];

% ── Metrics ───────────────────────────────────────────────────────────────
[on, off] = runs_of(avoid);
ev_len = (off - on + 1) * P.dt;
keep   = ev_len >= 0.1;
on = on(keep);  off = off(keep);  ev_len = ev_len(keep);
n_ev   = numel(on);

hv        = herr(~isnan(herr));
rms_head  = sqrt(mean(hv.^2));
rms_lane  = sqrt(mean(lane.^2));
max_lane  = max(abs(lane));
if any(avoid)
    sat_pct  = 100 * mean(abs(servo(avoid)) >= SAT - 0.5);
    peak_srv = max(abs(servo(avoid)));
else
    sat_pct = 0;  peak_srv = 0;
end
if n_ev > 0, avg_pass = mean(ev_len); else, avg_pass = 0; end
smooth    = mean(abs(diff(log.omega))) / P.dt;     % rad/s^2
pct_avoid = 100 * mean(avoid);

% ── Figure ────────────────────────────────────────────────────────────────
fname = 'Greenbotics - Control Analysis';
fig = findobj('Type','figure','Name',fname);
if isempty(fig)
    fig = figure('Name',fname, 'NumberTitle','off', 'Color','w', ...
                 'Position',[120 60 1400 820]);
else
    fig = fig(1);  clf(fig);
end
try, fig.Theme = 'light'; catch, end
tl = tiledlayout(fig, 2, 3, 'TileSpacing','compact', 'Padding','compact');
title(tl, sprintf('Greenbotics WRO 2026 — Control analysis (%s)', label), ...
      'FontWeight','bold', 'FontSize',14);

% A. Camera P-controller output over the run, avoidance windows shaded
ax = nexttile(tl);  hold(ax,'on');
for k = 1:n_ev
    patch(ax, [t(on(k)) t(off(k)) t(off(k)) t(on(k))], [-50 -50 50 50], ...
          [1 0.85 0.55], 'EdgeColor','none', 'FaceAlpha',0.45, ...
          'HandleVisibility','off');
end
plot(ax, t, servo, 'Color',BLUE, 'LineWidth',1.1);
yline(ax,  SAT, 'r--', 'HandleVisibility','off');
yline(ax, -SAT, 'r--', 'HandleVisibility','off');
ylim(ax,[-50 50]);  grid(ax,'on');
xlabel(ax,'time (s)');  ylabel(ax,'servo command (deg)');
title(ax,'A. Camera P-controller output (shaded = avoiding a pillar)');

% B. Control law check: logged command vs camera error
ax = nexttile(tl);  hold(ax,'on');
ok = ~isnan(log.blk_err(:)') & avoid;
r  = ok & (log.blk_col(:)' == 1);
g  = ok & (log.blk_col(:)' == 2);
if any(r), scatter(ax, log.blk_err(r), servo(r), 10, RED,   'filled', 'MarkerFaceAlpha',0.35); end
if any(g), scatter(ax, log.blk_err(g), servo(g), 10, GREEN, 'filled', 'MarkerFaceAlpha',0.35); end
e = linspace(-640, 640, 400);
plot(ax, e, max(-SAT, min(SAT, CFG.KP_BLOCK*e)), 'k-', 'LineWidth',1.6);
grid(ax,'on');  xlim(ax,[-640 640]);  ylim(ax,[-50 50]);
xlabel(ax,'camera error  c_x - target_x  (px)');  ylabel(ax,'servo command (deg)');
title(ax, sprintf('B. Control law: servo = %.2f \\times error, clamped \\pm%d°', ...
      CFG.KP_BLOCK, SAT));
lg = {};
if any(r), lg{end+1} = 'red pillar';   end
if any(g), lg{end+1} = 'green pillar'; end
lg{end+1} = 'design law';
legend(ax, lg, 'Location','southeast');

% C. Every pillar pass overlaid, like a family of step responses
ax = nexttile(tl);  hold(ax,'on');
for k = 1:n_ev
    idx = on(k):off(k);
    c = log.blk_col(on(k));
    if c == 1, col = RED; elseif c == 2, col = GREEN; else, col = [0.5 0.5 0.5]; end
    plot(ax, t(idx) - t(on(k)), servo(idx), 'Color',[col 0.65], 'LineWidth',1.2);
end
yline(ax, 0, 'k:');
grid(ax,'on');  ylim(ax,[-50 50]);
xlabel(ax,'time since pillar detected (s)');  ylabel(ax,'servo command (deg)');
title(ax, sprintf('C. Pillar avoidance responses (%d passes, avg %.1f s)', n_ev, avg_pass));

% D. Heading loop
ax = nexttile(tl);
yyaxis(ax,'left');   plot(ax, t, herr, 'LineWidth',0.9);       ylabel(ax,'heading error (deg)');
yyaxis(ax,'right');  plot(ax, t, log.omega, 'LineWidth',0.9);  ylabel(ax,'\omega (rad/s)');
grid(ax,'on');  xlabel(ax,'time (s)');
title(ax, sprintf('D. Heading loop: \\omega = 3 \\times error  (RMS error %.1f°)', rms_head));

% E. Lane keeping
ax = nexttile(tl);  hold(ax,'on');
plot(ax, t, lane, 'Color',[0.45 0.2 0.7], 'LineWidth',1);
yline(ax, 0, 'k:');
grid(ax,'on');  xlabel(ax,'time (s)');  ylabel(ax,'offset from lane centre (cm)');
title(ax, sprintf('E. Lane keeping  (RMS %.1f cm, max %.1f cm)', rms_lane, max_lane));

% F. Scorecard
ax = nexttile(tl);  axis(ax,'off');
if log.mission_complete, verdict = 'MISSION COMPLETE'; else, verdict = 'TIME UP'; end
txt = { ...
    sprintf('Result               : %s', verdict), ...
    sprintf('Laps                 : %d / 3', log.laps_done), ...
    sprintf('Total time           : %.1f s', log.total_time), ...
    '', ...
    'CAMERA P-CONTROLLER', ...
    sprintf('  KP_BLOCK           : %.2f deg/px', CFG.KP_BLOCK), ...
    sprintf('  Pillar passes      : %d', n_ev), ...
    sprintf('  Avg pass time      : %.1f s', avg_pass), ...
    sprintf('  Peak servo         : %.1f deg', peak_srv), ...
    sprintf('  Time saturated     : %.0f %% of avoiding', sat_pct), ...
    sprintf('  Time avoiding      : %.0f %% of run', pct_avoid), ...
    '', ...
    'HEADING / LANE', ...
    sprintf('  RMS heading error  : %.1f deg', rms_head), ...
    sprintf('  RMS lane offset    : %.1f cm', rms_lane), ...
    sprintf('  Steering smoothness: %.2f rad/s^2', smooth), ...
    '', ...
    'SAFETY', ...
    sprintf('  Min pillar clear   : %.2f m', log.min_block_clear), ...
    sprintf('  Min wall clear     : %.2f m', log.min_wall_clear), ...
    sprintf('  Emergency reverses : %d', log.n_reverses) };
text(ax, 0.0, 1.0, txt, 'Units','normalized', 'VerticalAlignment','top', ...
     'FontName','Courier', 'FontSize',11, 'Interpreter','none');
title(ax, 'F. Scorecard');

drawnow;
out_png = fullfile(res_dir, sprintf('%s_control_%s.png', label, stamp));
saveas(fig, out_png);

fprintf(['[CONTROL] passes %d | avg pass %.1f s | peak servo %.0f deg | ' ...
         'RMS heading %.1f deg | RMS lane %.1f cm\n'], ...
        n_ev, avg_pass, peak_srv, rms_head, rms_lane);
end

function [on, off] = runs_of(m)
% Start/end indices of each run of true values in a logical vector.
d   = diff([0, double(m(:)'), 0]);
on  = find(d ==  1);
off = find(d == -1) - 1;
end
