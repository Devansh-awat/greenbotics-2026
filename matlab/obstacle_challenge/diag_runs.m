function diag_runs(matfile, outpng, idx)
% diag_runs  One panel per run: trajectory coloured by state, pillars,
%   final position. Used to see where each run went wrong.
S = load(matfile); logs = S.logs; res = S.results;
[~, ~, track_img] = sim_setup();
if nargin < 3, idx = 1:numel(logs); end
n = numel(idx);
cols = [0 0.7 0; 1 0.55 0; 0.85 0.75 0; 0.9 0.1 0.1; 0.85 0.15 0.85];
fig = figure('Visible','off','Color','w','Position',[0 0 2400 1800]);
try, fig.Theme = 'light'; catch, end
for j = 1:n
    i = idx(j);
    subplot(min(3,n), ceil(n/min(3,n)), j); imshow(track_img); hold on;
    L = logs{i};
    for sid = 1:5
        m = L.state == sid;
        if any(m), plot(L.x(m), L.y(m), '.', 'Color', cols(sid,:), 'MarkerSize', 4); end
    end
    for k = 1:numel(L.obstacles)
        if strcmp(L.obstacles(k).color,'red'), oc=[1 0 0]; else, oc=[0 0.7 0]; end
        plot(L.obstacles(k).x, L.obstacles(k).y, 's', 'MarkerSize', 9, 'MarkerFaceColor', oc, 'MarkerEdgeColor','k');
    end
    plot(L.x(1), L.y(1), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    plot(L.x(end), L.y(end), 'kx', 'MarkerSize', 14, 'LineWidth', 3);
    if res(i).success, v='PASS'; else, v='FAIL'; end
    title(sprintf('%d %s %s laps %d revs %d', i, v, upper(L.direction), L.laps_done, L.n_reverses), 'FontSize', 14);
end
saveas(fig, outpng); close(fig);
end
