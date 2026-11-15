function run_eval(tag, mode, n, seed)
% run_eval  Evaluate one navigation mode and draw every run's path.
if nargin < 2, mode = 'gyro'; end
if nargin < 3, n = 12; end
if nargin < 4, seed = 42; end
close all force; clear functions; %#ok<CLFUNC>
if exist([tag '.txt'], 'file'), delete([tag '.txt']); end
diary([tag '.txt']);
evaluate(n, seed, mode);
diary off;
d = dir(fullfile('results', 'evaluation_*.mat'));
[~, i] = max([d.datenum]);
diag_runs(fullfile('results', d(i).name), [tag '_diag.png']);
fid = fopen([tag '.done'], 'w'); fclose(fid);
end
