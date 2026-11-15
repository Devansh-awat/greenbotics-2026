function build_state_machine()
% build_state_machine  Stateflow model of the simulator's navigation logic.
%
%   Builds greenbotics_state_machine.slx next to this file: the same states
%   and transition rules as control_step.m ('field' mode) and
%   parking_control.m, as an executable Stateflow chart. It then runs a
%   scripted 14-second scenario through the chart and checks every expected
%   state change, printing PASS/FAIL for each check.
%
%   Requires Simulink and Stateflow. Run from MATLAB:
%       >> build_state_machine
%
%   What the chart models: the decision logic only. Its inputs (pillar seen,
%   pillar area, distances, lap count, parking checkpoints) are the values
%   control_step.m computes each step from the simulated camera, ToF rays
%   and robot pose. Transition timing matches the MATLAB simulator to within
%   one 30 Hz frame.
%
%   States (state_id output, same numbering as run.m's state_num):
%     1 DRIVING_STRAIGHT   2 AVOIDING_BLOCK   3 CLEARING
%     4 EMERGENCY_REVERSE  5 PARK (park_phase 1-5)   6 MISSION_COMPLETE

assert(license('test', 'Simulink') && license('test', 'Stateflow'), ...
    'build_state_machine needs Simulink and Stateflow licences.');

here = fileparts(mfilename('fullpath'));
mdl  = 'greenbotics_state_machine';
slx  = fullfile(here, [mdl '.slx']);

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(slx, 'file'), delete(slx); end

%% ── Model and chart ─────────────────────────────────────────────────────
new_system(mdl);
chartName = 'Navigation State Machine';
chartBlk  = [mdl '/' chartName];
add_block('sflib/Chart', chartBlk, 'Position', [320 40 600 520]);
ch = find(sfroot, '-isa', 'Stateflow.Chart', 'Path', chartBlk);
ch.ActionLanguage = 'MATLAB';
try   % older releases only; newer ones always use the order set below
    ch.UserSpecifiedStateTransitionExecutionOrder = true;
catch
end

%% ── Data ────────────────────────────────────────────────────────────────
% Inputs (one per chart input port, in this order)
inNames = {'block_seen', ...     % 1/0: a red or green pillar is in the camera ROI
           'block_area', ...     % px^2: area of the biggest pillar blob
           'block_ahead_m', ...  % m: nearest pillar within +/-50 deg of heading
           'stuck', ...          % 1/0: watchdog (<6 cm moved in 3 s)
           'back_wall_m', ...    % m: rear ray distance to a wall
           'laps_done', ...      % laps completed
           'at_stage', ...       % 1/0: reached the parking staging point
           'at_align', ...       % 1/0: centred on the parking lot
           'facing_lot', ...     % 1/0: turned to face the lot
           'inside_lot', ...     % 1/0: inside the lot depth
           'parked_ok'};         % 1/0: between barriers, inside, parallel
for k = 1:numel(inNames)
    d = addData(ch, inNames{k}, 'Input');
    d.Port = k;
end

outNames = {'state_id', 'park_phase'};
for k = 1:numel(outNames)
    d = addData(ch, outNames{k}, 'Output', '0');
    d.Port = k;
end

% Counters
addData(ch, 'frames_lost',  'Local', '0');
addData(ch, 'emerg_frames', 'Local', '0');
addData(ch, 'cooldown',     'Local', '0');
addData(ch, 'park_timer',   'Local', '0');

% Thresholds (values from config.m / control_step.m / parking_control.m)
addData(ch, 'MIN_AREA',         'Constant', '1500');  % MIN_BLOCK_AREA_FOR_ACTION
addData(ch, 'CLEARANCE_FRAMES', 'Constant', '45');
addData(ch, 'EMERG_DIST_M',     'Constant', '0.19');
addData(ch, 'BACK_WALL_MIN_M',  'Constant', '0.22');
addData(ch, 'REVERSE_FRAMES',   'Constant', '24');
addData(ch, 'COOLDOWN_FRAMES',  'Constant', '45');
addData(ch, 'PARK_TIMEOUT',     'Constant', '150');

%% ── States ──────────────────────────────────────────────────────────────
% Layout: DRIVE (top left) with its three camera states, EMERGENCY_REVERSE
% to its right, the five parking steps in a row below, MISSION_COMPLETE
% under the last parking step.
drive = addState(ch, sprintf(['DRIVE\nentry: park_phase = 0;\n' ...
        'during: cooldown = max(0, cooldown - 1);']), [30 60 760 400]);
ds = addState(drive, sprintf('DRIVING_STRAIGHT\nentry: state_id = 1;'), ...
        [70 160 220 90]);
av = addState(drive, sprintf('AVOIDING_BLOCK\nentry: state_id = 2;'), ...
        [470 160 240 100]);
cl = addState(drive, sprintf(['CLEARING\nentry: state_id = 3; frames_lost = 0;\n' ...
        'during: frames_lost = frames_lost + 1;']), [470 320 280 100]);

er = addState(ch, sprintf(['EMERGENCY_REVERSE\n' ...
        'entry: state_id = 4; emerg_frames = 0;\n' ...
        'during: emerg_frames = emerg_frames + 1;\n' ...
        'exit: cooldown = COOLDOWN_FRAMES;']), [900 160 380 130]);

park = addState(ch, sprintf('PARK\nentry: state_id = 5;'), [30 540 1580 200]);
px = @(k) 70 + (k-1)*320;                     % 170 wide boxes, 150 gaps
pa = addState(park, sprintf('APPROACH\nentry: park_phase = 1;'), [px(1) 620 170 90]);
pl = addState(park, sprintf('ALIGN\nentry: park_phase = 2;'),    [px(2) 620 170 90]);
pt = addState(park, sprintf('TURN_IN\nentry: park_phase = 3;'),  [px(3) 620 170 90]);
pe = addState(park, sprintf('ENTER\nentry: park_phase = 4;'),    [px(4) 620 170 90]);
ps = addState(park, sprintf(['STRAIGHTEN\nentry: park_phase = 5; park_timer = 0;\n' ...
        'during: park_timer = park_timer + 1;']), [px(5) 620 220 90]);

done = addState(ch, sprintf('MISSION_COMPLETE\nentry: state_id = 6;'), ...
        [px(5) 820 220 80]);

%% ── Transitions ─────────────────────────────────────────────────────────
% addTrans(parent, from, to, label, order, fromClock, toClock, labelXY)
addDefault(ch, drive);
addDefault(drive, ds);
addDefault(park, pa);

% Inside DRIVE: camera pillar logic (control_step.m state machine)
addTrans(drive, ds, av, '[block_seen > 0.5 && block_area > MIN_AREA]', [], 3, 9, [300 125]);
addTrans(drive, av, cl, '[block_seen < 0.5]', [], 5, 1, [690 280]);
addTrans(drive, cl, av, '[block_seen > 0.5]', 1, 11, 7, [370 280]);
addTrans(drive, cl, ds, '[frames_lost >= CLEARANCE_FRAMES]', 2, 9, 6, [110 390]);

% Leaving DRIVE: 3 laps -> park (checked first), else emergency reverse
addTrans(ch, drive, park, '[laps_done >= 3]', 1, 6, 0, [420 490]);
addTrans(ch, drive, er, ...
    '[stuck > 0.5 || (block_ahead_m < EMERG_DIST_M && cooldown <= 0)]', 2, 2, 10, [800 100]);
addTrans(ch, er, drive, ...
    '[emerg_frames >= REVERSE_FRAMES || back_wall_m < BACK_WALL_MIN_M]', [], 8, 4, [810 330]);

% Parking sequence (parking_control.m)
addTrans(park, pa, pl, '[at_stage > 0.5]',   [], 3, 9, [px(1)+178 640]);
addTrans(park, pl, pt, '[at_align > 0.5]',   [], 3, 9, [px(2)+178 640]);
addTrans(park, pt, pe, '[facing_lot > 0.5]', [], 3, 9, [px(3)+178 640]);
addTrans(park, pe, ps, '[inside_lot > 0.5]', [], 3, 9, [px(4)+178 640]);
addTrans(ch, ps, done, '[parked_ok > 0.5 || park_timer > PARK_TIMEOUT]', [], 6, 0, [px(5)-340 765]);

%% ── Ports and solver ────────────────────────────────────────────────────
for k = 1:numel(inNames)
    y0 = 40 + 42*(k-1);
    add_block('simulink/Sources/In1', [mdl '/' inNames{k}], ...
        'Position', [120 y0 150 y0+14], 'SampleTime', '1/30', ...
        'PortDimensions', '1', 'OutDataTypeStr', 'double');
    add_line(mdl, [inNames{k} '/1'], sprintf('%s/%d', chartName, k));
end
for k = 1:numel(outNames)
    y0 = 200 + 80*(k-1);
    add_block('simulink/Sinks/Out1', [mdl '/' outNames{k}], ...
        'Position', [700 y0 730 y0+14]);
    add_line(mdl, sprintf('%s/%d', chartName, k), [outNames{k} '/1']);
end

N = 420;                       % 14 s at 30 Hz
set_param(mdl, 'Solver', 'FixedStepDiscrete', 'FixedStep', '1/30', ...
    'StopTime', num2str((N-1)/30, 10), ...
    'SaveTime', 'on', 'SaveOutput', 'on', 'SaveFormat', 'Array', ...
    'ReturnWorkspaceOutputs', 'on');

%% ── Test scenario ───────────────────────────────────────────────────────
f = (0:N-1)';
t = f/30;
U = zeros(N, numel(inNames));
U(:, 3) = 9;                                   % no pillar close ahead
U(:, 5) = 9;                                   % rear wall far away
seen = (f >= 30 & f < 90) | (f >= 110 & f < 130);
U(:, 1) = seen;                                % pillar visible twice
U(seen, 2) = 3000;                             % big enough to act on
U(f >= 200 & f < 210, 3) = 0.15;               % pillar suddenly too close
U(f >= 260, 6)  = 3;                           % third lap completed
U(f >= 300, 7)  = 1;                           % parking checkpoints
U(f >= 320, 8)  = 1;
U(f >= 340, 9)  = 1;
U(f >= 360, 10) = 1;
U(f >= 380, 11) = 1;

% frame, expected state_id, expected park_phase, what is being checked
checks = {
     15, 1, 0, 'starts driving straight'
     60, 2, 0, 'big pillar seen -> avoiding'
    100, 3, 0, 'pillar lost -> clearing'
    120, 2, 0, 'pillar back while clearing -> avoiding again'
    150, 3, 0, 'pillar lost again -> clearing'
    190, 1, 0, 'clear for 45 frames -> driving straight'
    205, 4, 0, 'pillar closer than 0.19 m -> emergency reverse'
    240, 1, 0, 'reversed 24 frames -> back to driving'
    280, 5, 1, '3 laps done -> park: approach'
    310, 5, 2, 'staging reached -> align'
    330, 5, 3, 'centred on lot -> turn in'
    350, 5, 4, 'facing lot -> enter'
    370, 5, 5, 'inside lot -> straighten'
    400, 6, 5, 'parked -> mission complete'};

simIn = Simulink.SimulationInput(mdl);
simIn = simIn.setExternalInput([t U]);
out   = sim(simIn);
tout  = out.tout;
yout  = out.yout;

fprintf('\nStateflow chart check: scripted scenario, %d checks\n', size(checks, 1));
fprintf('%-6s %-10s %-10s %-6s %s\n', 'frame', 'state_id', 'park', 'result', 'check');
nPass = 0;
for k = 1:size(checks, 1)
    [fr, wantS, wantP, what] = checks{k, :};
    [~, i] = min(abs(tout - fr/30));
    gotS = yout(i, 1);
    gotP = yout(i, 2);
    ok = (gotS == wantS) && (wantS ~= 5 || gotP == wantP);
    nPass = nPass + ok;
    if ok, res = 'PASS'; else, res = 'FAIL'; end
    fprintf('%-6d %-10s %-10s %-6s %s\n', fr, ...
        sprintf('%g (%g)', gotS, wantS), sprintf('%g (%g)', gotP, wantP), res, what);
end
fprintf('RESULT: %d / %d checks passed\n\n', nPass, size(checks, 1));

%% ── Save, open, export a picture ────────────────────────────────────────
save_system(mdl, slx);
open_system(chartBlk);
png = fullfile(here, 'state_machine_chart.png');
try
    sfprint(chartBlk, 'png', png);
    fprintf('Chart picture saved: %s\n', png);
catch err
    fprintf('Could not export the chart picture automatically (%s).\n', err.message);
end
fprintf('Model saved: %s\n', slx);
end

%% ── Local helpers ───────────────────────────────────────────────────────
function d = addData(ch, name, scope, init)
d = Stateflow.Data(ch);
d.Name     = name;
d.Scope    = scope;
d.DataType = 'double';
d.Props.Array.Size = '1';
if nargin > 3
    d.Props.InitialValue = init;
end
end

function s = addState(parent, label, pos)
s = Stateflow.State(parent);
s.LabelString = label;
s.Position    = pos;
end

function t = addTrans(parent, src, dst, label, order, srcClock, dstClock, labelXY)
t = Stateflow.Transition(parent);
t.Source      = src;
t.Destination = dst;
t.LabelString = label;
if nargin > 4 && ~isempty(order)
    t.ExecutionOrder = order;
end
if nargin > 6
    t.SourceOClock      = srcClock;
    t.DestinationOClock = dstClock;
end
if nargin > 7                       % place the label beside its arrow
    lp = t.LabelPosition;
    lp(1:2) = labelXY;
    t.LabelPosition = lp;
end
end

function addDefault(parent, dst)
% Default transition: a short arrow dropping onto the top of dst, drawn
% from dst's own position so it always sits inside its parent state.
p = dst.Position;                          % [x y w h]
t = Stateflow.Transition(parent);
t.Destination       = dst;
t.DestinationOClock = 0;
xs = p(1) + p(3)/2;
t.SourceEndpoint    = [xs, p(2) - 30];
try                                        % keep the arrow straight and short
    t.Midpoint = [xs, p(2) - 15];          % (otherwise it bends out to the
catch                                      %  chart origin and leaves its parent)
    t.MidPoint = [xs, p(2) - 15];
end
end
