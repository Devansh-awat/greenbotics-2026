% run.m — WRO Future Engineers Robot Simulation (interactive)
% Camera block-steering matches main_v3.py (Greenbotics GitHub).
% The control logic lives in control_step.m — shared with evaluate.m so the
% interactive sim and batch evaluation always behave identically.
%
% Press Run. After a run finishes, click "Randomize Blocks" to run again.
% A report figure (trajectory, speeds, clearances, summary) is saved into
% wro/results/ after every run.
clearvars; close all; clc;
addpath(fileparts(mfilename('fullpath')));

[P, CFG, track_img] = sim_setup();

% ── Initial obstacles (official WRO randomization procedure) ──────────────
obstacles = place_obstacles_random(P.cx, P.cy, 0, 0, P.scale, 0, 0);

% ── Figure ────────────────────────────────────────────────────────────────
fig = figure('Name','WRO Simulation','Color','k','Position',[40 40 1280 720]);
setappdata(fig, 'obstacles',    obstacles);
setappdata(fig, 'do_randomize', false);

ax1 = subplot('Position',[0.01 0.32 0.60 0.66]);
imshow(track_img,'Parent',ax1); hold(ax1,'on');
h_obs = draw_obs_markers(ax1, obstacles);
h_startmark = plot(ax1, P.start_x, P.start_y, 'o', 'MarkerSize',16, ...
     'MarkerEdgeColor',[0 0.9 0.9], 'LineWidth',2.5);
h_stext = text(P.start_x+14, P.start_y, 'START / FINISH', 'Parent',ax1, ...
     'Color',[0 0.75 0.75], 'FontSize',8, 'FontWeight','bold');
h_bar  = draw_barriers(ax1, P);
h_path = plot(ax1, P.start_x, P.start_y, 'c-', 'LineWidth',2);
h_bot  = draw_robot_icon(ax1, P.start_x, P.start_y, P.start_th, P.scale, P.robot_L, P.robot_W);
title(ax1,'Top-Down View','Color','w','FontSize',11);

ax2 = subplot('Position',[0.01 0.02 0.40 0.28]);
cam0  = render_frame(P.start_x, P.start_y, P.start_th, obstacles, ...
                     P.cx, P.cy, P.ri_px, P.ro_px, P.scale, CFG);
h_cam = imshow(cam0,'Parent',ax2); hold(ax2,'on');
roi_top_px = round(CFG.FRAME_HEIGHT * CFG.CROP_TOP_FRAC);
roi_bot_px = round(CFG.FRAME_HEIGHT * (1-CFG.CROP_BOTTOM_FRAC));
plot(ax2,[1 CFG.FRAME_WIDTH],[roi_top_px roi_top_px],'m-','LineWidth',1.5);
plot(ax2,[1 CFG.FRAME_WIDTH],[roi_bot_px roi_bot_px],'m-','LineWidth',1.5);
h_box   = gobjects(0);
h_cross = gobjects(0);
title(ax2,'Simulated Camera (ROI = between magenta lines)','Color','w','FontSize',9);

ax3 = subplot('Position',[0.63 0.32 0.36 0.66]);
axis(ax3,'off');
h_txt = text(ax3,0.04,0.97,'','Units','normalized','VerticalAlignment','top', ...
             'Color','w','FontName','Courier','FontSize',10);

uicontrol(fig, ...
    'Style','pushbutton', 'String','Randomize Blocks', ...
    'Units','normalized', 'Position',[0.64 0.18 0.32 0.08], ...
    'FontSize',13, 'FontWeight','bold', ...
    'BackgroundColor',[0.15 0.55 1.0], 'ForegroundColor',[1 1 1], ...
    'Callback', @(~,~) setappdata(fig, 'do_randomize', true));

run_number = 0;

% ══ OUTER LOOP: one iteration = one full simulation run ═══════════════════
while true
    run_number = run_number + 1;

    % Random driving direction each run, like the real rounds
    if rand < 0.5, P = set_direction(P, 'ccw'); else, P = set_direction(P, 'cw'); end
    set(h_startmark, 'XData', P.start_x, 'YData', P.start_y);
    set(h_stext, 'Position', [P.start_x+14, P.start_y, 0]);
    delete(h_bar);  h_bar = draw_barriers(ax1, P);
    uistack(h_path,'top'); uistack(h_bot.body,'top'); uistack(h_bot.arrow,'top');

    S = init_run_state(P);
    path_x = S.x; path_y = S.y;
    N = P.n_steps;
    log.t=zeros(1,N); log.x=zeros(1,N); log.y=zeros(1,N); log.th=zeros(1,N);
    log.v=zeros(1,N); log.omega=zeros(1,N);
    log.state=zeros(1,N,'uint8'); log.laps=zeros(1,N,'uint8');
    log.d_blk=zeros(1,N); log.d_wall=zeros(1,N);
    last_step = N

    if ishandle(fig)
        set(h_path,'XData',path_x,'YData',path_y);
        update_robot_icon(h_bot, S.x, S.y, S.th, P.scale, P.robot_L, P.robot_W);
    end
    fprintf('\n=== RUN %d START ===\n', run_number);

    % ── Main simulation loop ───────────────────────────────────────────────
    for step = 1:N
        if ~ishandle(fig), break; end

        % Randomize button pressed mid-run → new blocks immediately
        if getappdata(fig, 'do_randomize')
            obstacles = place_obstacles_random(P.cx, P.cy, 0, 0, P.scale, 0, 0);
            setappdata(fig, 'obstacles', obstacles);
            setappdata(fig, 'do_randomize', false);
            delete(h_obs);
            h_obs = draw_obs_markers(ax1, obstacles);
            uistack(h_path,'top'); uistack(h_bot.body,'top'); uistack(h_bot.arrow,'top');
            S.state = 'DRIVING_STRAIGHT';
            S.frames_since_block_lost = 0;
            S.lat_x = 0; S.lat_y = 0;
            fprintf('\n[RANDOMIZE] new blocks placed\n');
        end
        obstacles = getappdata(fig, 'obstacles');

        % One shared control + physics step
        S = control_step(S, obstacles, step, P, CFG);

        % Log
        log.t(step)=step*P.dt;  log.x(step)=S.x;  log.y(step)=S.y;  log.th(step)=S.th;
        log.v(step)=S.v;        log.omega(step)=S.omega;
        log.state(step)=state_num(S.state);       log.laps(step)=S.laps_done;
        log.d_blk(step)=S.d_blk_min;              log.d_wall(step)=min(S.d_in,S.d_out);

        if S.mission_complete
            last_step = step;
            if ishandle(fig)
                path_x(end+1)=S.x; path_y(end+1)=S.y; %#ok<AGROW>
                set(h_path,'XData',path_x,'YData',path_y);
                update_robot_icon(h_bot, S.x, S.y, S.th, P.scale, P.robot_L, P.robot_W);
                set(h_bot.body,'Color',[0.2 1 0.4]); set(h_bot.arrow,'Color',[0.2 1 0.4]);
            end
            break;
        end

        % Display update every 3 steps
        if mod(step,3)==0 && ishandle(fig)
            path_x(end+1)=S.x; path_y(end+1)=S.y; %#ok<AGROW>
            set(h_path,'XData',path_x,'YData',path_y);
            update_robot_icon(h_bot, S.x, S.y, S.th, P.scale, P.robot_L, P.robot_W);
            set(h_cam,'CData',S.cam_frame);

            delete(h_box); delete(h_cross);
            if ~isempty(S.block)
                if strcmp(S.block.color,'red'), bc=[1 0 0]; tx=CFG.RED_TARGET_X;
                else,                           bc=[0 0.8 0]; tx=CFG.GREEN_TARGET_X; end
                h_box   = rectangle('Parent',ax2,'Position',S.block.bbox,'EdgeColor',bc,'LineWidth',3);
                h_cross = plot(ax2, tx, CFG.FRAME_HEIGHT/2, '+', ...
                               'Color',[1 1 0],'MarkerSize',16,'LineWidth',2);
            else
                h_box   = gobjects(0);
                h_cross = gobjects(0);
            end

            sc = state_color(S.state);
            set(h_bot.body,'Color',sc); set(h_bot.arrow,'Color',sc);

            bd='--'; ba='--'; bcx='--';
            if ~isempty(S.block)
                bd=S.block.color; ba=sprintf('%d px2',S.block.area); bcx=sprintf('%.0f px',S.block.cx);
            end
            set(h_txt,'String',sprintf([ ...
                '  RUN %d    t = %.1f / %.0f s\n\n' ...
                '  STATE:\n  %s\n\n'        ...
                '  Yaw       : %6.1f deg\n' ...
                '  Track err : %+.1f px\n'  ...
                '  Omega     : %+.3f r/s\n' ...
                '  Speed     : %.3f m/s\n'  ...
                '  Laps      : %d / 3\n\n'  ...
                '  Block     : %s\n'        ...
                '  Area      : %s\n'        ...
                '  Centroid  : %s\n'        ...
                '  Threshold : %d px2\n\n'  ...
                '  Last avoid: %s\n'        ...
                '  ToF walls : F %.2f  L %.2f  R %.2f m\n' ...
                '  Wall gap  : in %.2f  out %.2f m\n\n' ...
                '  Direction : %s\n'        ...
                '  Parking   : %s  (touches %d)\n' ...
                '  Obstacles : %d red  %d green'], ...
                run_number, (step-1)*P.dt, P.total_time, S.state, ...
                S.current_yaw, S.radial_err, S.omega, S.v, S.laps_done, ...
                bd, ba, bcx, CFG.MIN_BLOCK_AREA_FOR_ACTION, ...
                iif(isempty(S.last_avoid_dir),'--',S.last_avoid_dir), ...
                S.d_front, S.d_left, S.d_right, S.d_in, S.d_out, ...
                upper(P.direction), S.park_result, S.barrier_touches, ...
                sum(strcmp({obstacles.color},'red')), sum(strcmp({obstacles.color},'green'))), ...
                'Color', sc);
            drawnow limitrate;
        end
    end

    if ~ishandle(fig)
        fprintf('\nWindow closed. Simulation ended.\n');
        break;
    end

    % ── Save the run report ────────────────────────────────────────────────
    f = fieldnames(log);
    for i = 1:numel(f), log.(f{i}) = log.(f{i})(1:last_step); end
    log.mission_complete = S.mission_complete;
    log.laps_done        = S.laps_done;
    log.lap_times        = S.lap_times;
    log.n_reverses       = S.n_reverses;
    log.total_time       = last_step * P.dt;
    log.min_block_clear  = min(log.d_blk);
    log.min_wall_clear   = min(log.d_wall);
    log.obstacles        = obstacles;
    log.parked           = S.parked;
    log.park_result      = S.park_result;
    log.barrier_touches  = S.barrier_touches;
    log.direction        = P.direction;
    try
        report_png = save_run_report(log, P, track_img, sprintf('run%d', run_number));
        fprintf('Report saved: %s\n', report_png);
    catch err
        fprintf('Report save failed: %s\n', err.message);
    end

    % ── Run finished → wait for Randomize click to start next run ─────────
    if S.mission_complete
        fprintf('\n=== RUN %d: MISSION COMPLETE (parked: %s) === Click "Randomize Blocks" for a new run.\n', ...
                run_number, S.park_result);
        set(h_txt,'String',sprintf([ ...
            '  RUN %d\n  MISSION COMPLETE!\n\n'   ...
            '  3 laps (%s) + PARKED: %s\n'        ...
            '  barrier touches: %d\n'             ...
            '  total time %.1f s\n\n'             ...
            '  Report saved to results/\n\n'      ...
            '  Click  "Randomize Blocks"\n'       ...
            '  to start a new run.\n\n'           ...
            '  Close the window to stop.'],       ...
            run_number, upper(P.direction), upper(S.park_result), ...
            S.barrier_touches, last_step*P.dt), 'Color',[0.2 1 0.4]);
    else
        fprintf('\n=== RUN %d DONE (time up, %d/3 laps) ===\n', run_number, S.laps_done);
        set(h_txt,'String',sprintf([ ...
            '  RUN %d FINISHED  (%.0f s)\n'   ...
            '  Laps completed: %d / 3\n\n'    ...
            '  Report saved to results/\n\n'  ...
            '  Click  "Randomize Blocks"\n'   ...
            '  to shuffle the blocks and\n'   ...
            '  start a new run.\n\n'          ...
            '  Close the window to stop.'],   ...
            run_number, P.total_time, S.laps_done), 'Color',[0.3 1 0.6]);
    end
    drawnow;

    while ishandle(fig) && ~getappdata(fig,'do_randomize')
        pause(0.1);
    end
    if ~ishandle(fig)
        fprintf('\nWindow closed. Simulation ended.\n');
        break;
    end

    % ── Randomize blocks for the next run (official WRO procedure) ────────
    obstacles = place_obstacles_random(P.cx, P.cy, 0, 0, P.scale, 0, 0);
    setappdata(fig, 'obstacles', obstacles);
    setappdata(fig, 'do_randomize', false);
    delete(h_obs);
    h_obs = draw_obs_markers(ax1, obstacles);
    uistack(h_path,'top'); uistack(h_bot.body,'top'); uistack(h_bot.arrow,'top');
end

% ── Local helpers ─────────────────────────────────────────────────────────
function v = iif(c,a,b); if c, v=a; else, v=b; end; end

function id = state_num(s)
    switch s
        case 'DRIVING_STRAIGHT',  id = 1;
        case 'AVOIDING_BLOCK',    id = 2;
        case 'CLEARING',          id = 3;
        case 'EMERGENCY_REVERSE', id = 4;
        case 'CORNER_TURN',       id = 1;
        otherwise                          % PARK_* phases
            id = 5;
    end
end

function c = state_color(s)
    switch s
        case 'DRIVING_STRAIGHT',  c=[0 0.8 0];
        case 'AVOIDING_BLOCK',    c=[1 0.4 0];
        case 'CLEARING',          c=[1 0.9 0];
        case 'EMERGENCY_REVERSE', c=[1 0.1 0.1];
        case 'CORNER_TURN',       c=[0 0.8 0.9];
        otherwise                          % PARK_* phases
            c=[1 0.2 1];
    end
end

function h = draw_barriers(ax, P)
    h = gobjects(2,1);
    for b = 1:2
        h(b) = rectangle('Parent',ax, 'Position', ...
            [P.bars(b,1), P.bars(b,3), ...
             P.bars(b,2)-P.bars(b,1), P.bars(b,4)-P.bars(b,3)], ...
            'FaceColor',[1 0 1], 'EdgeColor',[0.4 0 0.4], 'LineWidth',1);
    end
end

function handles = draw_obs_markers(ax, obs)
    handles = gobjects(numel(obs),1);
    for k = 1:numel(obs)
        if strcmp(obs(k).color,'red'), oc=[1 0 0]; else, oc=[0 0.8 0]; end
        handles(k) = plot(ax, obs(k).x, obs(k).y, 's', ...
            'MarkerSize',16,'MarkerFaceColor',oc,'MarkerEdgeColor','w','LineWidth',2);
    end
end
