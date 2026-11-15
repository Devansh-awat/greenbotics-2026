function S = control_step_gyro(S, obstacles, step, P, CFG)
% control_step_gyro  One 30 Hz step modelling the REAL robot's navigation
%   ARCHITECTURE (src/obstacle_challenge/{control,rules,tuning,main}.py as of
%   this sync). What is and isn't a literal port, stated plainly:
%
%   LITERAL PORT:
%     - straight segments held with the same gyro PD law and gains as
%       control.py's steer_with_gyro()/_heading_hold_step(): GYRO_KP=0.85,
%       GYRO_KD=0.1 (CFG.GYRO_KP/KD, threaded via gyro_steer.m)
%     - target heading changes by -90 deg (yaw) per corner, CCW driving
%     - laps counted every 4 corner turns, matching TURNS_TO_FINISH=13 in
%       tuning.py (12 turns = 3 laps, +1 to square up before parking)
%
%   SIMPLIFIED, NOT A LITERAL PORT (see matlab/README.md for why each one
%   isn't meaningfully reproducible in this top-down physics model):
%     - corner trigger: front-wall proximity + corner-square position, a
%       geometric PROXY for the real orange-line crossing trigger
%       (rules.count_turns) — floor lines are not rendered here. How deep
%       into the corner to go before turning depends on the first pillar of
%       the next straight: deeper if it must be passed on the outside,
%       earlier if on the inside (turn_depth)
%     - block avoidance: the camera decides WHEN to avoid; WHICH pillar and
%       HOW use the intent of rules.py, in top-down geometry:
%         * only pillars on the CURRENT straight are targets; a pillar seen
%           over the corner belongs to the next straight and is ignored
%           (rules._is_next_straight_pillar)
%         * the nearest unpassed pillar is the target, and the target hands
%           over to the next one once passed (rules.pick_target_block)
%         * red passed on its right, green on its left (rule_red/rule_green),
%           by steering for the middle of the gap between the pillar and
%           the wall on that side (rules._gap_to_wall), kept clear of the
%           walls and the parking barriers (gap_lateral, pass_aim)
%       The real rules work in camera-image coordinates (handoff lines,
%       wall areas, magenta) that have no top-down equivalent here.
%     - a pillar within 0.6 m ahead on the current straight is acted on
%       even if the main camera ROI misses it (the real robot's close-block
%       ROI, rules.rule_close_block, covers that range)
%     - laps for scoring are counted from the robot's real position, as in
%       'field' mode; the 12-turn count only decides when to park
%     - parking after 12 turns uses the same parking_control.m as 'field'
%       mode (the real robot's parking is a scripted routine in maneuvers.py)
%     - corner-turn snap uses its own CFG.CORNER_SNAP_KP (the real robot
%       executes the turn as a separate blocking routine in maneuvers.py,
%       not this PD loop, so there is no real constant to mirror there)
%   Sim safety nets kept: solid walls/blocks, emergency reverse, watchdog.

% ── 1. Camera + block detection at 10 Hz ──────────────────────────────────
if mod(step,3)==1 || isempty(S.cam_frame)
    S.cam_frame  = render_frame(S.x, S.y, S.th, [obstacles, P.barrier_obs], ...
                               P.cx, P.cy, P.ri_px, P.ro_px, P.scale, CFG);
    S.block      = find_biggest_block(S.cam_frame, CFG);
    S.wall_black = find_wall_black(S.cam_frame, CFG);   % detection only, unused below
end

% ── 2. Simulated gyro (BNO055): yaw = mod(90 - theta_deg, 360) ────────────
S.current_yaw = mod(90 - rad2deg(S.th), 360);
dist_c        = sqrt((S.x-P.cx)^2 + (S.y-P.cy)^2);
S.radial_err  = dist_c - P.rm_px;

% ── 3. Simulated ToF rays ──────────────────────────────────────────────────
S.d_front = ray_wall_distance(S.x,S.y,S.th,        P.cx,P.cy,P.half_in_w,P.half_out_w) * P.scale;
S.d_left  = ray_wall_distance(S.x,S.y,S.th-pi/2,   P.cx,P.cy,P.half_in_w,P.half_out_w) * P.scale;
S.d_right = ray_wall_distance(S.x,S.y,S.th+pi/2,   P.cx,P.cy,P.half_in_w,P.half_out_w) * P.scale;

% Wall gaps for telemetry/logging (perpendicular distances)
dxc = S.x - P.cx;  dyc = S.y - P.cy;
qx  = min(max(dxc,-P.half_in_w), P.half_in_w);
qy  = min(max(dyc,-P.half_in_w), P.half_in_w);
S.d_in  = sqrt((dxc-qx)^2 + (dyc-qy)^2) * P.scale;
S.d_out = (P.half_out_w - max(abs(dxc),abs(dyc))) * P.scale;

% Nearest block distance (telemetry + emergency logic).
% d_blk_ahead: pillar in the narrow front cone — the real VL53 ToF sees
% pillars too, so braking must react to them, not only to walls.
S.d_blk_min = inf; k_near = 0; d_min_front = inf; d_blk_ahead = inf;
for k = 1:numel(obstacles)
    dxo = obstacles(k).x - S.x;
    dyo = obstacles(k).y - S.y;
    dob = sqrt(dxo^2 + dyo^2);
    S.d_blk_min = min(S.d_blk_min, dob*P.scale);
    ang_rel_ob = atan2(sin(atan2(dyo,dxo)-S.th), cos(atan2(dyo,dxo)-S.th));
    % Emergency cone is NARROW: a pillar we are legally passing beside
    % must not count as "block ahead" (that caused reverse loops)
    if abs(ang_rel_ob) < deg2rad(35) && dob < d_min_front
        d_min_front = dob; k_near = k;
    end
    if abs(ang_rel_ob) < deg2rad(15)
        d_blk_ahead = min(d_blk_ahead, dob*P.scale);
    end
end

% ── 4a0. Stuck watchdog (SafetyMonitor twin) ───────────────────────────────
if mod(step,90)==0 && ~S.parking_started
    if sqrt((S.x-S.stuck_x)^2 + (S.y-S.stuck_y)^2)*P.scale < 0.06 && ...
       ~strcmp(S.state,'EMERGENCY_REVERSE')
        if k_near > 0 && d_min_front*P.scale < 0.5 && strcmp(obstacles(k_near).color,'red')
            S.emerg_omega =  P.max_omega * 0.7;
        else
            S.emerg_omega = -P.max_omega * 0.7;
        end
        S.emergency_frames = 0;
        S.state      = 'EMERGENCY_REVERSE';
        S.n_reverses = S.n_reverses + 1;
        if P.verbose, fprintf('\n[WATCHDOG] stuck -> EMERGENCY_REVERSE\n'); end
    end
    S.stuck_x = S.x;  S.stuck_y = S.y;
end

% ── 4a. Close-block emergency (simulation safety net, see rules.rule_close_block
%        for the real robot's equivalent reverse-then-swerve rule) ─────────
S.emerg_cooldown = max(0, S.emerg_cooldown - 1);
if ~strcmp(S.state,'EMERGENCY_REVERSE') && S.emerg_cooldown <= 0 && ~S.parking_started && ...
   k_near > 0 && d_min_front < 0.17/P.scale
    if strcmp(obstacles(k_near).color,'red')
        S.emerg_omega =  P.max_omega * 0.7;
    else
        S.emerg_omega = -P.max_omega * 0.7;
    end
    S.emergency_frames = 0;
    S.state      = 'EMERGENCY_REVERSE';
    S.n_reverses = S.n_reverses + 1;
    if P.verbose
        fprintf('\n[STATE] ==> EMERGENCY_REVERSE (%s block %.2f m)\n', ...
                obstacles(k_near).color, d_min_front*P.scale);
    end
end

% ── 4b. State machine (real robot structure) ───────────────────────────────
S.servo_deg = 0;

% Wall-centering correction from perpendicular ToF (simplified stand-in for
% the real robot's wall-following PD in rules.py / control.py).
% Servo convention in this sim: positive = counter-clockwise on screen
% (matches gyro_steer with yaw = 90 - theta). Right wall closer → steer
% away from it → positive here.
dL = min(S.d_left,  0.9);
dR = min(S.d_right, 0.9);
centering = max(-12, min(12, 20*(dL - dR)));

% Corner trigger needs: front wall close + aligned with the lane + cooldown
% elapsed + robot actually inside a corner square of the mat (prevents
% false triggers when the front ray clips the inner box mid-straight)
S.corner_cooldown = max(0, S.corner_cooldown - 1);
yaw_err_goal  = abs(mod(S.goal_yaw - S.current_yaw + 180, 360) - 180);
in_corner_sq  = abs(dxc)*P.scale > 0.42 && abs(dyc)*P.scale > 0.42;
[s_lane, ~]   = lane_pos(S, P);
[k_tgt, ds_tgt] = section_target(S, obstacles, P);   % see below
% turn only at the end of THIS straight, and only once every pillar on it
% has been passed (a pillar right before the corner is passed first)
% Turn depth: how far into the corner to drive before turning. The depth
% reached on this straight becomes the lateral position on the next one,
% so the robot turns deeper when the next straight's first pillar must be
% passed on the outside, and earlier when it must be passed on the inside.
s_turn        = turn_depth(S, obstacles, P);
corner_now    = S.corner_cooldown == 0 && yaw_err_goal < 45 && in_corner_sq && ...
                k_tgt == 0 && (s_lane >= s_turn || S.d_front < 0.25);

% Target pillar on the CURRENT straight (rules._is_next_straight_pillar +
% pick_target_block): nearest unpassed pillar between the robot and the
% corner ahead; pillars on the next straight are ignored.
cam_sees = ~isempty(S.block) && S.block.area > CFG.MIN_BLOCK_AREA_FOR_ACTION;
% a pillar within 0.6 m ahead on this straight is always acted on, as the
% real robot's close-block ROI sees it even when the main ROI does not
avoid_go = k_tgt > 0 && ((cam_sees && ds_tgt < 1.2) || ds_tgt < 0.6);

% After 12 turns (3 laps): park, same routine as 'field' mode
if S.turns_done >= 12 && ~S.parking_started && strcmp(S.state,'DRIVING_STRAIGHT') ...
        && ~(isfield(P,'parking_enabled') && ~P.parking_enabled)
    S.parking_started = true;
    S.state = 'PARK_APPROACH';
    if P.verbose, fprintf('\n[PARK] 12 turns done (t = %.1f s) -> parking\n', step*P.dt); end
end

if S.parking_started
    state_sw = 'PARKING';
else
    state_sw = S.state;
end

switch state_sw

    case 'PARKING'
        S = parking_control(S, P);

    case 'DRIVING_STRAIGHT'
        % Corner: front wall inside threshold → new heading goal (-90 yaw)
        if corner_now
            S.goal_yaw        = mod(S.goal_yaw - P.dir_sign*90, 360);
            S.turns_done      = S.turns_done + 1;
            S.corner_cooldown = 75;    % 2.5 s — one straight takes ~6 s
            S.state           = 'CORNER_TURN';
            if P.verbose
                fprintf('\n[TURN %d] front %.2f m -> goal yaw %.0f\n', ...
                        S.turns_done, S.d_front, S.goal_yaw);
            end
        elseif avoid_go
            S.state = 'AVOIDING_BLOCK';
            if P.verbose
                fprintf('\n[STATE] ==> AVOIDING_BLOCK [%s pillar %.2f m ahead]\n', ...
                        obstacles(k_tgt).color, ds_tgt);
            end
        else
            % Gyro PD heading hold, same gains and formula as the real
            % robot's steer_with_gyro() (control.py), plus wall centering —
            % the real robot leans on wall distance between corners too.
            [S.servo_deg, S.gyro_prev_err] = gyro_steer(S.goal_yaw, S.current_yaw, ...
                CFG.GYRO_KP, CFG.GYRO_KD, S.gyro_prev_err);
            S.servo_deg = max(-45, min(45, S.servo_deg + centering));
        end

    case 'AVOIDING_BLOCK'
        if corner_now
            % corner takes priority (block will be re-acquired after turn)
            S.goal_yaw        = mod(S.goal_yaw - P.dir_sign*90, 360);
            S.turns_done      = S.turns_done + 1;
            S.corner_cooldown = 75;
            S.state           = 'CORNER_TURN';
            if P.verbose, fprintf('\n[TURN %d] during avoidance\n', S.turns_done); end
        elseif k_tgt > 0
            % Pass the target pillar: red on its right, green on its left
            ob = obstacles(k_tgt);
            if strcmp(ob.color,'red')
                S.last_avoid_dir = 'RIGHT past red';
            else
                S.last_avoid_dir = 'LEFT past green';
            end
            [aim_x, aim_y] = pass_aim(S, ob, P);
            des_ang = atan2(aim_y - S.y, aim_x - S.x);
            err_des = atan2(sin(des_ang - S.th), cos(des_ang - S.th));
            omega_des   = max(-P.max_omega, min(P.max_omega, 3.0*err_des));
            S.servo_deg = -omega_des / P.max_omega * 45;
            S.frames_since_block_lost = 0;
        else
            % every pillar on this straight is passed -> hold heading briefly
            S.state = 'CLEARING';
            S.frames_since_block_lost = 0;
            S.servo_deg = 0.4*centering;
            if P.verbose, fprintf('\n[STATE] ==> CLEARING\n'); end
        end

    case 'CLEARING'
        S.frames_since_block_lost = S.frames_since_block_lost + 1;
        S.servo_deg = 0.5*centering;      % hold heading, keep off walls
        if corner_now
            S.goal_yaw        = mod(S.goal_yaw - P.dir_sign*90, 360);
            S.turns_done      = S.turns_done + 1;
            S.corner_cooldown = 75;
            S.state           = 'CORNER_TURN';
            if P.verbose, fprintf('\n[TURN %d] during clearing\n', S.turns_done); end
        elseif avoid_go
            S.state = 'AVOIDING_BLOCK';
            if P.verbose, fprintf('\n[STATE] ==> AVOIDING_BLOCK (next pillar)\n'); end
        elseif S.frames_since_block_lost > 20     % ~1.5 s glide past
            S.state         = 'DRIVING_STRAIGHT';
            S.gyro_prev_err = 0;   % fresh PD start, no stale derivative kick
            if P.verbose, fprintf('\n[STATE] ==> DRIVING_STRAIGHT (cleared)\n'); end
        end

    case 'CORNER_TURN'
        % Discrete turn snap — own gain (CORNER_SNAP_KP), not the straight-
        % line gyro PD: see header note on why there's no real constant here.
        err = mod(S.goal_yaw - S.current_yaw + 180, 360) - 180;
        S.servo_deg = max(-45, min(45, CFG.CORNER_SNAP_KP * err));
        if abs(err) < 7 || (k_tgt > 0 && ds_tgt < 0.7 && abs(err) < 35)
            % turn done (or a pillar sits just after the corner: start
            % passing it now instead of finishing the turn into it)
            if avoid_go
                S.state = 'AVOIDING_BLOCK';
            else
                S.state = 'DRIVING_STRAIGHT';
            end
            S.gyro_prev_err = 0;
            if P.verbose, fprintf('\n[STATE] ==> %s (turn done)\n', S.state); end
        end

    case 'EMERGENCY_REVERSE'
        S.emergency_frames = S.emergency_frames + 1;
        d_back = ray_wall_distance(S.x,S.y,S.th+pi, ...
                 P.cx,P.cy,P.half_in_w,P.half_out_w) * P.scale;
        if S.emergency_frames > 30 || d_back < 0.22
            S.state          = 'DRIVING_STRAIGHT';
            S.gyro_prev_err  = 0;
            S.emerg_cooldown = 30;
            if P.verbose, fprintf('\n[STATE] ==> DRIVING_STRAIGHT (backed off)\n'); end
        end
end

% ── 4c. Servo angle → turn rate; speed policy ──────────────────────────────
if S.parking_started
    % parking_control.m already set S.v and S.omega
elseif strcmp(S.state,'EMERGENCY_REVERSE')
    S.omega = S.emerg_omega;
    S.v     = -0.14;
else
    S.omega = -(S.servo_deg/45) * P.max_omega;
    if strcmp(S.state,'CORNER_TURN')
        S.v = P.nominal_v * 0.75;
    else
        % Brake for whichever is closer ahead: wall or pillar (ToF sees both)
        d_ahead     = min(S.d_front, d_blk_ahead);
        front_brake = max(0.40, min(1, d_ahead/0.40));
        S.v = P.nominal_v * (1 - 0.15*abs(S.servo_deg)/45) * front_brake;
        % Extra caution while a close block fills the camera view
        if strcmp(S.state,'AVOIDING_BLOCK') && ~isempty(S.block)
            S.v = S.v * (1 - 0.25*min(1, S.block.area/15000));
        end
    end
end

% ── 5. Move robot ───────────────────────────────────────────────────────────
S.th = atan2(sin(S.th + S.omega*P.dt), cos(S.th + S.omega*P.dt));
S.x  = S.x + (S.v/P.scale)*cos(S.th)*P.dt;
S.y  = S.y + (S.v/P.scale)*sin(S.th)*P.dt;

% ── 5b. Solid walls ─────────────────────────────────────────────────────────
if S.parking_started, wall_margin = 0.09; else, wall_margin = 0.16; end
half_out_c = (1.5 - wall_margin)/P.scale;
half_in_c  = (0.5 + wall_margin)/P.scale;
S.x = min(max(S.x, P.cx - half_out_c), P.cx + half_out_c);
S.y = min(max(S.y, P.cy - half_out_c), P.cy + half_out_c);
dxw = S.x - P.cx;  dyw = S.y - P.cy;
if abs(dxw) < half_in_c && abs(dyw) < half_in_c
    sx = sign(dxw); if sx==0, sx=1; end
    sy = sign(dyw); if sy==0, sy=1; end
    if (half_in_c - abs(dxw)) < (half_in_c - abs(dyw))
        S.x = P.cx + sx*half_in_c;
    else
        S.y = P.cy + sy*half_in_c;
    end
end

% ── 5c. Solid blocks ────────────────────────────────────────────────────────
r_col = 0.14/P.scale;
for k = 1:numel(obstacles)
    bxc = S.x - obstacles(k).x;
    byc = S.y - obstacles(k).y;
    ddc = sqrt(bxc^2 + byc^2);
    if ddc < r_col && ddc > 0
        S.x = obstacles(k).x + (bxc/ddc)*r_col;
        S.y = obstacles(k).y + (byc/ddc)*r_col;
    end
end

% ── 5c2. Solid parking barriers ─────────────────────────────────────────────
[S.x, S.y, touched] = barrier_pushout(S.x, S.y, P, 0.078);
if touched, S.barrier_touches = S.barrier_touches + 1; end

% ── 5d0. Laps from the robot's real position (same as 'field' mode) ────────
ang_now    = atan2(S.y-P.cy, S.x-P.cx);
d_ang      = atan2(sin(ang_now-S.prev_ang), cos(ang_now-S.prev_ang));
S.cum_ang  = S.cum_ang + d_ang;
S.prev_ang = ang_now;
new_laps   = floor(abs(S.cum_ang)/(2*pi));
if new_laps > S.laps_done
    S.lap_times(end+1) = step * P.dt; %#ok<AGROW>
    if P.verbose, fprintf('\n[LAP] %d complete (t=%.1f s)\n', new_laps, step*P.dt); end
end
S.laps_done = new_laps;

% ── 5d. Mission complete: 12 turns (3 laps) + back in the start section ────
in_start_sec = abs(S.x-P.cx)*P.scale < 0.5 && ...
               (S.y-P.cy)*P.scale > 0.5 && (S.y-P.cy)*P.scale < 1.5;
if S.turns_done >= 12 && in_start_sec && isfield(P,'parking_enabled') && ~P.parking_enabled
    S.mission_complete = true;
    if P.verbose
        fprintf('\n=== MISSION COMPLETE (gyro mode): 12 turns + start section (t=%.1f s) ===\n', step*P.dt);
    end
end
end

% ── Local helpers ──────────────────────────────────────────────────────────
function [k_t, ds_t] = section_target(S, obstacles, P)
% Nearest unpassed pillar on the current straight, in lane coordinates:
% s = along the current lane heading, l = across it (metres from centre).
th_g = deg2rad(90 - S.goal_yaw);               % lane heading (yaw convention)
h = [cos(th_g), sin(th_g)];  n = [-h(2), h(1)];
r = [S.x - P.cx, S.y - P.cy] * P.scale;
s_r = r*h';  l_r = r*n';
k_t = 0;  ds_t = inf;
for k = 1:numel(obstacles)
    p = [obstacles(k).x - P.cx, obstacles(k).y - P.cy] * P.scale;
    s_p = p*h';  l_p = p*n';
    if abs(l_p) < 0.5 || sign(l_p) ~= sign(l_r), continue; end  % other straight
    if s_p > 0.55, continue; end                % beyond the corner ahead
    ds = s_p - s_r;
    if ds < -0.12, continue; end                % already passed
    if ds < ds_t, ds_t = ds; k_t = k; end
end
end

function [ax, ay] = pass_aim(S, ob, P)
% Aim point on the pass line beside pillar ob: red passed on its right,
% green on its left, through the MIDDLE of the gap between the pillar and
% the wall on that side (rules._gap_to_wall). The aim point is always 0.30 m
% ahead of the robot ON the pass line (pure pursuit), so the robot joins
% the pass line before it reaches the pillar instead of cutting through it.
th_g = deg2rad(90 - S.goal_yaw);
h = [cos(th_g), sin(th_g)];  n = [-h(2), h(1)];   % n = right of travel
r = [S.x - P.cx, S.y - P.cy] * P.scale;
p = [ob.x - P.cx, ob.y - P.cy] * P.scale;
s_r = r*h';  l_p = p*n';
l_t = sign(l_p) * gap_lateral(ob, l_p, p);
s_t = s_r + 0.30;
a = (s_t*h + l_t*n) / P.scale;
ax = P.cx + a(1);  ay = P.cy + a(2);
end

function [s_r, l_r] = lane_pos(S, P)
% Robot position in lane coordinates of the current straight (metres):
% s along the lane heading, l across it.
th_g = deg2rad(90 - S.goal_yaw);
h = [cos(th_g), sin(th_g)];  n = [-h(2), h(1)];
r = [S.x - P.cx, S.y - P.cy] * P.scale;
s_r = r*h';  l_r = r*n';
end

function a_t = gap_lateral(ob, l_p, p)
% Distance from the centre (|l|, metres) of the middle of the gap the robot
% must use to pass pillar ob: red on its right, green on its left, between
% the pillar and the wall on that side (rules._gap_to_wall).
if strcmp(ob.color,'red'), side = +1; else, side = -1; end   % +1 = right
if side == sign(l_p)
    a_t = (abs(l_p) + 1.5) / 2;      % gap between pillar and outer wall
else
    a_t = (abs(l_p) + 0.5) / 2;      % gap between pillar and inner wall
end
% keep the robot's body clear of the walls; in the start section also
% clear of the parking barriers (0.20 m deep against the outer wall)
in_lot_sec = abs(p(1)) < 0.5 && p(2) > 0.5;
if in_lot_sec, cap = 1.22; else, cap = 1.32; end
a_t = min(max(a_t, 0.70), cap);
end

function s_turn = turn_depth(S, obstacles, P)
% How far along the current straight (s, metres from centre) to go before
% turning: lines the robot up with the gap for the first pillar of the NEXT
% straight when that pillar sits close to the corner exit.
s_turn = 0.78;                                  % default: middle of the lane
g2 = mod(S.goal_yaw - P.dir_sign*90, 360);      % heading after this corner
th2 = deg2rad(90 - g2);
h2 = [cos(th2), sin(th2)];  n2 = [-h2(2), h2(1)];
r = [S.x - P.cx, S.y - P.cy] * P.scale;
s2_r = r*h2';  l2_r = r*n2';
best = inf;  k_b = 0;
for k = 1:numel(obstacles)
    p = [obstacles(k).x - P.cx, obstacles(k).y - P.cy] * P.scale;
    s2 = p*h2';  l2 = p*n2';
    if abs(l2) < 0.5 || sign(l2) ~= sign(l2_r), continue; end
    ds = s2 - s2_r;
    if ds > 0 && ds < best, best = ds; k_b = k; end
end
if k_b > 0 && best < 0.9                        % pillar close to the corner exit
    p = [obstacles(k_b).x - P.cx, obstacles(k_b).y - P.cy] * P.scale;
    s_turn = gap_lateral(obstacles(k_b), p*n2', p) - 0.05;
end
end
