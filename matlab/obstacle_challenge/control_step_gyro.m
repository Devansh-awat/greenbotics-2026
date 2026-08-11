function S = control_step_gyro(S, obstacles, step, P, CFG)
% control_step_gyro  One 30 Hz step using the REAL robot's navigation logic
%   (mirrors src/obstacle_challenge/nationals/main.py):
%     - straight segments held with gyro heading lock (gyro_steer, KP=3)
%     - corner turn triggered by front ToF distance threshold
%     - target heading changes by -90 deg (yaw) per corner, CCW driving
%     - ToF wall centering on straights
%     - block avoidance = main_v3.py centroid servo law
%     - laps counted by corner turns (4 turns = 1 lap), like the real robot
%   Sim safety nets kept: solid walls/blocks, emergency reverse, watchdog.

% ── 1. Camera + block detection at 10 Hz ──────────────────────────────────
if mod(step,3)==1 || isempty(S.cam_frame)
    S.cam_frame = render_frame(S.x, S.y, S.th, [obstacles, P.barrier_obs], ...
                               P.cx, P.cy, P.ri_px, P.ro_px, P.scale, CFG);
    S.block     = find_biggest_block(S.cam_frame, CFG);
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
if mod(step,90)==0
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

% ── 4a. Close-block emergency (mirrors main_v3.py close-block maneuver) ───
S.emerg_cooldown = max(0, S.emerg_cooldown - 1);
if ~strcmp(S.state,'EMERGENCY_REVERSE') && S.emerg_cooldown <= 0 && ...
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

% Wall-centering correction from perpendicular ToF (nationals wall logic).
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
corner_now    = S.d_front < P.corner_dist && S.corner_cooldown == 0 && ...
                yaw_err_goal < 45 && in_corner_sq;

switch S.state

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
        elseif ~isempty(S.block) && S.block.area > CFG.MIN_BLOCK_AREA_FOR_ACTION
            S.state = 'AVOIDING_BLOCK';
            if P.verbose
                fprintf('\n[STATE] ==> AVOIDING_BLOCK [%s area=%.0f cx=%.0f]\n', ...
                        S.block.color, S.block.area, S.block.cx);
            end
        else
            % Gentle heading hold + wall centering. The real robot drives
            % with gyro KP near zero (main_v3.py switches 0.0 driving /
            % 3.0 parking) and lets wall distances do the centering — a
            % hard gyro pull here would fight obstacle avoidance.
            S.servo_deg = gyro_steer(S.goal_yaw, S.current_yaw, 1.5);
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
        elseif ~isempty(S.block)
            % Avoidance planner: steer toward the PASS POINT 0.30 m beside
            % the pillar ahead (right of red, left of green, relative to
            % the lane direction). This world-frame geometry is the same
            % planner validated in field mode (10/10 layouts) and stands
            % in for the real robot's close-ROI maneuvers, which are not
            % fully modelled here. The camera still decides WHEN to avoid
            % and WHICH colour rule applies (main_v3.py thresholds).
            if strcmp(S.block.color,'red')
                S.last_avoid_dir = 'RIGHT past red';
            else
                S.last_avoid_dir = 'LEFT past green';
            end
            % nearest pillar of the detected colour in the front half
            k_pass = 0; d_pass = inf;
            for k = 1:numel(obstacles)
                if ~strcmp(obstacles(k).color, S.block.color), continue; end
                dxo = obstacles(k).x - S.x;
                dyo = obstacles(k).y - S.y;
                dob = sqrt(dxo^2 + dyo^2);
                br  = atan2(sin(atan2(dyo,dxo)-S.th), cos(atan2(dyo,dxo)-S.th));
                if abs(br) < deg2rad(75) && dob < d_pass
                    d_pass = dob; k_pass = k;
                end
            end
            if k_pass > 0
                ob    = obstacles(k_pass);
                phi_b = atan2(P.cy-ob.y, P.cx-ob.x) - P.dir_sign*pi/2;   % lane flow at pillar
                if strcmp(ob.color,'red'), side = +1; else, side = -1; end
                off_ang = phi_b + side*pi/2;
                px_t = ob.x + (0.30/P.scale)*cos(off_ang);
                py_t = ob.y + (0.30/P.scale)*sin(off_ang);
                des_ang = atan2(py_t - S.y, px_t - S.x);
                err_des = atan2(sin(des_ang - S.th), cos(des_ang - S.th));
                omega_des   = max(-P.max_omega, min(P.max_omega, 3.0*err_des));
                S.servo_deg = -omega_des / P.max_omega * 45;
            else
                S.servo_deg = 0.4*centering;   % pillar already behind
            end
            S.frames_since_block_lost = 0;
        else
            % small hysteresis so one missed detection doesn't flap states
            S.frames_since_block_lost = S.frames_since_block_lost + 1;
            S.servo_deg = 0.4*centering;
            if S.frames_since_block_lost > 5
                % CLEARING (real robot: CLEARANCE_FRAMES): hold the current
                % heading past the block before the gyro re-engages —
                % re-engaging immediately steers back into the block.
                S.state = 'CLEARING';
                S.frames_since_block_lost = 0;
                if P.verbose, fprintf('\n[STATE] ==> CLEARING\n'); end
            end
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
        elseif ~isempty(S.block) && S.block.area > CFG.MIN_BLOCK_AREA_FOR_ACTION
            S.state = 'AVOIDING_BLOCK';
            if P.verbose, fprintf('\n[STATE] ==> AVOIDING_BLOCK (block back)\n'); end
        elseif S.frames_since_block_lost > 45     % ~1.5 s glide past
            S.state = 'DRIVING_STRAIGHT';
            if P.verbose, fprintf('\n[STATE] ==> DRIVING_STRAIGHT (cleared)\n'); end
        end

    case 'CORNER_TURN'
        err = mod(S.goal_yaw - S.current_yaw + 180, 360) - 180;
        S.servo_deg = max(-45, min(45, CFG.GYRO_KP * err));
        if abs(err) < 7
            S.state = 'DRIVING_STRAIGHT';
            % 4 corners = 1 lap (exactly how the real robot counts laps)
            if mod(S.turns_done,4)==0
                S.laps_done          = S.turns_done/4;
                S.lap_times(end+1)   = step * P.dt; %#ok<AGROW>
                if P.verbose, fprintf('\n[LAP] %d complete (t=%.1f s)\n', S.laps_done, step*P.dt); end
            end
            if P.verbose, fprintf('\n[STATE] ==> DRIVING_STRAIGHT (turn done)\n'); end
        end

    case 'EMERGENCY_REVERSE'
        S.emergency_frames = S.emergency_frames + 1;
        d_back = ray_wall_distance(S.x,S.y,S.th+pi, ...
                 P.cx,P.cy,P.half_in_w,P.half_out_w) * P.scale;
        if S.emergency_frames > 30 || d_back < 0.22
            S.state          = 'DRIVING_STRAIGHT';
            S.emerg_cooldown = 30;
            if P.verbose, fprintf('\n[STATE] ==> DRIVING_STRAIGHT (backed off)\n'); end
        end
end

% ── 4c. Servo angle → turn rate; speed policy ──────────────────────────────
if strcmp(S.state,'EMERGENCY_REVERSE')
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
wall_margin = 0.16;
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

% ── 5d. Mission complete: 12 turns (3 laps) + back in the start section ────
in_start_sec = abs(S.x-P.cx)*P.scale < 0.5 && ...
               (S.y-P.cy)*P.scale > 0.5 && (S.y-P.cy)*P.scale < 1.5;
if S.turns_done >= 12 && in_start_sec
    S.mission_complete = true;
    if P.verbose
        fprintf('\n=== MISSION COMPLETE (gyro mode): 12 turns + start section (t=%.1f s) ===\n', step*P.dt);
    end
end
end
