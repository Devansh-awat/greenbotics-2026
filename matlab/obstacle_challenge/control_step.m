function S = control_step(S, obstacles, step, P, CFG)
% control_step  One 30 Hz control + physics step of the WRO robot sim.
%   Shared by run.m (interactive) and simulate_run.m (headless batch),
%   so the two can never drift apart.
%
%   Two navigation modes (P.nav_mode):
%     'gyro'  — the REAL robot logic: gyro heading lock + ToF corner turns
%               + main_v3.py block steering  (see control_step_gyro.m)
%     'field' — vector-field navigation (simulation baseline, this file)

if strcmp(P.nav_mode, 'gyro')
    S = control_step_gyro(S, obstacles, step, P, CFG);
    return;
end

% ── 1. Camera + block detection at 10 Hz ──────────────────────────────────
if mod(step,3)==1 || isempty(S.cam_frame)
    S.cam_frame = render_frame(S.x, S.y, S.th, [obstacles, P.barrier_obs], ...
                               P.cx, P.cy, P.ri_px, P.ro_px, P.scale, CFG);
    S.block     = find_biggest_block(S.cam_frame, CFG);
end

% ── 2. Simulated gyro (mirrors main_v3.py) ────────────────────────────────
S.current_yaw = mod(90 - rad2deg(S.th), 360);

% ── 3. Vector field — one desired direction from all influences ───────────
% (a) Flow along the track (tangent around the ring, sign per direction)
dist_c       = sqrt((S.x-P.cx)^2 + (S.y-P.cy)^2);
S.radial_err = dist_c - P.rm_px;
ang_centre   = atan2(P.cy-S.y, P.cx-S.x);
tangent_ang  = ang_centre - P.dir_sign*pi/2;
fx = cos(tangent_ang);
fy = sin(tangent_ang);

% (b) Pull toward the lane centreline
ux  = (S.x-P.cx)/max(dist_c,1);
uy  = (S.y-P.cy)/max(dist_c,1);
k_r = 0.9 * max(-1, min(1, S.radial_err/(P.rm_px*0.35)));
fx  = fx - k_r*ux;
fy  = fy - k_r*uy;

% (c) Wall repulsion — perpendicular distance to both square walls
d0_rep = 0.22;
dxc = S.x - P.cx;  dyc = S.y - P.cy;
qx  = min(max(dxc,-P.half_in_w), P.half_in_w);
qy  = min(max(dyc,-P.half_in_w), P.half_in_w);
vix = dxc - qx;  viy = dyc - qy;
din_px = sqrt(vix^2 + viy^2);
S.d_in = din_px * P.scale;
if S.d_in < d0_rep && din_px > 0
    w  = (d0_rep - S.d_in)/d0_rep;
    fx = fx + 2.2*w*(vix/din_px);
    fy = fy + 2.2*w*(viy/din_px);
end
S.d_out = (P.half_out_w - max(abs(dxc),abs(dyc))) * P.scale;
if S.d_out < d0_rep
    if abs(dxc) > abs(dyc), nxo = -sign(dxc); nyo = 0;
    else,                   nxo = 0;          nyo = -sign(dyc); end
    w  = (d0_rep - S.d_out)/d0_rep;
    fx = fx + 2.2*w*nxo;
    fy = fy + 2.2*w*nyo;
end

% (d) Block avoidance — pull toward the PASS POINT beside each block
%     (right of red, left of green, relative to the LANE FLOW at the block)
d0_blk = 0.45;
S.d_blk_min = inf;
for k = 1:numel(obstacles)
    bxr = obstacles(k).x - S.x;
    byr = obstacles(k).y - S.y;
    dpx = sqrt(bxr^2 + byr^2);
    dm  = dpx * P.scale;
    S.d_blk_min = min(S.d_blk_min, dm);
    if dm < d0_blk && dpx > 0
        w = (d0_blk - dm)/d0_blk;
        phi_b = atan2(P.cy-obstacles(k).y, P.cx-obstacles(k).x) - P.dir_sign*pi/2;
        if strcmp(obstacles(k).color,'red'), side = +1; else, side = -1; end
        off_ang = phi_b + side*pi/2;
        if (bxr*cos(phi_b) + byr*sin(phi_b)) > -0.05/P.scale
            px_t = obstacles(k).x + (0.30/P.scale)*cos(off_ang);
            py_t = obstacles(k).y + (0.30/P.scale)*sin(off_ang);
            dx_t = px_t - S.x;  dy_t = py_t - S.y;
            dn   = sqrt(dx_t^2 + dy_t^2);
            if dn > 1
                fx = fx + 2.0*w*(dx_t/dn);
                fy = fy + 2.0*w*(dy_t/dn);
            end
        end
        if dm < 0.22
            w2 = (0.22 - dm)/0.22;
            fx = fx - 1.5*w2*(bxr/dpx);
            fy = fy - 1.5*w2*(byr/dpx);
        end
    end
end

% ── 4a0. Stuck watchdog: barely moved in the last 3 s → back off ──────────
% (disabled while parking: deliberate slow moves would false-trigger it)
if mod(step,90)==0 && ~S.parking_started
    if sqrt((S.x-S.stuck_x)^2 + (S.y-S.stuck_y)^2)*P.scale < 0.06 && ...
       ~strcmp(S.state,'EMERGENCY_REVERSE')
        dbest = inf; kbest = 0;
        for k = 1:numel(obstacles)
            dk = sqrt((obstacles(k).x-S.x)^2 + (obstacles(k).y-S.y)^2);
            if dk < dbest, dbest = dk; kbest = k; end
        end
        if kbest > 0 && dbest*P.scale < 0.5
            if strcmp(obstacles(kbest).color,'red')
                S.emerg_omega =  P.max_omega * 0.7;
            else
                S.emerg_omega = -P.max_omega * 0.7;
            end
        else
            S.emerg_omega = P.max_omega * 0.7 * (2*(rand>0.5)-1);
        end
        S.emergency_frames = 0;
        S.state      = 'EMERGENCY_REVERSE';
        S.n_reverses = S.n_reverses + 1;
        if P.verbose, fprintf('\n[WATCHDOG] Robot stuck -> EMERGENCY_REVERSE\n'); end
    end
    S.stuck_x = S.x;  S.stuck_y = S.y;
end

% ── 4a. Emergency close-block check — LAST RESORT ─────────────────────────
S.emerg_cooldown = max(0, S.emerg_cooldown - 1);
if ~strcmp(S.state, 'EMERGENCY_REVERSE') && S.emerg_cooldown <= 0 && ~S.parking_started
    d_min = inf; k_near = 0;
    for k = 1:numel(obstacles)
        dxo = obstacles(k).x - S.x;
        dyo = obstacles(k).y - S.y;
        dob = sqrt(dxo^2 + dyo^2);
        ang_rel_ob = atan2(sin(atan2(dyo,dxo)-S.th), cos(atan2(dyo,dxo)-S.th));
        if abs(ang_rel_ob) < deg2rad(50) && dob < d_min
            d_min = dob; k_near = k;
        end
    end
    if d_min < 0.19/P.scale
        if strcmp(obstacles(k_near).color,'red')
            S.emerg_omega =  P.max_omega * 0.7;
        else
            S.emerg_omega = -P.max_omega * 0.7;
        end
        S.emergency_frames = 0;
        S.state      = 'EMERGENCY_REVERSE';
        S.n_reverses = S.n_reverses + 1;
        if P.verbose
            fprintf('\n[STATE] ==> EMERGENCY_REVERSE (%s block %.2f m ahead)\n', ...
                    obstacles(k_near).color, d_min*P.scale);
        end
    end
end

% ── 4b. State machine — camera steering matches main_v3.py ────────────────
S.servo_deg = 0;
switch S.state

    case 'DRIVING_STRAIGHT'
        S.lat_x = 0;  S.lat_y = 0;
        if ~isempty(S.block) && S.block.area > CFG.MIN_BLOCK_AREA_FOR_ACTION
            S.state = 'AVOIDING_BLOCK';
            if P.verbose
                fprintf('\n[STATE] ==> %s  [%s  area=%.0f  cx=%.0f]\n', ...
                        S.state, S.block.color, S.block.area, S.block.cx);
            end
        end

    case 'AVOIDING_BLOCK'
        if ~isempty(S.block)
            if strcmp(S.block.color,'red')
                target_x         = CFG.RED_TARGET_X;    % 20 → pass right of red
                side             = +1;
                S.last_avoid_dir = 'RIGHT past red';
            else
                target_x         = CFG.GREEN_TARGET_X;  % 620 → pass left of green
                side             = -1;
                S.last_avoid_dir = 'LEFT past green';
            end
            S.servo_deg = max(-45, min(45, CFG.KP_BLOCK * (S.block.cx - target_x)));
            prox = min(1, S.block.area / 12000);
            mag  = (abs(S.servo_deg)/45) * (0.9 + 1.5*prox);
            lat_ang = tangent_ang + side*pi/2;   % lane-relative (no orbiting)
            S.lat_x = mag * cos(lat_ang);
            S.lat_y = mag * sin(lat_ang);
        else
            S.state = 'CLEARING';
            S.frames_since_block_lost = 0;
            if P.verbose, fprintf('\n[STATE] ==> %s\n', S.state); end
        end

    case 'CLEARING'
        S.lat_x = S.lat_x * 0.94;
        S.lat_y = S.lat_y * 0.94;
        if isempty(S.block)
            S.frames_since_block_lost = S.frames_since_block_lost + 1;
        else
            S.state = 'AVOIDING_BLOCK';
            if P.verbose, fprintf('\n[STATE] ==> %s (block back)\n', S.state); end
        end
        if S.frames_since_block_lost > CFG.CLEARANCE_FRAMES
            S.state = 'DRIVING_STRAIGHT';
            S.frames_since_block_lost = 0;
            if P.verbose, fprintf('\n[STATE] ==> %s\n', S.state); end
        end

    case 'EMERGENCY_REVERSE'
        S.emergency_frames = S.emergency_frames + 1;
        d_back = ray_wall_distance(S.x, S.y, S.th+pi, ...
                 P.cx, P.cy, P.half_in_w, P.half_out_w) * P.scale;
        if S.emergency_frames > 24 || d_back < 0.22
            S.state          = 'DRIVING_STRAIGHT';
            S.emerg_cooldown = 45;
            S.lat_x = 0;  S.lat_y = 0;
            if P.verbose, fprintf('\n[STATE] ==> %s (backed off)\n', S.state); end
        end
end

fx = fx + S.lat_x;
fy = fy + S.lat_y;

% ── 4c. Simulated ToF rays (braking + telemetry) ──────────────────────────
S.d_front = ray_wall_distance(S.x,S.y,S.th,      P.cx,P.cy,P.half_in_w,P.half_out_w) * P.scale;
S.d_left  = ray_wall_distance(S.x,S.y,S.th-0.61, P.cx,P.cy,P.half_in_w,P.half_out_w) * P.scale;
S.d_right = ray_wall_distance(S.x,S.y,S.th+0.61, P.cx,P.cy,P.half_in_w,P.half_out_w) * P.scale;

% ── 4d. Desired direction → steering + speed ──────────────────────────────
if S.parking_started && ~S.mission_complete
    S = parking_control(S, P);
elseif strcmp(S.state, 'EMERGENCY_REVERSE')
    S.omega = S.emerg_omega;
    S.v     = -0.14;
else
    des_ang = atan2(fy, fx);
    err_des = atan2(sin(des_ang - S.th), cos(des_ang - S.th));
    S.omega = max(-P.max_omega, min(P.max_omega, 3.0 * err_des));
    align       = max(0, cos(err_des));
    front_brake = max(0.40, min(1, S.d_front/0.40));
    S.v = P.nominal_v * (0.35 + 0.65*align) * front_brake;
end

% ── 5. Move robot ──────────────────────────────────────────────────────────
S.th = atan2(sin(S.th + S.omega*P.dt), cos(S.th + S.omega*P.dt));
S.x  = S.x + (S.v/P.scale)*cos(S.th)*P.dt;
S.y  = S.y + (S.v/P.scale)*sin(S.th)*P.dt;

% ── 5b. Solid walls (final guarantee) ─────────────────────────────────────
% Parking needs to hug the outer wall, so the cruise margin shrinks to the
% robot's physical half-width during the maneuver.
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

% ── 5c. Solid blocks ───────────────────────────────────────────────────────
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

% ── 5c2. Solid parking barriers (touches counted per rule 9.24.7) ─────────
[S.x, S.y, touched] = barrier_pushout(S.x, S.y, P, 0.078);
if touched, S.barrier_touches = S.barrier_touches + 1; end

% ── 5d. Lap counting + finish at the start section (WRO rule 9.22) ────────
ang_now    = atan2(S.y-P.cy, S.x-P.cx);
d_ang      = atan2(sin(ang_now-S.prev_ang), cos(ang_now-S.prev_ang));
S.cum_ang  = S.cum_ang + d_ang;
S.prev_ang = ang_now;
new_laps   = floor(abs(S.cum_ang)/(2*pi));
if new_laps > S.laps_done
    S.lap_times(end+1) = step * P.dt; %#ok<AGROW>
    if P.verbose, fprintf('\n[LAP] Completed lap %d at t=%.1f s\n', new_laps, step*P.dt); end
end
S.laps_done = new_laps;
% After 3 laps: find the parking lot and parallel park (rules §5)
if S.laps_done >= 3 && ~S.parking_started
    S.parking_started = true;
    S.state = 'PARK_APPROACH';
    S.lat_x = 0;  S.lat_y = 0;
    if P.verbose
        fprintf('\n[PARK] 3 laps done (t = %.1f s) -> heading to parking lot\n', step*P.dt);
    end
end
end
