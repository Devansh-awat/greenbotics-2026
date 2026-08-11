function S = init_run_state(P)
% init_run_state  Fresh robot + controller state for one run.

S.x  = P.start_x;
S.y  = P.start_y;
S.th = P.start_th;

S.state     = 'DRIVING_STRAIGHT';
S.block     = [];
S.cam_frame = [];

S.frames_since_block_lost = 0;
S.last_avoid_dir          = '';
S.lat_x = 0;  S.lat_y = 0;

S.emerg_omega      = 0;
S.emergency_frames = 0;
S.emerg_cooldown   = 0;

S.stuck_x = S.x;  S.stuck_y = S.y;

S.cum_ang   = 0;
S.prev_ang  = atan2(S.y - P.cy, S.x - P.cx);
S.laps_done = 0;
S.lap_times = [];
S.mission_complete = false;

S.v = 0;  S.omega = 0;  S.servo_deg = 0;
S.current_yaw = 0;  S.radial_err = 0;
S.d_front = inf;  S.d_left = inf;  S.d_right = inf;
S.d_in = inf;  S.d_out = inf;  S.d_blk_min = inf;
S.n_reverses = 0;

% gyro-mode fields (real robot logic)
S.goal_yaw        = mod(90 - rad2deg(P.start_th), 360);
S.turns_done      = 0;
S.corner_cooldown = 0;    % frames until the next corner may trigger

% parking fields
S.parking_started = false;
S.parked          = false;
S.park_result     = 'none';    % 'full' (15 pts) | 'partial' (7 pts) | 'none'
S.park_timer      = 0;
S.barrier_touches = 0;
end
