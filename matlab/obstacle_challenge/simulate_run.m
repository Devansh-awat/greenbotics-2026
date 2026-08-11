function log = simulate_run(obstacles, P, CFG)
% simulate_run  One full headless run (no graphics). Returns a log struct
%   with the trajectory, states, speeds, clearances and result summary.
%   Used by evaluate.m for batch testing over many randomized layouts.

S = init_run_state(P);
N = P.n_steps;

log.t      = zeros(1,N);
log.x      = zeros(1,N);
log.y      = zeros(1,N);
log.th     = zeros(1,N);
log.v      = zeros(1,N);
log.omega  = zeros(1,N);
log.state  = zeros(1,N,'uint8');
log.laps   = zeros(1,N,'uint8');
log.d_blk  = zeros(1,N);
log.d_wall = zeros(1,N);

last = N;
for step = 1:N
    S = control_step(S, obstacles, step, P, CFG);

    log.t(step)      = step * P.dt;
    log.x(step)      = S.x;
    log.y(step)      = S.y;
    log.th(step)     = S.th;
    log.v(step)      = S.v;
    log.omega(step)  = S.omega;
    log.state(step)  = state_num(S.state);
    log.laps(step)   = S.laps_done;
    log.d_blk(step)  = S.d_blk_min;
    log.d_wall(step) = min(S.d_in, S.d_out);

    if S.mission_complete
        last = step;
        break;
    end
end

% Trim to actual length
f = fieldnames(log);
for i = 1:numel(f)
    log.(f{i}) = log.(f{i})(1:last);
end

log.mission_complete = S.mission_complete;
log.laps_done        = S.laps_done;
log.lap_times        = S.lap_times;
log.n_reverses       = S.n_reverses;
log.total_time       = last * P.dt;
log.min_block_clear  = min(log.d_blk);
log.min_wall_clear   = min(log.d_wall);
log.obstacles        = obstacles;
log.parked           = S.parked;
log.park_result      = S.park_result;
log.barrier_touches  = S.barrier_touches;
log.direction        = P.direction;
end

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
