function S = parking_control(S, P)
% parking_control  Parking maneuver after 3 laps (rules §5):
%   PARK_APPROACH    drive to the staging point beside the lot
%   PARK_ALIGN       stop centred on the lot, still out in the lane
%   PARK_TURN_IN     rotate to face the lot opening
%   PARK_ENTER       creep into the gap between the magenta barriers
%   PARK_STRAIGHTEN  rotate parallel to the outer wall
%   Verdict: 'full' (15 pts: fully inside + parallel + zero barrier
%   touches) or 'partial' (7 pts), stored in S.park_result.
%
%   Note: the sim's unicycle model can rotate on the spot, so the entry is
%   a turn-in rather than the Ackermann S-curve the physical car performs;
%   the lot geometry, clearances and scoring checks are to rule.

wall_th = pi/2;   % facing the outer wall of the start section (+y)

switch S.state

    case 'PARK_APPROACH'
        S = pursue(S, P, P.stage_pt, 0.12);
        if dist_to(S, P.stage_pt) < 0.07/P.scale
            S.state = 'PARK_ALIGN';
            if P.verbose, fprintf('\n[PARK] staging reached -> ALIGN\n'); end
        end

    case 'PARK_ALIGN'
        S = pursue(S, P, P.align_pt, 0.09);
        if dist_to(S, P.align_pt) < 0.035/P.scale
            S.state = 'PARK_TURN_IN';
            if P.verbose, fprintf('\n[PARK] centred on lot -> TURN IN\n'); end
        end

    case 'PARK_TURN_IN'
        err = atan2(sin(wall_th - S.th), cos(wall_th - S.th));
        S.omega = max(-P.max_omega, min(P.max_omega, 3.0*err));
        S.v     = 0;
        if abs(rad2deg(err)) < 6
            S.state = 'PARK_ENTER';
            if P.verbose, fprintf('\n[PARK] facing lot -> ENTER\n'); end
        end

    case 'PARK_ENTER'
        S = pursue(S, P, P.enter_pt, 0.06);
        if (S.y - P.cy)*P.scale > 1.37
            S.state      = 'PARK_STRAIGHTEN';
            S.park_timer = 0;
            if P.verbose, fprintf('\n[PARK] inside lot -> STRAIGHTEN\n'); end
        end

    case 'PARK_STRAIGHTEN'
        S.park_timer = S.park_timer + 1;
        err_th  = atan2(sin(P.lane_th - S.th), cos(P.lane_th - S.th));
        S.omega = max(-P.max_omega, min(P.max_omega, 3.0*err_th));
        S.v     = 0;

        cx_ok  = abs(S.x-P.lot_x)*P.scale < 0.045;   % fully between barriers
        dep_ok = (S.y-P.cy)*P.scale > 1.35;          % fully inside lot depth
        ang_ok = abs(rad2deg(err_th)) < 8;           % parallel to the wall
        if (cx_ok && dep_ok && ang_ok) || S.park_timer > 150
            if cx_ok && dep_ok && ang_ok && S.barrier_touches == 0
                S.park_result = 'full';
            else
                S.park_result = 'partial';
            end
            S.parked           = true;
            S.mission_complete = true;
            S.v = 0;  S.omega = 0;
            if P.verbose
                fprintf('\n=== PARKED (%s): x-off %.0f mm, depth %.2f m, angle %.1f deg, touches %d ===\n', ...
                    S.park_result, abs(S.x-P.lot_x)*P.scale*1000, ...
                    (S.y-P.cy)*P.scale, abs(rad2deg(err_th)), S.barrier_touches);
            end
        end
end
end

function S = pursue(S, P, pt, spd)
    des = atan2(pt(2)-S.y, pt(1)-S.x);
    err = atan2(sin(des-S.th), cos(des-S.th));
    S.omega = max(-P.max_omega, min(P.max_omega, 3.0*err));
    S.v     = spd * max(0.3, cos(err));
end

function d = dist_to(S, pt)
    d = sqrt((S.x-pt(1))^2 + (S.y-pt(2))^2);
end
