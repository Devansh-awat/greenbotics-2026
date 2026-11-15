function [s, err] = gyro_steer(goal, yaw, kp, kd, prev_err)
% gyro_steer  Gyro heading-lock PD step.
%   Mirrors control.py's steer_with_gyro() / _heading_hold_step() exactly:
%   wrap the heading error to [-180,180], then s = kp*err + kd*(err-prev_err),
%   clamped to the servo's +-45 deg limit. Pass the returned err back in as
%   prev_err next frame so the derivative term is threaded over time, the
%   way drive_straight_with_gyro()'s loop does it on the real robot.
%
%   kd and prev_err are optional (default 0) for callers that just want the
%   plain P step, e.g. a short discrete corner-turn snap.
if nargin < 4, kd = 0; end
if nargin < 5, prev_err = 0; end

err = goal - yaw;
err = mod(err + 180, 360) - 180;
s   = max(-45, min(45, kp*err + kd*(err - prev_err)));
end
