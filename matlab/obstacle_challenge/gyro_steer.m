function s = gyro_steer(goal, yaw, kp)
% mirrors steer_with_gyro() in main.py
err = goal - yaw;
err = mod(err + 180, 360) - 180;
s   = max(-45, min(45, kp * err));
end