function d = ang_diff(a, b)
% mirrors get_angular_difference() in main.py
d = abs(mod(a - b + 180, 360) - 180);
end