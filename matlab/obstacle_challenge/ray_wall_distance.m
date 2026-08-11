function d = ray_wall_distance(px, py, ang, cx, cy, half_in, half_out)
% Distance (px) from (px,py) along direction ang to the nearest wall:
% the outer 3x3 m square border (from inside) or the inner 1x1 m box.
    c = cos(ang); s = sin(ang);
    d = inf;
    if c > 1e-9,      d = min(d, (cx + half_out - px)/c);
    elseif c < -1e-9, d = min(d, (cx - half_out - px)/c);
    end
    if s > 1e-9,      d = min(d, (cy + half_out - py)/s);
    elseif s < -1e-9, d = min(d, (cy - half_out - py)/s);
    end
    tmin = -inf; tmax = inf; ok = true;
    if abs(c) < 1e-9
        if abs(px-cx) > half_in, ok = false; end
    else
        t1 = (cx - half_in - px)/c;  t2 = (cx + half_in - px)/c;
        tmin = max(tmin, min(t1,t2)); tmax = min(tmax, max(t1,t2));
    end
    if ok && abs(s) < 1e-9
        if abs(py-cy) > half_in, ok = false; end
    elseif ok
        t1 = (cy - half_in - py)/s;  t2 = (cy + half_in - py)/s;
        tmin = max(tmin, min(t1,t2)); tmax = min(tmax, max(t1,t2));
    end
    if ok && tmax >= tmin && tmax > 0
        if tmin > 0, d = min(d, tmin); end
    end
end
