function update_robot_icon(h, rx, ry, rtheta, scale, L, W)
lp = L/scale;   wp = W/scale;
C  = [lp/2 wp/2; -lp/2 wp/2; -lp/2 -wp/2; lp/2 -wp/2; lp/2 wp/2]';
R  = [cos(rtheta) -sin(rtheta); sin(rtheta) cos(rtheta)];
rc = R * C;
set(h.body,  'XData', rx+rc(1,:), 'YData', ry+rc(2,:));
set(h.dot,   'XData', rx,          'YData', ry);
set(h.arrow, 'XData', [rx, rx+lp*cos(rtheta)], ...
    'YData', [ry, ry+lp*sin(rtheta)]);
end