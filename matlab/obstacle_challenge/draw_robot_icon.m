function h = draw_robot_icon(ax, rx, ry, rtheta, scale, L, W)
lp = L/scale;   wp = W/scale;
C  = [lp/2 wp/2; -lp/2 wp/2; -lp/2 -wp/2; lp/2 -wp/2; lp/2 wp/2]';
R  = [cos(rtheta) -sin(rtheta); sin(rtheta) cos(rtheta)];
rc = R * C;
h.body  = plot(ax, rx+rc(1,:), ry+rc(2,:), 'y-', 'LineWidth', 2.5);
h.dot   = plot(ax, rx, ry, 'wo', 'MarkerFaceColor','y', 'MarkerSize', 8);
h.arrow = plot(ax, [rx, rx+lp*cos(rtheta)], [ry, ry+lp*sin(rtheta)], 'w-', 'LineWidth', 2);
end