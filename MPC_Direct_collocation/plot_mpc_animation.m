function plot_mpc_animation(t, x, t_ref, q_ref_history, L1, L2, FPS, speed)
% x 的列顺序为 [q1, dq1, q2, dq2]，参考角按 [q1_ref, q2_ref] 存储。
% t_ref 记录参考角生效的时刻，首个时刻应不晚于 t(1)。

t_anim = (t(1):speed/FPS:t(end)).';
if t_anim(end) < t(end)
    t_anim(end+1,1) = t(end);
end

q = interp1(t, x(:,[1 3]), t_anim);

% 每轮 MPC 内保持参考角，下一轮开始时切换。
ref_index = discretize(t_anim, [t_ref(:); Inf]);
q_goal = q_ref_history(ref_index,:);

x1 = L1*cos(q(:,1));
y1 = L1*sin(q(:,1));
x2 = x1 + L2*cos(sum(q,2));
y2 = y1 + L2*sin(sum(q,2));
x_goal = L1*cos(q_goal(:,1)) + L2*cos(sum(q_goal,2));
y_goal = L1*sin(q_goal(:,1)) + L2*sin(sum(q_goal,2));

fig = figure('Name', '两连杆 MPC 动画', 'Color', 'w');
ax = axes('Parent', fig);
hold(ax, 'on');
axis(ax, 'equal');
lim = 1.1*(L1 + L2);
axis(ax, [-lim lim -lim lim]);
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');

goal = plot(ax, NaN, NaN, 'gx', 'MarkerSize', 12, 'LineWidth', 2);
trail = animatedline(ax, 'Color', 'b', 'LineWidth', 1.2);
arm = plot(ax, NaN, NaN, 'ko-', 'LineWidth', 2, 'MarkerFaceColor', 'w');
tip = plot(ax, NaN, NaN, 'ro', 'MarkerFaceColor', 'r');
legend(ax, [goal, tip], {'目标末端', '实际末端'}, 'Location', 'northeast');
ttl = title(ax, '');

for k = 1:numel(t_anim)
    if ~isgraphics(fig)
        break
    end

    set(goal, 'XData', x_goal(k), 'YData', y_goal(k));
    addpoints(trail, x2(k), y2(k));
    set(arm, 'XData', [0 x1(k) x2(k)], 'YData', [0 y1(k) y2(k)]);
    set(tip, 'XData', x2(k), 'YData', y2(k));
    drawnow
    pause(1/FPS);
end
end
