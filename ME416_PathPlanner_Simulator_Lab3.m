function simulate_limo_swarm_astar()
% ===============================================================
% LIMO Swarm Simulator
% A* global planning + Pure Pursuit + robot–robot avoidance
% Includes 0.3 x 0.15 m rotated footprint boxes (visual only)
%
% Save as: simulate_limo_swarm_astar.m
% Run:     simulate_limo_swarm_astar
% ===============================================================

close all; clc; rng(2);

%% ===================== CONFIG =====================
N          = 12;     % number of robots
DT         = 0.05;   % [s] simulation time step
SIM_TIME   = 300;    % [s] max sim time

CFG.GRID_SIZE      = 0.25;   % [m] resolution of A* grid
CFG.ROBOT_RADIUS   = 0.35;   % [m] for A* obstacle inflation

CFG.LOOKAHEAD_DISTANCE = 0.6;      % [m] pure pursuit
CFG.V_DESIRED           = 0.4;    % [m/s]
CFG.MAX_LINEAR_VEL      = 0.5;    % [m/s]
CFG.MAX_ANGULAR_VEL     = deg2rad(20);
CFG.GOAL_POSITION_TOL   = 0.15;    % [m]

SAFE_DIST      = CFG.ROBOT_RADIUS * 3.0;
FULL_STOP_DIST = CFG.ROBOT_RADIUS * 1.5;

REPLAN_PERIOD     = 1.0;   % [s] min time between replans
STUCK_DIST_THRESH = 0.03;  % [m] movement threshold
STUCK_TIME        = 2.0;   % [s] stuck duration before recovery

% LIMO physical size (visual only)
LIMO_LENGTH = 0.3;   % [m]
LIMO_WIDTH  = 0.15;  % [m]

%% ===================== MAP (rectangular) =====================
xmin = 0; xmax = 8;
ymin = 0; ymax = 5;

ox = []; oy = [];

% bottom & top walls
for x = xmin:CFG.GRID_SIZE:xmax
    ox(end+1) = x; oy(end+1) = ymin; %#ok<*AGROW>
    ox(end+1) = x; oy(end+1) = ymax;
end
% left & right walls
for y = ymin:CFG.GRID_SIZE:ymax
    ox(end+1) = xmin; oy(end+1) = y;
    ox(end+1) = xmax; oy(end+1) = y;
end

%% ===================== ROBOT INITIALIZATION =====================
robots = struct([]);
inner_margin    = 1.0;
MIN_START_DIST  = 0.45;   % [m] minimum robot-to-robot spacing

fprintf('Initializing %d robots with safe separation...\n', N);

for i = 1:N
    valid = false;

    while ~valid
        sx  = xmin + inner_margin + rand*(xmax - xmin - 2*inner_margin);
        sy  = ymin + inner_margin + rand*(ymax - ymin - 2*inner_margin);
        sth = rand * 2*pi;

        valid = true;
        for j = 1:i-1
            d = hypot(sx - robots(j).pose(1), ...
                      sy - robots(j).pose(2));
            if d < MIN_START_DIST
                valid = false;
                break;
            end
        end
    end

    % Goal position (anywhere in map)
    GOAL_EDGE_MARGIN = CFG.ROBOT_RADIUS + 0.15;   % ≈ 0.5 m
  % [m]

edge = randi(4);  % 1=left, 2=right, 3=bottom, 4=top

switch edge
    case 1  % left
        gx = xmin + GOAL_EDGE_MARGIN;
        gy = ymin + rand*(ymax - ymin);

    case 2  % right
        gx = xmax - GOAL_EDGE_MARGIN;
        gy = ymin + rand*(ymax - ymin);

    case 3  % bottom
        gx = xmin + rand*(xmax - xmin);
        gy = ymin + GOAL_EDGE_MARGIN;

    case 4  % top
        gx = xmin + rand*(xmax - xmin);
        gy = ymax - GOAL_EDGE_MARGIN;
end


    robots(i).pose     = [sx, sy, sth];
    robots(i).goal     = [gx, gy];
    robots(i).path     = [];
    robots(i).traj     = [sx sy];
    robots(i).lastPlan = -inf;
    robots(i).stuckT   = 0;
    robots(i).lastPos  = [sx sy];
    robots(i).done     = false;
end


%% ===================== VISUALIZATION SETUP =====================
figure('Position',[100 100 1200 750]);
hold on; grid on; axis equal;
plot([xmin xmax xmax xmin xmin], [ymin ymin ymax ymax ymin], ...
     'k', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
title('LIMO Swarm — A* + Pure Pursuit');

colors  = lines(N);
h_traj  = gobjects(N,1);
h_path  = gobjects(N,1);
h_robot = gobjects(N,1);
h_body  = gobjects(N,1);

for i = 1:N
    h_traj(i) = plot(NaN, NaN, '-',  'Color', colors(i,:));
    h_path(i) = plot(NaN, NaN, '--', 'Color', colors(i,:));
    h_robot(i)= plot(robots(i).pose(1), robots(i).pose(2), 'o', ...
        'MarkerFaceColor', colors(i,:), ...
        'MarkerEdgeColor', 'k', ...
        'MarkerSize', 8);

    plot(robots(i).goal(1), robots(i).goal(2), 'x', ...
        'Color', colors(i,:), ...
        'LineWidth', 2, ...
        'MarkerSize', 10);

    % Body rectangle
    [rx, ry] = limo_rectangle(robots(i).pose, LIMO_LENGTH, LIMO_WIDTH);
    h_body(i) = fill(rx, ry, colors(i,:), ...
        'FaceAlpha', 0.25, ...
        'EdgeColor', colors(i,:), ...
        'LineWidth', 1.5);
end

xlim([xmin - 0.2, xmax + 0.2]);
ylim([ymin - 0.2, ymax + 0.2]);

%% ===================== MAIN SIMULATION LOOP =====================
t = 0;

while t < SIM_TIME
    t = t + DT;

    if all([robots.done])
        fprintf('All robots reached goals at t = %.1f s\n', t);
        break;
    end

    for i = 1:N
        if robots(i).done
            continue;
        end

        x  = robots(i).pose(1);
        y  = robots(i).pose(2);
        th = robots(i).pose(3);

        % ---------- Goal check ----------
        if hypot(x - robots(i).goal(1), y - robots(i).goal(2)) < CFG.GOAL_POSITION_TOL
            robots(i).done = true;
            continue;
        end

        % ---------- Replanning (A*) ----------
        if isempty(robots(i).path) || (t - robots(i).lastPlan) > REPLAN_PERIOD
            ox_dyn = ox;
            oy_dyn = oy;

            % Add ALL other robots (including finished) as obstacles
            for j = 1:N
                if j == i
                    continue;
                end
                ox_dyn(end+1) = robots(j).pose(1);
                oy_dyn(end+1) = robots(j).pose(2);
            end

            planner = astar_planner_create(ox_dyn, oy_dyn, ...
                                           CFG.GRID_SIZE, CFG.ROBOT_RADIUS);

            [rx, ry] = astar_planning(planner, x, y, ...
                                      robots(i).goal(1), robots(i).goal(2));

            if isempty(rx)
                % Fallback: straight line
                rx = [x, robots(i).goal(1)];
                ry = [y, robots(i).goal(2)];
            else
                % Returned path is goal->start; flip to start->goal
                rx = fliplr(rx);
                ry = fliplr(ry);
            end

            robots(i).path     = create_path(rx, ry);
            robots(i).lastPlan = t;
        end

        % ---------- Pure Pursuit ----------
        [lx, ly] = find_lookahead_point(x, y, robots(i).path, CFG.LOOKAHEAD_DISTANCE);

        alpha     = atan2(ly - y, lx - x);
        curvature = 2 * sin(wrap_to_pi(alpha - th)) / CFG.LOOKAHEAD_DISTANCE;
        w         = curvature * CFG.V_DESIRED;
        w         = max(min(w, CFG.MAX_ANGULAR_VEL), -CFG.MAX_ANGULAR_VEL);
        v         = CFG.V_DESIRED;

        % ---------- Robot–robot speed scaling ----------
        min_neighbor_dist = inf;
        for j = 1:N
            if j == i
                continue;
            end
            d = hypot(robots(j).pose(1) - x, robots(j).pose(2) - y);
            min_neighbor_dist = min(min_neighbor_dist, d);
        end

        if min_neighbor_dist < SAFE_DIST
            if min_neighbor_dist <= FULL_STOP_DIST
                scale = 0.05;
            else
                scale = (min_neighbor_dist - FULL_STOP_DIST) / ...
                        (SAFE_DIST - FULL_STOP_DIST);
                scale = max(0.05, min(scale, 1.0));
            end
            v = v * scale;
        end

        % ---------- Stuck detection + simple recovery ----------
        if hypot(x - robots(i).lastPos(1), y - robots(i).lastPos(2)) < STUCK_DIST_THRESH
            robots(i).stuckT = robots(i).stuckT + DT;
        else
            robots(i).stuckT  = 0;
            robots(i).lastPos = [x, y];
        end

        if robots(i).stuckT > STUCK_TIME
            v = -0.2;
            w = 0.0;
            robots(i).stuckT   = 0;
            robots(i).lastPlan = -inf;   % force replan
        end

        % ---------- Integrate unicycle model ----------
        th = wrap_to_pi(th + w * DT);
        x  = x + v * cos(th) * DT;
        y  = y + v * sin(th) * DT;

        % Keep inside map bounds
        x = min(max(x, xmin + 0.01), xmax - 0.01);
        y = min(max(y, ymin + 0.01), ymax - 0.01);

        robots(i).pose = [x, y, th];
        robots(i).traj = [robots(i).traj; x, y];
    end

    % ---------- Visualization update ----------
    for i = 1:N
        set(h_traj(i), 'XData', robots(i).traj(:,1), ...
                       'YData', robots(i).traj(:,2));

        if ~isempty(robots(i).path)
            set(h_path(i), 'XData', robots(i).path.x, ...
                           'YData', robots(i).path.y);
        else
            set(h_path(i), 'XData', NaN, 'YData', NaN);
        end

        set(h_robot(i), 'XData', robots(i).pose(1), ...
                        'YData', robots(i).pose(2));

        [rx, ry] = limo_rectangle(robots(i).pose, LIMO_LENGTH, LIMO_WIDTH);
        set(h_body(i), 'XData', rx, 'YData', ry);
    end

    title(sprintf('t = %.1f s', t));
    drawnow limitrate;
end

fprintf('Simulation finished at t = %.1f s\n', t);

end  % ===== end simulate_limo_swarm_astar =====


%% ===================== RECTANGLE HELPER =====================
function [rx, ry] = limo_rectangle(pose, L, W)
% Returns rectangle vertices for a robot pose [x y theta]
    x  = pose(1);
    y  = pose(2);
    th = pose(3);

    hL = L/2;
    hW = W/2;

    % Rectangle in body frame
    corners = [ ...
         hL,  hW;
         hL, -hW;
        -hL, -hW;
        -hL,  hW;
         hL,  hW];

    R  = [cos(th), -sin(th); sin(th), cos(th)];
    cw = (R * corners')';

    rx = cw(:,1) + x;
    ry = cw(:,2) + y;
end


%% ===================== PATH & CONTROL HELPERS =====================
function path = create_path(x, y)
    x = x(:)';  y = y(:)';
    path.x = x;
    path.y = y;
    path.s = zeros(size(x));

    for k = 2:numel(x)
        path.s(k) = path.s(k-1) + hypot(x(k) - x(k-1), y(k) - y(k-1));
    end
end

function [lx, ly] = find_lookahead_point(robot_x, robot_y, path, lookahead_distance)
    % Same structure as your real-robot function
    distances = sqrt((path.x - robot_x).^2 + (path.y - robot_y).^2);
    [~, closest_idx] = min(distances);

    s_closest   = path.s(closest_idx);
    s_lookahead = s_closest + lookahead_distance;

    if s_lookahead > path.s(end)
        lookahead_idx = numel(path.x);
    else
        idx = find(path.s >= s_lookahead, 1, 'first');
        if isempty(idx)
            lookahead_idx = numel(path.x);
        else
            lookahead_idx = idx;
        end
    end

    lx = path.x(lookahead_idx);
    ly = path.y(lookahead_idx);
end

function angle = wrap_to_pi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end


%% ===================== A* PLANNER (from your code) =====================
function planner = astar_planner_create(ox, oy, resolution, rr)
    % Create A* planner structure
    planner = struct();
    planner.resolution = resolution;
    planner.rr = rr;
    planner.motion = get_motion_model();

    % Calculate grid bounds from obstacle extents
    planner.min_x = floor(min(ox));
    planner.min_y = floor(min(oy));
    planner.max_x = ceil(max(ox));
    planner.max_y = ceil(max(oy));

    planner.x_width = round((planner.max_x - planner.min_x) / resolution) + 1;
    planner.y_width = round((planner.max_y - planner.min_y) / resolution) + 1;

    % Initialize occupancy grid
    planner.obstacle_map = false(planner.x_width, planner.y_width);

    % Inflate obstacles
    for ix = 0:planner.x_width-1
        x = calc_grid_position(planner, ix, planner.min_x);
        for iy = 0:planner.y_width-1
            y = calc_grid_position(planner, iy, planner.min_y);
            for k = 1:length(ox)
                d = sqrt((ox(k) - x)^2 + (oy(k) - y)^2);
                if d <= rr
                    planner.obstacle_map(ix+1, iy+1) = true;
                    break;
                end
            end
        end
    end
end

function [rx, ry] = astar_planning(planner, sx, sy, gx, gy)
    % A* path planning (grid-based)

    start_node = create_node( ...
        calc_xy_index(planner, sx, planner.min_x), ...
        calc_xy_index(planner, sy, planner.min_y), ...
        0.0, -1);

    goal_node = create_node( ...
        calc_xy_index(planner, gx, planner.min_x), ...
        calc_xy_index(planner, gy, planner.min_y), ...
        0.0, -1);

    open_set   = containers.Map('KeyType','double','ValueType','any');
    closed_set = containers.Map('KeyType','double','ValueType','any');

    start_idx = calc_grid_index(planner, start_node);
    open_set(start_idx) = start_node;

    goal_reached = false;

    while open_set.Count > 0
        keys = cell2mat(open_set.keys);
        min_cost = inf;
        c_id = keys(1);

        % Find node with minimum f = g + h
        for k = keys
            node = open_set(k);
            f_cost = node.cost + calc_heuristic(goal_node, node);
            if f_cost < min_cost
                min_cost = f_cost;
                c_id = k;
            end
        end

        current = open_set(c_id);

        % Goal check
        if current.x == goal_node.x && current.y == goal_node.y
            goal_node.parent_index = current.parent_index;
            goal_node.cost         = current.cost;
            goal_reached = true;
            break;
        end

        % Move to closed set
        remove(open_set, c_id);
        closed_set(c_id) = current;

        % Expand neighbors
        for i = 1:size(planner.motion, 1)
            node = create_node( ...
                current.x + planner.motion(i,1), ...
                current.y + planner.motion(i,2), ...
                current.cost + planner.motion(i,3), ...
                c_id);

            n_id = calc_grid_index(planner, node);

            if ~verify_node(planner, node)
                continue;
            end

            if isKey(closed_set, n_id)
                continue;
            end

            if ~isKey(open_set, n_id)
                open_set(n_id) = node;
            else
                if open_set(n_id).cost > node.cost
                    open_set(n_id) = node;
                end
            end
        end
    end

    if ~goal_reached
        rx = [];
        ry = [];
        return;
    end

    [rx, ry] = calc_final_path(planner, goal_node, closed_set);
end

function node = create_node(x, y, cost, parent_index)
    node = struct('x', x, 'y', y, 'cost', cost, 'parent_index', parent_index);
end

function [rx, ry] = calc_final_path(planner, goal_node, closed_set)
    rx = calc_grid_position(planner, goal_node.x, planner.min_x);
    ry = calc_grid_position(planner, goal_node.y, planner.min_y);
    parent_index = goal_node.parent_index;

    while parent_index ~= -1
        n = closed_set(parent_index);
        rx = [rx, calc_grid_position(planner, n.x, planner.min_x)];
        ry = [ry, calc_grid_position(planner, n.y, planner.min_y)];
        parent_index = n.parent_index;
    end
end

function d = calc_heuristic(n1, n2)
    w = 1.0;
    d = w * sqrt((n1.x - n2.x)^2 + (n1.y - n2.y)^2);
end

function pos = calc_grid_position(planner, index, min_position)
    pos = index * planner.resolution + min_position;
end

function idx = calc_xy_index(planner, position, min_pos)
    idx = round((position - min_pos) / planner.resolution);
end

function grid_idx = calc_grid_index(planner, node)
    % Unique integer index for node in grid
    grid_idx = (node.y) * planner.x_width + node.x;
end

function valid = verify_node(planner, node)
    % Map from node indices to world coords
    px = calc_grid_position(planner, node.x, planner.min_x);
    py = calc_grid_position(planner, node.y, planner.min_y);

    if px < planner.min_x || py < planner.min_y || ...
       px > planner.max_x || py > planner.max_y
        valid = false;
        return;
    end

    ix = node.x + 1;
    iy = node.y + 1;

    if ix < 1 || ix > planner.x_width || ...
       iy < 1 || iy > planner.y_width
        valid = false;
        return;
    end

    if planner.obstacle_map(ix, iy)
        valid = false;
        return;
    end

    valid = true;
end

function motion = get_motion_model()
    motion = [ ...
        1,  0, 1;
        0,  1, 1;
       -1,  0, 1;
        0, -1, 1;
       -1, -1, sqrt(2);
       -1,  1, sqrt(2);
        1, -1, sqrt(2);
        1,  1, sqrt(2)];
end
