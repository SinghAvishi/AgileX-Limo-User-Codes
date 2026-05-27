function runLimoPlannerPath_AStar_Realtime()
% runLimoPlannerPath_AStar_Realtime
% ---------------------------------------------------------------
% Real-time A*-based path planning + pure pursuit control
% for a single physical LIMO robot, using MoCap over MQTT.
%
% NEW: Goal and RunState management via MQTT
%   - Listens to goal/limoXXX for {"goal": [x, y]}
%   - Listens to runState for WAIT/GO/STOP/HALT commands
%   - Only processes goals in WAIT state
%   - Ignores goals in RUN/STOP/HALT states
% ---------------------------------------------------------------

clearvars; close all; clc;

fprintf('\n========================================\n');
fprintf('  REAL-TIME A* PLANNER + LIMO CONTROL\n');
fprintf('  WITH MQTT GOAL & RUNSTATE\n');
fprintf('========================================\n\n');

%% ==========================
%% USER INPUT: LIMO CONFIG
%% ==========================
fprintf('--- LIMO Network Configuration ---\n');
fprintf('Enter the last 3 digits of LIMO IP address\n');
fprintf('Example: Enter "101" for 192.168.1.101\n');
LIMO_IP_LAST_3 = input('IP last 3 digits: ', 's');

fprintf('\nEnter LIMO identification number\n');
fprintf('Example: "777" or "807"\n');
LIMO_NUMBER = input('LIMO number: ', 's');

fprintf('\nConfiguration Summary:\n');
fprintf('  LIMO IP: 192.168.1.%s\n', LIMO_IP_LAST_3);
fprintf('  LIMO Number: %s\n', LIMO_NUMBER);
fprintf('  Goal topic: goal/limo%s\n', LIMO_NUMBER);
fprintf('  RunState topic: runState\n');
fprintf('  Other robots: Wildcard subscription to rb/limo#\n');
fprintf('========================================\n\n');

%% ==========================
%% CONFIGURATION STRUCT
%% ==========================

CFG.limo_ip_prefix = '192.168.1.';
CFG.limo_port      = 12345;
CFG.mqtt_broker    = 'mqtt://rasticvm.lan';

% MoCap origin transform
CFG.MOCAP_ORIGIN_X = -4.5;
CFG.MOCAP_ORIGIN_Y =  2.7;

% Pure-pursuit control parameters
CFG.LOOKAHEAD_DISTANCE = 0.5;    % [m]
CFG.V_DESIRED          = 0.2;    % [m/s]
CFG.GOAL_POSITION_TOL  = 0.10;   % [m]
CFG.GOAL_HEADING_TOL   = deg2rad(25); % [rad]

% Safety limits
CFG.MAX_LINEAR_VEL  = 0.2;       % [m/s]
CFG.MAX_ANGULAR_VEL = deg2rad(60); % [rad/s]
CFG.MAX_TIME        = 360;       % [s]

% Control loop timing
CFG.CONTROL_RATE = 20;           % [Hz]
CFG.dt           = 1/CFG.CONTROL_RATE;

% A* planning parameters
CFG.GRID_SIZE     = 0.5;     % [m] grid resolution
CFG.ROBOT_RADIUS  = 0.3;     % [m] inflation radius
CFG.REPLAN_PERIOD = 1.0;     % [s] how often to replan

% Robot-robot avoidance distances
CFG.SAFE_DIST = CFG.ROBOT_RADIUS * 3.0;       % start slowing down
CFG.FULL_STOP_DIST = CFG.ROBOT_RADIUS * 1.5;  % almost stop

% Stuck detection
CFG.STUCK_TIME_THRESHOLD = 2.0;  % [s]
CFG.STUCK_DIST_THRESHOLD = 0.02; % [m]

% Loop detection
CFG.LOOPS_BEFORE_RECOVERY = 2;
CFG.LOOP_RECOVERY_DURATION = 1.0; % [s]

%% ==========================
%% BUILD A* MAP
%% ==========================
fprintf('Building A* map...\n');
[ox_map, oy_map, MAP] = build_clean_map(CFG.GRID_SIZE);
xmin = MAP.xmin; xmax = MAP.xmax;
ymin = MAP.ymin; ymax = MAP.ymax;

fprintf('Map: [%.1f, %.1f] x [%.1f, %.1f] m\n', xmin, xmax, ymin, ymax);
fprintf('Static obstacles: %d points\n\n', numel(ox_map));

%% ==========================
%% COMMUNICATION SETUP
%% ==========================

tcp = [];
mqttSelf = [];
mqttOthers = [];
mqttGoal = [];
mqttRunState = [];

% try
    %% MQTT for *this* LIMO (pose)
    fprintf('Connecting to MQTT broker (self): %s\n', CFG.mqtt_broker);
    mqttSelf = mqttclient(CFG.mqtt_broker, ClientID="Punnisa");
    subscribe(mqttSelf, sprintf("rb/limo%s", LIMO_NUMBER));
    fprintf('  Subscribed to: rb/limo%s\n', LIMO_NUMBER);

    %% MQTT for goal
    fprintf('Connecting to MQTT broker (goal)...\n');
    % mqttGoal = mqttclient(CFG.mqtt_broker);
    goal_topic = sprintf("goal/limo%s", LIMO_NUMBER);
    subscribe(mqttSelf, sprintf("goal/limo%s", LIMO_NUMBER));
    fprintf('  Subscribed to: %s\n', goal_topic);

    %% MQTT for runState
    fprintf('Connecting to MQTT broker (runState)...\n');
    % mqttRunState = mqttclient(CFG.mqtt_broker);
    subscribe(mqttSelf, sprintf("cmd/limo%s", LIMO_NUMBER));
    fprintf('  Subscribed to: runState\n');

    %% MQTT for other robots (wildcard subscription)
    fprintf('Connecting to MQTT broker (other robots)...\n');
    % mqttOthers = mqttclient(CFG.mqtt_broker);
    subscribe(mqttSelf, "rb/#");
    fprintf('  Subscribed to: rb/# (wildcard for all robots)\n');

    %% WAIT FOR MOCAP STABILIZATION (SELF)
    fprintf('\nWaiting for MoCap data for LIMO %s...\n', LIMO_NUMBER);
    wait_time = 1;
    for i = 1:wait_time
        fprintf('  %d/%d seconds... ', i, wait_time);
        [test_pose, valid] = getRobotPose_MQTT(mqttSelf, LIMO_NUMBER, CFG, [], 0);
        if valid
            fprintf('Pose: (%.2f, %.2f, %.1f°)\n', ...
                    test_pose(1), test_pose(2), rad2deg(test_pose(3)));
        else
            fprintf('No data yet\n');
        end
        pause(1);
    end
    fprintf('MoCap wait complete.\n\n');

    %% GET INITIAL POSE (SELF)
    fprintf('Locking initial pose...\n');
    % valid = false;
    % max_attempts = inf;
    % curr_pose = [0,0,0];
    % for attempt = 1:max_attempts
    %     [curr_pose, valid] = getRobotPose_MQTT(mqttSelf, LIMO_NUMBER, CFG, [], 0);
    %     if valid
    %         fprintf('✓ Initial pose: (%.2f, %.2f, %.1f°)\n', ...
    %                 curr_pose(1), curr_pose(2), rad2deg(curr_pose(3)));
    %         break;
    %     end
    %     fprintf('  Attempt %d/%d: No valid data...\n', attempt, max_attempts);
    %     pause(0.5);
    % end
    % if ~valid
    %     error('Cannot read initial MoCap data for LIMO %s.', LIMO_NUMBER);
    % end
    curr_pose = [0,0,0];
    valid = false;
    while ~valid
        [curr_pose, valid] = getRobotPose_MQTT(mqttSelf, LIMO_NUMBER, CFG, [], 0);
        pause(0.5);
    end
    start_pose = curr_pose;

    %% TCP TO LIMO
    limo_ip = [CFG.limo_ip_prefix, LIMO_IP_LAST_3];
    if (limo_ip > 0)
        fprintf('\nConnecting to LIMO TCP: %s:%d\n', limo_ip, CFG.limo_port);
        tcp = tcpclient(limo_ip, CFG.limo_port, 'Timeout', 5);
        write(tcp, uint8('0.00,0.00'));
        pause(0.5);
    end

    %% ==========================
    %% STATE MACHINE INITIALIZATION
    %% ==========================
    
    % RunState: WAIT, GO, STOP, HALT
    runState = 'WAIT';
    
    % Goal tracking
    Goal = [];  % Will be populated from MQTT
    has_goal = false;
    
    % Path variables
    path = struct('x', [], 'y', [], 's', []);
    lastPlanTime = -inf;
    
    % For plotting
    trajectory = start_pose(1:2);
    
    % Static obstacles
    static_ox = ox_map;
    static_oy = oy_map;
    newly_added_ox = [];
    newly_added_oy = [];

    %% ==========================
    %% VISUALIZATION SETUP
    %% ==========================
    figure('Name','Real-time A* + LIMO + MQTT','Position',[100 100 1000 800]);
    hold on; grid on; axis equal;

    % Static obstacles
    plot(ox_map, oy_map, 'ks', 'MarkerSize', 6, 'MarkerFaceColor', 'k', ...
         'DisplayName','Static obstacles');

    % Goal (initially empty)
    goal_plot = plot(NaN, NaN, 'g*', 'MarkerSize', 16, 'LineWidth', 2, ...
         'DisplayName','Goal');

    % Start
    plot(start_pose(1), start_pose(2), 'ms', 'MarkerSize', 14, ...
         'MarkerFaceColor','m', 'DisplayName','Start');

    % A* path
    path_plot = plot(NaN, NaN, 'b-', 'LineWidth', 2, ...
                     'DisplayName','A* path');

    % Other robots
    others_plot = plot(NaN, NaN, 'ro', 'MarkerSize', 10, ...
                       'MarkerFaceColor','r', 'DisplayName','Other robots');

    % Stuck obstacles
    stuck_plot = plot(NaN, NaN, 'cX', 'MarkerSize', 12, ...
                     'DisplayName','Stuck obstacles');

    % Trajectory
    traj_plot = plot(trajectory(1), trajectory(2), 'k-', 'LineWidth', 1.5, ...
                     'DisplayName','Actual path');

    % Robot
    robot_plot = plot(start_pose(1), start_pose(2), 'yo', 'MarkerSize', 12, ...
                      'MarkerFaceColor','y', 'DisplayName','LIMO');

    % Lookahead
    lookahead_plot = plot(NaN, NaN, 'mo', 'MarkerSize', 10, ...
                          'MarkerFaceColor','m', 'DisplayName','Lookahead');

    xlabel('X [m]'); ylabel('Y [m]');
    xlim([xmin-0.1, xmax+0.1]);
    ylim([ymin-0.1, ymax+0.1]);
    title(sprintf('LIMO %s - WAIT State - Awaiting Goal', LIMO_NUMBER));
    legend('Location','eastoutside');

    %% ==========================
    %% MAIN CONTROL LOOP
    %% ==========================
    fprintf('\n------------------------------------------------------------\n');
    fprintf('MAIN LOOP - State Machine Active\n');
    fprintf('------------------------------------------------------------\n');
    fprintf('Initial state: WAIT\n');
    fprintf('Waiting for goal message on: goal/limo%s\n\n', LIMO_NUMBER);

    start_time = tic;
    prev_pose_self = start_pose;
    
    % Storage for other robot poses (keyed by limo number)
    other_robot_poses = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    % Movement state (for stuck/loop detection)
    robot_state = 'MOVING';
    stuck_timer = 0.0;
    last_pos_stuck_check = start_pose(1:2);
    cumulative_heading_change = 0.0;
    loop_counter = 0;
    recovery_mode = false;
    recovery_start_time = 0.0;
    
    % Stuck recovery
    stuck_recovery_timer = 0.0;
    new_stuck_obs_pos = [0,0];

    while true
        loop_start = tic;
        curr_time = toc(start_time);

        %% --- CHECK RUNSTATE ---
       [runState, should_exit] = checkRunState(mqttSelf, runState, LIMO_NUMBER);
        
        if should_exit
            fprintf('\n! HALT received - Exiting gracefully\n');
            write(tcp, uint8('0.00,0.00'));
            break;
        end

        %% --- PROCESS GOAL MESSAGES (only in WAIT state) ---
        if strcmp(runState, 'WAIT')
            [Goal, goal_received] = checkGoalMessage(mqttSelf, LIMO_NUMBER);
            
            if goal_received
                has_goal = true;
                fprintf('\n✓ Goal received: [%.2f, %.2f]\n', Goal(1), Goal(2));
                set(goal_plot, 'XData', Goal(1), 'YData', Goal(2));
                
                % Plan initial path
                fprintf('Planning initial A* path...\n');
                planner = astar_planner_create(static_ox, static_oy, CFG.GRID_SIZE, CFG.ROBOT_RADIUS);
                [rx, ry] = astar_planning(planner, start_pose(1), start_pose(2), Goal(1), Goal(2));
                
                if isempty(rx) || numel(rx) < 2
                    fprintf('  A* failed → straight line\n');
                    rx = linspace(start_pose(1), Goal(1), 15);
                    ry = linspace(start_pose(2), Goal(2), 15);
                else
                    rx = fliplr(rx);
                    ry = fliplr(ry);
                    fprintf('  A* path: %d points\n', numel(rx));
                end
                
                path = create_path(rx, ry);
                lastPlanTime = curr_time;
                set(path_plot, 'XData', path.x, 'YData', path.y);
                
                fprintf('Goal set. Waiting for GO command...\n');
            end
        end

        %% --- GET CURRENT POSE ---
        [curr_pose, valid] = getRobotPose_MQTT(mqttSelf, LIMO_NUMBER, CFG, prev_pose_self, CFG.dt);
        if ~valid
            pause(CFG.dt/2);
            continue;
        end
        prev_pose_self = curr_pose;
        
        curr_x = curr_pose(1);
        curr_y = curr_pose(2);
        curr_theta = curr_pose(3);

        %% --- GET OTHER ROBOT POSES ---
        neighbor_positions = getOtherRobotPoses(mqttSelf, LIMO_NUMBER, CFG, other_robot_poses);

        %% --- COMMAND LOGIC BASED ON RUNSTATE ---
        v_cmd = 0.0;
        w_cmd = 0.0;
        lookahead_x = curr_x;
        lookahead_y = curr_y;
        
        if strcmp(runState, 'WAIT')
            % WAIT state: stay still, no motion
            v_cmd = 0.0;
            w_cmd = 0.0;
            
        elseif strcmp(runState, 'STOP')
            % STOP state: must stop within 100ms, stay stopped
            v_cmd = 0.0;
            w_cmd = 0.0;
            
        elseif strcmp(runState, 'GO')
            % GO state: normal operation IF we have a goal
            if ~has_goal || isempty(Goal)
                v_cmd = 0.0;
                w_cmd = 0.0;
            else
                % Check if goal reached
                dist_to_goal = hypot(curr_x - Goal(1), curr_y - Goal(2));
                if dist_to_goal < CFG.GOAL_POSITION_TOL
                    fprintf('\n✓ Goal reached at t=%.1fs\n', curr_time);
                    v_cmd = 0.0;
                    w_cmd = 0.0;
                    has_goal = false;  % Clear goal
                    runState = 'WAIT';  % Return to WAIT
                    fprintf('Returning to WAIT state\n');
                else
                    % STUCK DETECTION
                    if strcmp(robot_state, 'MOVING')
                        dist_moved = hypot(curr_x - last_pos_stuck_check(1), curr_y - last_pos_stuck_check(2));
                        
                        if dist_moved < CFG.STUCK_DIST_THRESHOLD
                            stuck_timer = stuck_timer + CFG.dt;
                        else
                            stuck_timer = 0.0;
                            last_pos_stuck_check = [curr_x, curr_y];
                        end
                        
                        if stuck_timer > CFG.STUCK_TIME_THRESHOLD
                            fprintf('\n! ROBOT STUCK at t=%.2fs\n', curr_time);
                            robot_state = 'STUCK_STOP';
                            stuck_recovery_timer = curr_time + 1.0;
                            new_stuck_obs_pos = [curr_x, curr_y];
                            stuck_timer = 0.0;
                        end
                    end

                    % LOOP DETECTION
                    if curr_time > 1.0
                        heading_change = wrapToPi(curr_theta - prev_pose_self(3));
                        cumulative_heading_change = cumulative_heading_change + abs(heading_change);
                        
                        if cumulative_heading_change >= 2 * pi
                            loop_counter = loop_counter + 1;
                            cumulative_heading_change = 0.0;
                            if loop_counter >= CFG.LOOPS_BEFORE_RECOVERY
                                recovery_mode = true;
                                recovery_start_time = curr_time;
                                loop_counter = 0;
                                fprintf('Loop detected - Recovery mode\n');
                            end
                        end
                    end

                    % REPLANNING
                    if (curr_time - lastPlanTime) >= CFG.REPLAN_PERIOD || isempty(path.x)
                        current_ox = static_ox;
                        current_oy = static_oy;
                        
                        % Add other robots as obstacles
                        if ~isempty(neighbor_positions)
                            for k = 1:size(neighbor_positions, 1)
                                for ang = 0:pi/2:(2*pi - pi/2)
                                    current_ox = [current_ox, neighbor_positions(k,1) + CFG.ROBOT_RADIUS*cos(ang)];
                                    current_oy = [current_oy, neighbor_positions(k,2) + CFG.ROBOT_RADIUS*sin(ang)];
                                end
                            end
                        end

                        planner = astar_planner_create(current_ox, current_oy, CFG.GRID_SIZE, CFG.ROBOT_RADIUS);
                        [rx, ry] = astar_planning(planner, curr_x, curr_y, Goal(1), Goal(2));

                        if isempty(rx) || numel(rx) < 2
                            rx = linspace(curr_x, Goal(1), 15);
                            ry = linspace(curr_y, Goal(2), 15);
                        else
                            rx = fliplr(rx);
                            ry = fliplr(ry);
                        end

                        path = create_path(rx, ry);
                        lastPlanTime = curr_time;
                        set(path_plot, 'XData', path.x, 'YData', path.y);
                    end

                    % CALCULATE MOTION COMMAND
                    if strcmp(robot_state, 'STUCK_STOP')
                        if curr_time >= stuck_recovery_timer
                            robot_state = 'STUCK_BACKUP';
                            stuck_recovery_timer = curr_time + 3.5;
                        end
                        
                    elseif strcmp(robot_state, 'STUCK_BACKUP')
                        v_cmd = -0.2;
                        if curr_time >= stuck_recovery_timer
                            robot_state = 'STUCK_REPLAN';
                        end
                        
                    elseif strcmp(robot_state, 'STUCK_REPLAN')
                        % Add virtual obstacles
                        stuck_x = new_stuck_obs_pos(1);
                        stuck_y = new_stuck_obs_pos(2);
                        sim_radius = 0.7;
                        offsets = [0, 0; sim_radius, 0; -sim_radius, 0; 0, sim_radius; 0, -sim_radius];
                        for k = 1:size(offsets, 1)
                            static_ox = [static_ox, stuck_x + offsets(k,1)];
                            static_oy = [static_oy, stuck_y + offsets(k,2)];
                        end
                        newly_added_ox = [newly_added_ox, stuck_x];
                        newly_added_oy = [newly_added_oy, stuck_y];
                        
                        lastPlanTime = -inf;
                        robot_state = 'MOVING';
                        
                    elseif recovery_mode
                        if curr_time - recovery_start_time >= CFG.LOOP_RECOVERY_DURATION
                            recovery_mode = false;
                        end
                        v_cmd = CFG.V_DESIRED;
                        w_cmd = 0.0;
                        lookahead_x = curr_x + 0.5 * cos(curr_theta);
                        lookahead_y = curr_y + 0.5 * sin(curr_theta);
                        
                    else
                        % PURE PURSUIT
                        if ~isempty(path.x) && numel(path.x) >= 2
                            [lookahead_x, lookahead_y, ~, ~] = ...
                                findLookaheadPoint(curr_x, curr_y, path, CFG.LOOKAHEAD_DISTANCE);

                            alpha = atan2(lookahead_y - curr_y, lookahead_x - curr_x);
                            curvature = 2 * sin(wrapToPi(alpha - curr_theta)) / CFG.LOOKAHEAD_DISTANCE;
                            w_cmd = curvature * CFG.V_DESIRED;
                            w_cmd = max(min(w_cmd, CFG.MAX_ANGULAR_VEL), -CFG.MAX_ANGULAR_VEL);
                            v_cmd = CFG.V_DESIRED;
                        end
                    end

                    % ROBOT-ROBOT AVOIDANCE
                    if strcmp(robot_state, 'MOVING') && ~isempty(neighbor_positions)
                        min_neighbor_dist = inf;
                        for k = 1:size(neighbor_positions, 1)
                            d = hypot(neighbor_positions(k,1) - curr_x, neighbor_positions(k,2) - curr_y);
                            if d < min_neighbor_dist
                                min_neighbor_dist = d;
                            end
                        end
                        
                        if min_neighbor_dist < CFG.SAFE_DIST
                            if min_neighbor_dist <= CFG.FULL_STOP_DIST
                                scale = 0.05;
                            else
                                scale = (min_neighbor_dist - CFG.FULL_STOP_DIST) / (CFG.SAFE_DIST - CFG.FULL_STOP_DIST);
                                scale = max(0.05, min(scale, 1.0));
                            end
                            v_cmd = v_cmd * scale;
                        end
                    end
                end
            end
        end

        % Clamp velocities
        v_cmd = max(0, min(v_cmd, CFG.MAX_LINEAR_VEL));
        w_cmd = max(-CFG.MAX_ANGULAR_VEL, min(w_cmd, CFG.MAX_ANGULAR_VEL));

        %% --- SEND COMMAND ---
        cmd_str = sprintf('%.2f,%.2f', v_cmd, w_cmd);
        write(tcp, uint8(cmd_str));

        %% --- UPDATE PLOTS ---
        trajectory = [trajectory; curr_x, curr_y];
        set(traj_plot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
        set(robot_plot, 'XData', curr_x, 'YData', curr_y);
        set(lookahead_plot, 'XData', lookahead_x, 'YData', lookahead_y);
        set(stuck_plot, 'XData', newly_added_ox, 'YData', newly_added_oy);

        if ~isempty(neighbor_positions)
            set(others_plot, 'XData', neighbor_positions(:,1), 'YData', neighbor_positions(:,2));
        else
            set(others_plot, 'XData', NaN, 'YData', NaN);
        end

        % Title with state info
        if has_goal && ~isempty(Goal)
            dist_to_goal = hypot(curr_x - Goal(1), curr_y - Goal(2));
            title_str = sprintf(['LIMO %s | RunState: %s | MovState: %s | t=%.1fs | ', ...
                               'd2goal=%.2fm | v=%.2f m/s | w=%.1f°/s'], ...
                  LIMO_NUMBER, runState, robot_state, curr_time, dist_to_goal, v_cmd, rad2deg(w_cmd));
        else
            title_str = sprintf('LIMO %s | RunState: %s | t=%.1fs | No Goal', ...
                  LIMO_NUMBER, runState, curr_time);
        end
        title(title_str);

        drawnow limitrate;

        %% --- LOOP RATE ---
        elapsed = toc(loop_start);
        if elapsed < CFG.dt
            pause(CFG.dt - elapsed);
        end
    end

% catch ME
%     fprintf('\nError: %s\n', ME.message);
%     if ~isempty(ME.stack)
%         fprintf('  In: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
%     end
% end

%% ==========================
%% CLEANUP
%% ==========================
if ~isempty(tcp) && isvalid(tcp)
    write(tcp, uint8('0.00,0.00'));
    pause(0.3);
    clear tcp;
end

if ~isempty(mqttSelf), clear mqttSelf; end
if ~isempty(mqttGoal), clear mqttGoal; end
if ~isempty(mqttRunState), clear mqttRunState; end
if ~isempty(mqttOthers), clear mqttOthers; end

fprintf('\nScript complete.\n\n');

end  % main function


%% ========================================================================
%% MQTT MESSAGE HANDLERS
%% ========================================================================

function [newState, should_exit] = checkRunState(mqttClient, currentState, LIMO_NUMBER)
    % Check for runState message and update state
    % Returns: newState (WAIT/GO/STOP/HALT), should_exit (true if HALT)
    
    newState = currentState;
    should_exit = false;
    
    % try
        msg = peek(mqttClient);
        if isempty(msg)
            return;
        end
        
        topics = string(msg.Topic);          % convert to string array
        idx = find(topics == strcat("cmd/limo",string(LIMO_NUMBER)));      % returns row indices

        if (isempty(idx))
            return;
        end
        % 
        % if ~strcmp(char(msg.Topic), sprintf('cmd/limo%s',LIMO_NUMBER))
        %     return;
        % end
        
        dataStr = char(msg.Data(idx));
        
        % Valid commands: WAIT, GO, STOP, HALT
        if strcmp(dataStr, 'WAIT') || strcmp(dataStr, 'GO') || ...
           strcmp(dataStr, 'STOP') || strcmp(dataStr, 'HALT')
            
            if ~strcmp(dataStr, currentState)
                fprintf('\n>>> RunState changed: %s → %s\n', currentState, dataStr);
            
                newState = dataStr;
                
                if strcmp(dataStr, 'HALT')
                    should_exit = true;
                end
            end
        end
        
    % catch
    %     % Silent fail
    % end
end


%% ========================================================================
%% A* PATH PLANNING FUNCTIONS
%% ========================================================================

function planner = astar_planner_create(ox, oy, resolution, rr)
    planner = struct();
    planner.resolution = resolution;
    planner.rr = rr;
    planner.motion = get_motion_model();
    
    planner.min_x = round(min(ox));
    planner.min_y = round(min(oy));
    planner.max_x = round(max(ox));
    planner.max_y = round(max(oy));
    
    planner.x_width = round((planner.max_x - planner.min_x) / resolution);
    planner.y_width = round((planner.max_y - planner.min_y) / resolution);
    
    planner.obstacle_map = false(planner.x_width, planner.y_width);
    
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
    start_node = create_node(calc_xy_index(planner, sx, planner.min_x), ...
                            calc_xy_index(planner, sy, planner.min_y), 0.0, -1);
    goal_node = create_node(calc_xy_index(planner, gx, planner.min_x), ...
                           calc_xy_index(planner, gy, planner.min_y), 0.0, -1);
    
    open_set = containers.Map('KeyType', 'double', 'ValueType', 'any');
    closed_set = containers.Map('KeyType', 'double', 'ValueType', 'any');
    
    start_idx = calc_grid_index(planner, start_node);
    open_set(start_idx) = start_node;
    
    while open_set.Count > 0
        keys = cell2mat(open_set.keys);
        min_cost = inf;
        c_id = keys(1);
        
        for k = keys
            node = open_set(k);
            f_cost = node.cost + calc_heuristic(goal_node, node);
            if f_cost < min_cost
                min_cost = f_cost;
                c_id = k;
            end
        end
        
        current = open_set(c_id);
        
        if current.x == goal_node.x && current.y == goal_node.y
            goal_node.parent_index = current.parent_index;
            goal_node.cost = current.cost;
            break;
        end
        
        remove(open_set, c_id);
        closed_set(c_id) = current;
        
        for i = 1:size(planner.motion, 1)
            node = create_node(current.x + planner.motion(i,1), ...
                             current.y + planner.motion(i,2), ...
                             current.cost + planner.motion(i,3), c_id);
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
    grid_idx = (node.y - planner.min_y) * planner.x_width + (node.x - planner.min_x);
end

function valid = verify_node(planner, node)
    px = calc_grid_position(planner, node.x, planner.min_x);
    py = calc_grid_position(planner, node.y, planner.min_y);
    
    if px < planner.min_x || py < planner.min_y || px >= planner.max_x || py >= planner.max_y
        valid = false;
        return;
    end
    
    if planner.obstacle_map(node.x + 1, node.y + 1)
        valid = false;
        return;
    end
    
    valid = true;
end

function motion = get_motion_model()
    motion = [1, 0, 1;
             0, 1, 1;
             -1, 0, 1;
             0, -1, 1;
             -1, -1, sqrt(2);
             -1, 1, sqrt(2);
             1, -1, sqrt(2);
             1, 1, sqrt(2)];
end

%% ========================================================================
%% PATH STRUCTURE
%% ========================================================================

function path = create_path(x, y)
    path = struct();
    path.x = x(:)';
    path.y = y(:)';
    path.s = zeros(1, length(x));
    
    for i = 2:length(x)
        dx = path.x(i) - path.x(i-1);
        dy = path.y(i) - path.y(i-1);
        path.s(i) = path.s(i-1) + sqrt(dx^2 + dy^2);
    end
end

%% ========================================================================
%% MAP BUILDING
%% ========================================================================

function [ox, oy, MAP] = build_clean_map(grid)
    MAP.xmin = -4.5;
    MAP.xmax = 4.5;
    MAP.ymin = -2.5;
    MAP.ymax = 2.5;
    
    ox = [];
    oy = [];
    
    % Outer boundary
    bx = MAP.xmin:grid:MAP.xmax;
    by = MAP.ymin:grid:MAP.ymax;
    
    % bottom
    ox = [ox bx];
    oy = [oy MAP.ymin * ones(size(bx))];
    
    % top
    ox = [ox bx];
    oy = [oy MAP.ymax * ones(size(bx))];
    
    % left
    ox = [ox MAP.xmin * ones(size(by))];
    oy = [oy by];
    
    % right
    ox = [ox MAP.xmax * ones(size(by))];
    oy = [oy by];
end

%% ========================================================================
%% MQTT POSE HELPER
%% ========================================================================

function [pose, valid] = getRobotPose_MQTT(mqttClient, limoNum, CFG, prev_pose, dt)
    pose  = [0, 0, 0];
    valid = false;
    
    % try
        mqttMsg = peek(mqttClient);
        if isempty(mqttMsg)
            return;
        end

        topics = string(mqttMsg.Topic);          % convert to string array
        idx = find(topics == strcat("rb/limo",string(limoNum)));      % returns row indices

        if (isempty(idx))
            return;
        end
        
        % expected_topic = sprintf('rb/limo%s', limoNum);
        % % if ~strcmp(char(mqttMsg.Topic), expected_topic)
        % if ~any(all(char(mqttMsg.Topic) == expected_topic,2))
        %     return;
        % end
        
        jsonString = char(mqttMsg.Data(idx(1)));
        jsonData   = jsondecode(jsonString);
        
        if ~isfield(jsonData, 'pos') || ~isfield(jsonData, 'rot')
            return;
        end
        
        mocap_x = jsonData.pos(1);
        mocap_y = jsonData.pos(3);
        
        % x = mocap_x - CFG.MOCAP_ORIGIN_X;
        % y = -(mocap_y - CFG.MOCAP_ORIGIN_Y);

        x = mocap_x;
        y = -mocap_y;
        
        if ~isempty(prev_pose) && dt > 0
            dx = x - prev_pose(1);
            dy = y - prev_pose(2);
            speed = hypot(dx, dy) / dt;
            speed_threshold = 0.05;
            
            if speed > speed_threshold
                theta = atan2(dy, dx);
            else
                theta = prev_pose(3);
            end
        else
            if numel(jsonData.rot) >= 3
                theta = -jsonData.rot(3);
            else
                theta = 0;
            end
        end
        
        pose  = [x, y, theta];
        valid = true;
        
    % catch
    %     % silent fail
    % end
end

%% ========================================================================
%% PURE PURSUIT HELPER
%% ========================================================================

function [lookahead_x, lookahead_y, lookahead_idx, crosstrack_error] = ...
    findLookaheadPoint(robot_x, robot_y, path, lookahead_distance)
    
    distances = sqrt((path.x - robot_x).^2 + (path.y - robot_y).^2);
    [crosstrack_error, closest_idx] = min(distances);
    
    s_closest   = path.s(closest_idx);
    s_lookahead = s_closest + lookahead_distance;
    
    if s_lookahead > path.s(end)
        lookahead_idx = numel(path.x);
    else
        lookahead_idx = find(path.s >= s_lookahead, 1, 'first');
        if isempty(lookahead_idx)
            lookahead_idx = numel(path.x);
        end
    end
    
    lookahead_x = path.x(lookahead_idx);
    lookahead_y = path.y(lookahead_idx);
end

function [Goal, goal_received] = checkGoalMessage(mqttClient, limoNum)
    % Check for goal message in format {"goal": [x, y]}
    % Returns: Goal as [x; y], goal_received flag
    
    Goal = [];
    goal_received = false;
    
    % try
        msg = peek(mqttClient);
        if isempty(msg)
            return;
        end
        
        topics = string(msg.Topic);          % convert to string array


        expected_topic = strcat("goal/limo",string(limoNum));
        idx = find(topics == expected_topic);      % returns row indices

        if (isempty(idx))
            return;
        end
        
        jsonString = char(msg.Data(idx(1)));
        jsonData = jsondecode(jsonString);
        
        if isfield(jsonData, 'goal') && numel(jsonData.goal) == 2
            Goal = [jsonData.goal(1); jsonData.goal(2)];
            goal_received = true;
        end
    % 
    % catch 
    %     % Silent fail
    % end
end

function neighbor_positions = getOtherRobotPoses(mqttClient, myLimoNum, CFG, robot_poses_map)
    % Get poses of all other robots from wildcard subscription
    % Returns: Nx2 array of [x, y] positions
    
    neighbor_positions = [];
    
    % try
        % % Read all available messages
        % while true
            msg = peek(mqttClient);
            if isempty(msg)
                return;
            end
            %%

            for i=1:height(msg)
                %%

            
                % Parse topic to get limo number
                topic = string(msg.Topic);
                if ~startsWith(topic(i), 'rb/limo')
                    continue;
                end
                
                % Extract limo number from topic (e.g., "rb/limo807" -> "807")
                topicIndividual = char(topic(i));
                limo_num = topicIndividual(8:end);
            
            % % Skip if this is our own r80obot
            % if strcmp(limo_num, myLimoNum)
            %     continue;
            % end
            
            % Parse JSON data
            jsonString = char(msg.Data(i));
            jsonData = jsondecode(jsonString);
            
            if ~isfield(jsonData, 'pos') || ~isfield(jsonData, 'rot')
                continue;
            end
            
            % Transform pose
            mocap_x = jsonData.pos(1);
            mocap_y = jsonData.pos(3);
            
            % x = mocap_x - CFG.MOCAP_ORIGIN_X;
            % y = -(mocap_y - CFG.MOCAP_ORIGIN_Y);

            x = mocap_x;
            y = -mocap_y;
            
            % Estimate heading from motion or use rotation
            if isKey(robot_poses_map, limo_num)
                prev_pose = robot_poses_map(limo_num);
                dx = x - prev_pose(1);
                dy = y - prev_pose(2);
                speed = hypot(dx, dy) / CFG.dt;
                
                if speed > 0.05
                    theta = atan2(dy, dx);
                else
                    theta = prev_pose(3);
                end
            else
                if numel(jsonData.rot) >= 3
                    theta = -jsonData.rot(3);
                else
                    theta = 0;
                end
            end
            
            % Store updated pose
            robot_poses_map(limo_num) = [x, y, theta];0
            end

        % end
        
        % Extract positions from all stored robots
        all_limo_nums = keys(robot_poses_map);
        for i = 1:length(all_limo_nums)
            limo_num = all_limo_nums{i};
            if ~strcmp(limo_num, myLimoNum)
                pose = robot_poses_map(limo_num);
                neighbor_positions = [neighbor_positions; pose(1), pose(2)];
            end
        end
        
    % catch
    %     % Silent fail
    % end
end
