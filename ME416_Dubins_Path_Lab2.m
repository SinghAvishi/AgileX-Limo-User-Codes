function limo_follow_three_dubins_paths()
   % LIMO path following over 3 courses using precomputed smooth paths
   % - Uses MQTT + mocap to get live pose
   % - Uses spline-based paths (approx Dubins) precomputed from via-points
   % - Uses feedforward + feedback controller to track each path
   % - Sends [v, omega] over "rb/limo777/cmd_vel"
   %
   % IMPORTANT:
   %   Adjust the cmd_vel message format if your LIMO expects JSON/etc.
   %
   %% ---------------- Parameters ----------------
   R_min    = 0.3;       % "desired" min turn radius [m] (only for reference)
   ds       = 0.02;      % path sampling [m]
   v_ff     = 0.4;       % feedforward linear speed [m/s] (<= 1.0)
   omega_max = 2.84;     % max angular velocity [rad/s]
   v_max     = 1.0;      % max linear velocity [m/s]

   % Feedback gains (simple heuristic)
   kx     = 0.0;         % we mainly use lateral + heading error
   ky     = 2.0;
   ktheta = 3.0;

   goal_tol = 0.15;      % distance [m] to consider a goal "reached"
   dt_min   = 0.05;      % minimum control period [s]

   %% ------------- Define via-points and obstacles ----------------------
   % === Course 1 via-points ===
   via1 = [
       0.00  0.00;
       0.20  0.90;
       0.60  2.00;
       1.40  3.00;
       2.50  3.50;
       3.00  2.50;
       3.70  1.20;
       4.40  2.80;
       4.60  4.50  % goal
   ];
   obs1 = [1.5 1.5 1.5 1.5 1.5 1.5 3.5 3.5 3.5 3.5 3.5 3.5;
           0.0 0.5 1.0 1.5 2.0 2.5 4.5 4.0 3.5 3.0 2.5 2.0];

   % === Course 2 via-points ===
   via2 = [
       0.00 0.00;
       0.50 0.80;
       1.30 1.40;
       2.50 1.90;
       3.80 2.60;
       4.50 3.50;
       5.00 4.50  % goal
   ];
   obs2 = [1.0 2.5 4.0 0.0 1.5 3.0 1.0 2.5 4.0;
           1.0 1.0 1.0 2.5 2.5 2.5 4.0 4.0 4.0];

   % === Course 3 via-points ===
   via3 = [
       0.00 0.00;
       1.00 0.20;
       2.00 0.80;
       3.00 1.40;
       4.00 0.50;
       4.50 1.40;
       4.50 3.90;
       2.50 3.90;
       1.20 3.20;
       0.00 1.50  % goal
   ];
   obs3 = [0.0 0.5 1.0 1.0 1.0 1.5 2.0 2.5 2.5 2.5 2.5 3.0 3.5 4.0 4.0 4.0 4.0 4.0 4.0 0.0 0.5 1.0 2.5 2.5;
           1.0 1.0 1.0 1.5 2.0 2.0 2.0 2.0 2.5 3.0 3.5 3.5 3.5 3.5 3.0 2.5 2.0 1.5 1.0 3.5 3.5 3.5 0.0 0.5];

   allVia = {via1, via2, via3};
   allObs = {obs1, obs2, obs3};
   titles = {'Course 1','Course 2','Course 3'};

   %% ------------- Precompute smooth paths for all courses --------------
   fprintf('Generating smooth paths for all 3 courses...\n');
   allPaths = cell(1,3);   % each: [x, y, yaw]
   allGoals = zeros(3,2);  % each row: [x_goal, y_goal];

   for i = 1:3
       via  = allVia{i};
       path = generate_dubins_from_via(via, R_min, ds); % N×3: [x y yaw]
       allPaths{i}  = path;
       allGoals(i,:) = via(end,:);  % last via-point is goal
   end

   %% ------------- Optional offline visualization -----------------------
   figure(10); clf;
   for i = 1:3
       subplot(1,3,i); hold on; grid on; axis equal;
       via  = allVia{i};
       obs  = allObs{i};
       path = allPaths{i};

       plot(obs(1,:), obs(2,:), 'ko', 'MarkerFaceColor','k', 'MarkerSize',8);
       plot(via(:,1), via(:,2), 'c--o', 'LineWidth',1.2,'MarkerSize',4);
       plot(path(:,1), path(:,2), 'r-', 'LineWidth',2);
       plot(via(1,1), via(1,2), 'go','MarkerSize',8,'MarkerFaceColor','g');
       plot(via(end,1), via(end,2), 'bx','MarkerSize',10,'LineWidth',2);
       xlabel('X [m]'); ylabel('Y [m]');
       title(titles{i});
       legend('Obstacles','Via-points','Path','Start','Goal','Location','best');
   end
   drawnow;
   fprintf('Precomputed paths visualized.\n');

   %% ------------- Connect to MQTT and mocap ----------------------------
   mq = [];
   try
       disp('Trying MQTT: mqtt://rasticvm.lan (default port)...');
       mq = mqttclient("mqtt://rasticvm.lan");
       disp('Success.');
   catch
       disp('Failed. Trying mqtt://rasticvm.lan:1883 ...');
       try
           mq = mqttclient("mqtt://rasticvm.lan","Port",1883);
           disp('Success.');
       catch ME
           error('MQTT connection failed: %s', ME.message);
       end
   end

   limoID   = "limo777";
   posTopic = "rb/" + limoID + "/pos";
   rotTopic = "rb/" + limoID + "/rot";
   cmdTopic = "rb/" + limoID + "/cmd_vel";

   subscribe(mq, posTopic);
   subscribe(mq, rotTopic);
   pause(0.5); % allow some messages to arrive

   % Get initial pose
   pose = getMocapPose_simple(mq, limoID);
   if any(isnan(pose))
       warning('Initial mocap pose not valid, continuing anyway.');
   else
       fprintf('Initial pose from mocap: [%.2f, %.2f, %.2f rad]\n', pose(1), pose(2), pose(3));
   end

   %% ------------- Precompute feedforward omega for each path -----------
   allOmegaFF = cell(1,3);
   for ci = 1:3
       p = allPaths{ci};
       N = size(p,1);
       omega_ff = zeros(N,1);
       for k = 1:N-1
           dx  = p(k+1,1) - p(k,1);
           dy  = p(k+1,2) - p(k,2);
           ds_k = hypot(dx,dy);
           if ds_k < 1e-4
               ds_k = ds;
           end
           dtheta = angle_mod_local(p(k+1,3) - p(k,3));
           dt_k   = ds_k / v_ff;
           omega_ff(k) = dtheta / dt_k;
       end
       omega_ff(N) = omega_ff(N-1);
       allOmegaFF{ci} = omega_ff;
   end

   %% ------------- Live tracking: follow Course 1 → 2 → 3 ---------------
   figure(20); clf; hold on; grid on; axis equal;
   xlabel('X [m]'); ylabel('Y [m]');
   title('LIMO live tracking of 3 paths');

   colors = {'r','g','b'};
   for ci = 1:3
       path  = allPaths{ci};
       goal_xy  = allGoals(ci,:);
       plot(path(:,1), path(:,2), '-', 'Color', colors{ci}, 'LineWidth',1.5);
       plot(goal_xy(1), goal_xy(2), 'kx','MarkerSize',10,'LineWidth',2);
   end
   legend('Course1 path','Course1 goal','Course2 path','Course2 goal','Course3 path','Course3 goal','Location','best');
   drawnow;

   fprintf('Starting path tracking for all 3 courses...\n');

   for ci = 1:3
       fprintf('\n=== Starting Course %d ===\n', ci);
       path     = allPaths{ci};      % [x,y,yaw]
       omega_ff = allOmegaFF{ci};    % feedforward ω
       goal_xy  = allGoals(ci,:);
       i = 1;
       N = size(path,1);

       while true
           t_loop_start = tic;

           % --- get current pose from mocap ---
           pose = getMocapPose_simple(mq, limoID);
           if any(isnan(pose))
               warning('Mocap pose NaN, sending zero command.');
               v_cmd = 0; omega_cmd = 0;
               publish(mq, cmdTopic, sprintf('%.3f,%.3f', v_cmd, omega_cmd));
               pause(dt_min);
               continue;
           end
           x = pose(1); y = pose(2); theta = pose(3);

           % --- check if goal reached ---
           dist_goal = hypot(x - goal_xy(1), y - goal_xy(2));
           if dist_goal < goal_tol
               fprintf('Course %d complete. Distance to goal: %.3f m\n', ci, dist_goal);
               break;
           end

           % --- clamp path index to [1, N-1] ---
           if i >= N
               i = N-1;
           end

           xr    = path(i,1);  yr    = path(i,2);  thetar = path(i,3);
           w_ff  = omega_ff(i);

           % --- tracking error in reference frame ---
           dx = x - xr;
           dy = y - yr;
           ex =  cos(thetar)*dx + sin(thetar)*dy;
           ey = -sin(thetar)*dx + cos(thetar)*dy;
           eth = angle_mod_local(theta - thetar);

           % --- feedforward + feedback control law ---
           v_cmd = v_ff * cos(eth) + kx * ex;
           w_cmd = w_ff + ky * v_ff * ey + ktheta * eth;

           % --- saturate ---
           v_cmd = max(min(v_cmd,  v_max), -v_max);
           w_cmd = max(min(w_cmd, omega_max), -omega_max);

           % --- send command over MQTT ---
           cmdMsg = sprintf('%.3f,%.3f', v_cmd, w_cmd);
           publish(mq, cmdTopic, cmdMsg);

           % --- plot live pose ---
           plot(x, y, 'mo', 'MarkerSize',4, 'MarkerFaceColor','m');
           drawnow limitrate;

           % advance along path
           i = i + 1;

           % maintain approx time step
           dt = max(ds / max(v_ff,0.05), dt_min);
           elapsed = toc(t_loop_start);
           pause(max(dt - elapsed, 0));
       end

       % stop robot at end of course
       publish(mq, cmdTopic, sprintf('%.3f,%.3f', 0, 0));
       fprintf('Stopped at end of Course %d.\n', ci);
       pause(1.0);
   end

   fprintf('\nAll 3 courses completed (script side). Check behavior in the field.\n');
end

%% ========================================================================
%                     Helper: Smooth path from via-points
% ========================================================================
function path = generate_dubins_from_via(via, R, ds)
   % We approximate a Dubins-like path using a cubic spline through via-points.
   % Output: path(:,1) = x, path(:,2) = y, path(:,3) = yaw.

   n = size(via,1);
   t  = 1:n;

   % number of samples: ~1/ds per segment, at least 10
   samplesPerSeg = max(ceil(1/ds), 10);
   ts = linspace(1, n, (n-1)*samplesPerSeg + 1);

   xs = spline(t, via(:,1), ts);
   ys = spline(t, via(:,2), ts);

   dx = gradient(xs);
   dy = gradient(ys);
   yaws = atan2(dy, dx);

   path = [xs(:), ys(:), yaws(:)];
end

%% ========================================================================
%                     Helper: Simple Mocap Pose Read
% ========================================================================
function pose = getMocapPose_simple(mq, limoID)
   x = NaN; y = NaN; theta = NaN;
   posTopic = "rb/" + limoID + "/pos";
   rotTopic = "rb/" + limoID + "/rot";

   for i = 1:10
       try
           m = read(mq, 'Timeout', 0.05);
           if isempty(m)
               continue;
           end
           fn = m.Properties.VariableNames;
           topic   = string(m.(fn{1}){1});
           payload = string(m.(fn{2}){1});

           if topic == posTopic
               nums = str2double(strsplit(strrep(strrep(payload,"[",""),"]",""), ","));
               if numel(nums) >= 2
                   x = nums(1);
                   y = nums(2);
               end
           elseif topic == rotTopic
               nums = str2double(strsplit(strrep(strrep(payload,"[",""),"]",""), ","));
               if numel(nums) == 4
                   qw = nums(1); qx = nums(2); qy = nums(3); qz = nums(4);
                   theta = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
               elseif numel(nums) == 3
                   theta = nums(3); % assume yaw
               end
           end

           if ~isnan(x) && ~isnan(theta)
               break;
           end
       catch
           % ignore and retry
       end
   end

   pose = [x; y; theta];
end

%% ========================================================================
%                     Small angle helpers
% ========================================================================
function out = angle_mod_local(x)
   out = mod(x + pi, 2*pi) - pi;
end


