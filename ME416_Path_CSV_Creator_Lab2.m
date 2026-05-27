function make_csv_from_visualizer_paths()
% ========================================================================
%   Create CSV files for the 3 Dubins-like spline paths used in:
%
%       limo_follow_three_dubins_paths.m
%
%   This script:
%       1. Defines the same via-points
%       2. Generates the same paths using generate_dubins_from_via()
%       3. Writes:
%           course1_path.csv
%           course2_path.csv
%           course3_path.csv
%       4. Also writes a combined all_paths.csv:
%              [course_id, x, y, yaw]
%
% ========================================================================

clc; close all;

%% ---------------- Parameters ----------------
R_min = 0.3;   % min radius
ds    = 0.02;  % sampling resolution

%% ---------------- Via-points (EXACT COPIES) ----------------

% === Course 1 ===
via1 = [
    0.00  0.00;
    0.20  0.90;
    0.60  2.00;
    1.40  3.00;
    2.50  3.50;
    3.00  2.50;
    3.70  1.20;
    4.40  2.80;
    4.60  4.50
];

% === Course 2 ===
via2 = [
    0.00 0.00;
    0.50 0.80;
    1.30 1.40;
    2.50 1.90;
    3.80 2.60;
    4.50 3.50;
    5.00 4.50
];

% === Course 3 ===
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
    0.00 1.50
];

allVia  = {via1, via2, via3};

%% ---------------- Generate Paths ----------------
fprintf("Generating Dubins-like spline paths...\n");

allPaths = cell(1,3);

for ci = 1:3
    allPaths{ci} = generate_dubins_from_via(allVia{ci}, R_min, ds); % Nx3 [x y yaw]
    fprintf("  Course %d → %d points\n", ci, size(allPaths{ci},1));
end

%% ---------------- Write Individual CSV Files ----------------
fprintf("Saving paths to individual CSV files...\n");

writematrix(allPaths{1}, "course1_path.csv");
writematrix(allPaths{2}, "course2_path.csv");
writematrix(allPaths{3}, "course3_path.csv");

fprintf("Saved:\n  course1_path.csv\n  course2_path.csv\n  course3_path.csv\n");

%% ---------------- Write Combined all_paths.csv ----------------
combined = [];

for ci = 1:3
    pts = allPaths{ci};
    cid = ci * ones(size(pts,1),1);
    combined = [combined; cid, pts];
end

writematrix(combined, "all_paths.csv");

fprintf("Saved combined file → all_paths.csv (columns: course_id, x, y, yaw)\n");

end

%% ========================================================================
%      Helper from your original script (unchanged)
%% ========================================================================
function path = generate_dubins_from_via(via, R, ds)

   n = size(via,1);
   t  = 1:n;

   samplesPerSeg = max(ceil(1/ds), 10);
   ts = linspace(1, n, (n-1)*samplesPerSeg + 1);

   xs = spline(t, via(:,1), ts);
   ys = spline(t, via(:,2), ts);

   dx = gradient(xs);
   dy = gradient(ys);

   yaws = atan2(dy, dx);

   path = [xs(:), ys(:), yaws(:)];

end
