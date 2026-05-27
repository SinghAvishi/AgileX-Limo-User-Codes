%% AgileX Limo Line Following Robot
% Combines path detection with Limo steering control for
autonomous line following
% NO VISUALIZATION - Maximum performance mode
clear all;
close all;
clc;
%% Initialize TCP/IP Connection to Limo
limoAddress = '192.168.1.101'; % Set this to the Limo's IP
address
limoPort = 12345; % Do not change
% Create a TCP/IP client
tcpipClient = tcpip(limoAddress, limoPort, 'NetworkRole','client');
% Open the connection
fopen(tcpipClient);
disp('Connected to Limo rover');
%% Initialize Camera
cam = ipcam('http://192.168.1.101:8000/video_stream');
disp('IP camera connected');
%% Vehicle Parameters
maxLinearVel = 0.15; % m/s (reduced for safer line following)
maxSteeringAngle = 0.3; % radians (max steering angle)
commandRate = 0.001; % seconds between commands (much faster response)
%% Control Parameters
baseSpeed = 0.05; % Base forward speed (significantly
reduced to prevent overshooting)
velocitySign = 1; % Set to 1 or -1 to change forward direction (1 = forward for this robot)
steeringGain = 1.5; % Steering sensitivity (increased from 1 for more aggressive centering)
bottomRegionRatio = 0.18; % Focus on bottom 18% of image (very
close to robot)
straightBias = 0.1; % Small bias - prioritize turning over
going straight (reduced from 0.5)
leftTurnBias = 0.5; % Very strong bias toward left turns (increased from 0.3)
centeringAggressiveness = 0.3; % Additional correction when offcenter (0-1) of image (very close to robot - reduced from 0.4)
straightBias = 0.2; % Bias toward going straight (increased
from 0.15)
leftTurnBias = 0.1; % Slight bias toward left turns at Tintersections
%% Main Control Loop
try
 disp('Starting line following (NO DISPLAY MODE)... Press Ctrl+C to stop');
 pause(2);

 loopCount = 0;
 inRecoveryMode = false;
 lineFirstLostTime = 0;
 recoveryStartTime = 0;

 while true
 loopCount = loopCount + 1;

 % Get steering command from path detection (no visualization)
 [steeringAngle, lineDetected] = path_finder_control(cam, steeringGain, bottomRegionRatio, straightBias, leftTurnBias,centeringAggressiveness);

 % Track when line was first lost
 if lineDetected
 lineFirstLostTime = 0; % Reset timer when line is
found
 if inRecoveryMode
 disp('RECOVERY SUCCESS: Line found! Resuming normal operation.');
 inRecoveryMode = false;
 end
 else
 % Line not detected
 if lineFirstLostTime == 0
 lineFirstLostTime = tic; % Start timer when line
first lost
 fprintf('Line lost at loop %d, starting timer \n', loopCount);
 end
 end

 % Check if we need to enter recovery mode
 if lineFirstLostTime > 0 && ~inRecoveryMode
 timeWithoutLine = toc(lineFirstLostTime);
 if timeWithoutLine > 2.0
 inRecoveryMode = true;
 recoveryStartTime = tic;
 fprintf('RECOVERY MODE: Line lost for %.1f seconds, backing up...\n', timeWithoutLine);
 end
 end

 % Control logic
 if inRecoveryMode
 % Recovery mode: back up until line is found
 linearVel = -0.05; % Negative for backward (adjusted
based on your velocitySign)
 steering = 0; % Go straight back

 % Safety timeout: stop backing up after 5 seconds
 if toc(recoveryStartTime) > 5.0
 fprintf('RECOVERY TIMEOUT: Stopping after %.1f seconds of backing up.\n', toc(recoveryStartTime));
 linearVel = 0;
 steering = 0;
 inRecoveryMode = false;
 lineFirstLostTime = 0; % Reset so it can try
again
 end

 % Print recovery status
 if mod(loopCount, 100) == 0
 fprintf('Loop %d | RECOVERY MODE | Backing up Time: %.1fs\n', loopCount, toc(recoveryStartTime));
 end

 elseif lineDetected
 % Normal mode: follow the line
 linearVel = baseSpeed * velocitySign;
 steering = max(min(steeringAngle, maxSteeringAngle),-maxSteeringAngle);
 else
 % Line just lost but not yet in recovery: stop and
wait
 linearVel = 0;
 steering = 0;
 if mod(loopCount, 100) == 0 && lineFirstLostTime > 0
 fprintf('Loop %d | Waiting... | No line for:')
%.1fs\n', loopCount, toc(lineFirstLostTime));
 end
 end

 % Print normal status less frequently
 if mod(loopCount, 500) == 0 && ~inRecoveryMode &&lineDetected
 fprintf('Loop %d | Line: %d | Speed: %.2f | Steering: .3f\n', loopCount, lineDetected, linearVel, steering);
 end

 % Send command to Limo (single write for maximum speed)
 command = sprintf('%f,%f', linearVel, steering);

 if strcmp(tcpipClient.Status, 'open')
 fwrite(tcpipClient, command);
 else
 fprintf('ERROR: TCP connection not open! Status:')
%s\n', tcpipClient.Status);
 end

 pause(commandRate);
 end

catch ME
 % Error handling
 disp('Error occurred:');
 disp(ME.message);
end
%% Cleanup
disp('Stopping robot and closing connections...');
% Stop the robot
command = sprintf('%f,%f', 0.0, 0.0);
for i=1:5
 fwrite(tcpipClient, command);
 pause(0.1);
end
% Close connections
fclose(tcpipClient);
delete(tcpipClient);
clear tcpipClient;
clear cam;
disp('Cleanup complete');
%% Path Detection and Control Function (NO VISUALIZATION)
function [steeringAngle, lineDetected] = path_finder_control(cam,steeringGain, bottomRegionRatio, straightBias, leftTurnBias,centeringAggressiveness)
 % Initialize outputs
 steeringAngle = 0;
 lineDetected = false;

 % --- Get frame ---
 frame = snapshot(cam);
 [height, width, ~] = size(frame);

 % --- FOCUS ON BOTTOM REGION (nearest to robot) ---
 bottomRowStart = round(height * (1 - bottomRegionRatio));
 frameROI = frame(bottomRowStart:end, :, :); % Crop to bottom
region only
 [roiHeight, roiWidth, ~] = size(frameROI);

 % Lighter Gaussian blur for speed
 frameSmooth = imgaussfilt(frameROI, 0.8);

 % --- Detect blue color with more robust thresholds ---
 R = frameSmooth(:,:,1);
 G = frameSmooth(:,:,2);
 B = frameSmooth(:,:,3);

 % More aggressive blue detection
 blueMask = (B > 80) & (B > R + 30) & (B > G + 30) & (R < 150)& (G < 150);

 % --- Streamlined morphological cleanup ---
 blueMask = bwareaopen(blueMask, 250); % Remove small noise
 blueMask = imclose(blueMask, strel('disk', 3)); % Close gaps(smaller kernel)

 % --- Find connected blue regions ---
 stats = regionprops(blueMask, 'Area', 'Centroid','MajorAxisLength',  'MinorAxisLength', 'Orientation');

 if isempty(stats)
 return; % No blue detected at all
 end

 % --- Filter for line-like regions ---
 areas = [stats.Area];
 aspectRatios = [stats.MajorAxisLength] ./[stats.MinorAxisLength];
 centroids = vertcat(stats.Centroid);

 % Accept regions that are:
 % 1. Large enough (area > 400 pixels, reduced threshold)
 % 2. Line-like (aspect ratio > 1.5)
 validLines = (areas > 400) & (aspectRatios > 1.5);
 stats = stats(validLines);

 if isempty(stats)
 return; % No valid line found
 end

 % --- Pick the CLOSEST line (highest Y value = closest to robot) ---
 centroids = vertcat(stats.Centroid);
 [~, idx] = max(centroids(:, 2)); % Y-coordinate: higher =
closer to bottom = closer to robot
 mainRegion = stats(idx);

 % --- Extract geometry ---
 centroid = mainRegion.Centroid;
 theta = deg2rad(-mainRegion.Orientation);
 L = mainRegion.MajorAxisLength;

 % --- Compute endpoints ---
 x1 = centroid(1) - (L/2) * cos(theta);
 y1 = centroid(2) - (L/2) * sin(theta);
 x2 = centroid(1) + (L/2) * cos(theta);
 y2 = centroid(2) + (L/2) * sin(theta);

 % --- Determine which is nearer (bottom = closer to robot)---
 if y1 > y2
 startPt = [x1, y1];
 endPt = [x2, y2];
 else
 startPt = [x2, y2];
 endPt = [x1, y1];
 end

 % --- Calculate steering control with aggressive centering---
 % Image center (using ROI width)
 imageCenterX = roiWidth / 2;

 % Use BOTH the endpoint (far point) AND centroid for better
centering
 targetX = endPt(1);
 centroidX = centroid(1);

 % Calculate error from endpoint (primary steering target)
 errorX = targetX - imageCenterX;

 % Calculate centroid offset (secondary centering correction)
 centroidError = centroidX - imageCenterX;

 % Combine both errors: endpoint for direction + centroid for
centering
 combinedError = errorX + (centroidError *centeringAggressiveness);

 % Normalize error by image width
 normalizedError = combinedError / (roiWidth / 2);

 % --- Apply biases for intersection behavior ---
 % Small straight bias: only go straight if very well centered
 % This prioritizes turning at intersections
 if abs(normalizedError) < straightBias
 normalizedError = 0; % Force straight only if very
centered
 else
 % When not centered, apply aggressive turning
 % Amplify the error to encourage turning at intersections
 if abs(normalizedError) > straightBias
 excessError = normalizedError - sign(normalizedError)* straightBias;
 normalizedError = sign(normalizedError) *straightBias + excessError * 1.5; % Increased from 1.3
 end
 end

 % Strong left turn bias: significantly prefer left over right
 % If turning left (negative error = line to left), amplify it
more
 % If turning right (positive error = line to right), reduce
it
 if normalizedError < 0
 % Left turn - amplify to prefer it
 normalizedError = normalizedError * (1 + leftTurnBias);
 elseif normalizedError > 0
 % Right turn - reduce to avoid it unless necessary
 normalizedError = normalizedError * (1 - leftTurnBias *0.3);
 end

 % Calculate steering angle with adjustable gain
 steeringAngle = -normalizedError * steeringGain;

 % Clamp to reasonable values
 steeringAngle = max(min(steeringAngle, 0.3), -0.3);

 % Line detected successfully
 lineDetected = true;
end
