clear; close all; clc;

%% ROS2 NETWORK CONFIGURATION
setenv('ROS_DOMAIN_ID', '0');
setenv('ROS_LOCALHOST_ONLY', '0');

% Restart daemon for clean connection
system('ros2 daemon stop');
pause(1);
system('ros2 daemon start');
pause(2);

%% LOAD CALIBRATION (Full HD)
use_undistort = true;
cameraParams = [];

if exist('cameraparams3.mat', 'file')
    load('cameraparams3.mat', 'cameraParams3');
    fprintf('[OK] Full HD calibration loaded from cameraParams3.mat\n');
    fprintf('  Focal length: [%.1f, %.1f]\n', cameraParams3.FocalLength(1), cameraParams3.FocalLength(2));
    fprintf('  Principal point: [%.1f, %.1f]\n', cameraParams3.PrincipalPoint(1), cameraParams3.PrincipalPoint(2));
    fprintf('  Reprojection error: %.3f pixels\n', cameraParams3.MeanReprojectionError);
    
    if cameraParams3.ImageSize(1) == 720 && cameraParams3.ImageSize(2) == 1280
        fprintf('  Resolution matches 1280x720\n');
    else
        fprintf('  Warning: Calibration resolution %dx%d, expected 1280x720\n', ...
            cameraParams3.ImageSize(2), cameraParams3.ImageSize(1));
    end
else
    fprintf('No calibration file found. Running without undistortion\n');
    use_undistort = false;
    cameraParams = [];
end

%% PRE-CACHE CAMERA PARAMETERS
if use_undistort && exist('cameraParams3', 'var')
    fx = cameraParams3.FocalLength(1);
    fy = cameraParams3.FocalLength(2);
    cx = cameraParams3.PrincipalPoint(1);
    cy = cameraParams3.PrincipalPoint(2);
    fprintf('Camera parameters cached for fast access\n');
else
    fx = 800; fy = 800; cx = 640; cy = 360;
    fprintf('Using default camera parameters\n');
end

%% PHYSICAL CONSTANTS
MARKER_SIZE = 0.165;
SAFETY_MARGIN = 0.15;

target_input = inputdlg('Enter number of markers:', 'Goal', [1 50], {'3'});
if isempty(target_input), return; end
target_count = str2double(target_input{1});

detected_markers = struct('id', {}, 'x', {}, 'y', {}, 'confidence', {}, 'detection_count', {}); 
coverage_path = []; path_idx = 1;
bump_locations = [];

% Coverage tracking
covered_area_polygons = {};
total_covered_area = 0;
mission_polygon = [];
area_to_cover = 0;
coverage_percentage = 0;

%% ROS2 SETUP
fprintf('Creating ROS2 node...\n');
node = ros2node("/matlab_mission_node");

velPub = ros2publisher(node, "/pc_cmd_vel", "geometry_msgs/Twist", "Reliability", "reliable");

imgSub = ros2subscriber(node, "/oakd/rgb/image_raw/compressed", "sensor_msgs/CompressedImage", ...
    "Reliability", "besteffort");

depthSub = ros2subscriber(node, "/oakd/stereo/image_raw", "sensor_msgs/Image", ...
    "Reliability", "besteffort");

odomSub = ros2subscriber(node, "/pc_odom", "nav_msgs/Odometry", ...
    "Reliability", "reliable", "Durability", "volatile");

bumperSub = ros2subscriber(node, "/bumper_hit", "std_msgs/Bool", ...
    "Reliability", "reliable");

msg = ros2message(velPub);
rate = rateControl(10);

fprintf('ROS2 node created (Full HD mode)\n');

%% GUI SETUP
fig = figure('Name', 'TB4 Coverage System', ...
    'Color', [0.08 0.08 0.12], 'Position', [50 50 1600 900], ...
    'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'figure');

mainPanel = uipanel('Parent', fig, 'Position', [0 0 1 1], ...
    'BackgroundColor', [0.08 0.08 0.12], 'BorderType', 'none');

% Camera panel
camPanel = uipanel('Parent', mainPanel, 'Position', [0.005 0.08 0.49 0.91], ...
    'BackgroundColor', [0.05 0.05 0.1], 'BorderType', 'line', ...
    'HighlightColor', [0.3 0.5 0.8], 'BorderWidth', 2, ...
    'Title', 'Camera Feed (Full HD)', 'TitlePosition', 'centertop', ...
    'ForegroundColor', [0.8 0.9 1], 'FontSize', 12, 'FontWeight', 'bold');

axCam = axes('Parent', camPanel, 'Position', [0.03 0.02 0.94 0.88]);
axCam.Color = [0 0 0];

% Map panel
mapPanel = uipanel('Parent', mainPanel, 'Position', [0.505 0.08 0.49 0.91], ...
    'BackgroundColor', [0.05 0.05 0.1], 'BorderType', 'line', ...
    'HighlightColor', [0.3 0.8 0.5], 'BorderWidth', 2, ...
    'Title', 'Coverage Map', 'TitlePosition', 'centertop', ...
    'ForegroundColor', [0.8 1 0.9], 'FontSize', 12, 'FontWeight', 'bold');

axMap = axes('Parent', mapPanel, 'Position', [0.08 0.08 0.84 0.82]);
hold(axMap, 'on'); grid(axMap, 'on'); axis(axMap, 'equal');
axMap.Color = [0.02 0.02 0.05];
axMap.GridColor = [0.2 0.25 0.35];
axMap.GridAlpha = 0.6;
axMap.XColor = [0.6 0.7 0.8];
axMap.YColor = [0.6 0.7 0.8];
xlabel(axMap, 'X (m)', 'Color', [0.7 0.8 0.9], 'FontSize', 10, 'FontWeight', 'bold');
ylabel(axMap, 'Y (m)', 'Color', [0.7 0.8 0.9], 'FontSize', 10, 'FontWeight', 'bold');

% Dashboard
dashboardPanel = uipanel('Parent', mainPanel, 'Position', [0.005 0.005 0.99 0.07], ...
    'BackgroundColor', [0.06 0.06 0.09], 'BorderType', 'line', ...
    'HighlightColor', [0.4 0.5 0.7], 'BorderWidth', 2);

statusBar = uipanel('Parent', mainPanel, 'Position', [0.005 0.002 0.99 0.022], ...
    'BackgroundColor', [0.04 0.04 0.06], 'BorderType', 'none');

statusText = uicontrol('Parent', statusBar, 'Style', 'text', ...
    'Position', [10 0 1550 20], 'String', 'Initializing...', ...
    'BackgroundColor', [0.04 0.04 0.06], 'ForegroundColor', [0.5 0.9 0.5], ...
    'FontSize', 10, 'HorizontalAlignment', 'left', 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% Dashboard metrics
dashboardMetrics = {
    'POS', sprintf('X:%.2f Y:%.2f', 0, 0), [20 0 180 30];
    'HEAD', '0.0 deg', [210 0 150 30];
    'STATE', 'INIT', [370 0 160 30];
    'MARK', sprintf('0/%d', target_count), [540 0 140 30];
    'SWATH', '0.00m', [690 0 130 30];
    'WP', '0/0', [830 0 130 30];
    'COV', '0.0%', [970 0 150 30];
    'SCAN', '0.00 m2', [1130 0 160 30];
    'FPS', '0', [1300 0 100 30];
    'DEPTH', '-- m', [1410 0 120 30]
};

dashboardHandles = struct();
for i = 1:size(dashboardMetrics, 1)
    uicontrol('Parent', dashboardPanel, 'Style', 'text', ...
        'Position', dashboardMetrics{i,3}, 'String', dashboardMetrics{i,1}, ...
        'BackgroundColor', [0.06 0.06 0.09], 'ForegroundColor', [0.5 0.6 0.7], ...
        'FontSize', 8, 'HorizontalAlignment', 'left', 'FontName', 'Segoe UI', 'FontWeight', 'bold');
    
    fieldName = dashboardMetrics{i,1};
    dashboardHandles.(fieldName) = ...
        uicontrol('Parent', dashboardPanel, 'Style', 'text', ...
        'Position', dashboardMetrics{i,3} + [80 0 -80 0], 'String', dashboardMetrics{i,2}, ...
        'BackgroundColor', [0.06 0.06 0.09], 'ForegroundColor', [0.8 0.9 1], ...
        'FontSize', 10, 'HorizontalAlignment', 'right', 'FontName', 'Segoe UI', 'FontWeight', 'bold');
end

% Info text on map
infoBox = annotation(fig, 'textbox', ...
    'Position', [0.52 0.92 0.46 0.05], ...
    'String', {''}, ...
    'BackgroundColor', [0 0 0 0.7], ...
    'EdgeColor', [0.3 0.7 0.3], ...
    'LineWidth', 1.5, ...
    'Color', [0.9 1 0.9], ...
    'FontSize', 9, ...
    'FontName', 'Segoe UI', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle');

odom_offset = [];
posX = 0; posY = 0; theta = 0;
current_state = 'EXPLORE_DRIVE';
search_timer = tic;

%% BUMPER PARAMETERS
BUMPER_ESCAPE_SPEED = -0.25;
BUMPER_ESCAPE_DURATION = 1.5;
BUMPER_TURN_SPEED = 1.2;
BUMPER_360_DURATION = (2*pi) / 1.2;
BUMPER_180_DURATION = pi / 1.2;
BUMPER_FWD_SPEED = 0.15;
BUMPER_FWD_DURATION = 0.8;

is_emergency = false;
emergency_start_time = 0;
emergency_phase = 1;
last_bumper_state = false;

%% CAMERA EXTRINSICS
cam_offset_x = 0.00;
cam_offset_y = -0.06;
cam_offset_z = 0.25;
cam_pitch = -0.26;

%% CAMERA CONE PARAMETERS
CAMERA_HFOV = deg2rad(69);
CAMERA_VFOV = deg2rad(54);
CAMERA_MAX_DEPTH = 4.0;
CAMERA_MIN_DEPTH = 0.3;
OPTIMAL_DETECTION_DISTANCE = 1.5;

n_segments = 30;
t = linspace(0, 2*pi, n_segments);

r_max = CAMERA_MAX_DEPTH * tan(CAMERA_HFOV/2);
cone_max_x = r_max * cos(t);
cone_max_y = r_max * sin(t);
cone_max_z = ones(size(t)) * CAMERA_MAX_DEPTH;

r_min = CAMERA_MIN_DEPTH * tan(CAMERA_HFOV/2);
cone_min_x = r_min * cos(t);
cone_min_y = r_min * sin(t);
cone_min_z = ones(size(t)) * CAMERA_MIN_DEPTH;

r_opt = OPTIMAL_DETECTION_DISTANCE * tan(CAMERA_HFOV/2);
cone_opt_x = r_opt * cos(t);
cone_opt_y = r_opt * sin(t);
cone_opt_z = ones(size(t)) * OPTIMAL_DETECTION_DISTANCE;

cone_geometry = struct(...
    'max', [cone_max_x', cone_max_y', cone_max_z'],...
    'min', [cone_min_x', cone_min_y', cone_min_z'],...
    'opt', [cone_opt_x', cone_opt_y', cone_opt_z']);

EFFECTIVE_SWATH = 2 * OPTIMAL_DETECTION_DISTANCE * tan(CAMERA_HFOV/2);

%% OTHER PARAMETERS
OBSTACLE_THRESHOLD = 0.35;
OBSTACLE_SLOWDOWN = 0.60;
DEPTH_PATCH_SIZE = 20;

frame_counter = 0;
PROCESS_EVERY_N_FRAMES = 2;
DISPLAY_SCALE = 0.35;
last_depth_process_time = tic;
LAST_PLOT_UPDATE = tic;
PLOT_UPDATE_INTERVAL = 0.3;
fps_timer = tic;
fps_counter = 0;
current_fps = 0;
dynamic_swath = EFFECTIVE_SWATH;

coverage_resolution = 0.05;
fprintf('System ready (camera cone loaded)\n');

%% WAIT FOR FIRST MESSAGES
fprintf('Waiting for camera and odometry...\n');
while isempty(imgSub.LatestMessage) || isempty(odomSub.LatestMessage)
    pause(0.1);
    if ~isempty(imgSub.LatestMessage)
        fprintf('Camera connected\n');
    end
    if ~isempty(odomSub.LatestMessage)
        fprintf('Odometry connected\n');
    end
end
fprintf('All systems ready. Starting main loop...\n');

%% MAIN LOOP
try
    while ishandle(fig)
        loop_start = tic;
        fps_counter = fps_counter + 1;
        
        if toc(fps_timer) > 2.0
            current_fps = fps_counter;
            fps_counter = 0;
            fps_timer = tic;
            set(dashboardHandles.FPS, 'String', sprintf('%d', current_fps));
        end
        
        % --- ODOMETRY ---
        if isempty(odomSub.LatestMessage)
            title(axMap, 'Waiting for odometry...');
            drawnow; continue;
        end
        
        odomMsg = odomSub.LatestMessage;
        rawX = odomMsg.pose.pose.position.x;
        rawY = odomMsg.pose.pose.position.y;
        q = odomMsg.pose.pose.orientation;
        eul = quat2eul([q.w, q.x, q.y, q.z]);
        rawTheta = eul(1);

        if isempty(odom_offset)
            odom_offset = [rawX, rawY, rawTheta];
            set(statusText, 'String', sprintf('Origin set [%.2f, %.2f] %.1f deg', rawX, rawY, rad2deg(rawTheta)));
        end

        dx_raw = rawX - odom_offset(1);
        dy_raw = rawY - odom_offset(2);
        rot = -odom_offset(3);
        posX = dx_raw * cos(rot) - dy_raw * sin(rot);
        posY = dx_raw * sin(rot) + dy_raw * cos(rot);
        theta = angdiff(odom_offset(3), rawTheta);
        
        set(dashboardHandles.POS, 'String', sprintf('X:%.2f Y:%.2f', posX, posY));
        set(dashboardHandles.HEAD, 'String', sprintf('%.1f deg', rad2deg(theta)));
        set(dashboardHandles.STATE, 'String', strrep(current_state, '_', ' '));
        set(dashboardHandles.MARK, 'String', sprintf('%d/%d', length(detected_markers), target_count));
        set(dashboardHandles.SWATH, 'String', sprintf('%.2fm', dynamic_swath));
        set(dashboardHandles.WP, 'String', sprintf('%d/%d', path_idx, size(coverage_path,1)));
        set(dashboardHandles.COV, 'String', sprintf('%.1f%%', coverage_percentage));
        set(dashboardHandles.SCAN, 'String', sprintf('%.2f m2', total_covered_area));

        % --- BUMPER CHECK ---
        bumperMsg = bumperSub.LatestMessage;
        bumper_hit = false;
        if ~isempty(bumperMsg)
            bumper_hit = logical(bumperMsg.data);
            if bumper_hit && ~last_bumper_state && ~is_emergency
                is_emergency = true;
                emergency_start_time = tic;
                emergency_phase = 1;
                bump_locations = [bump_locations; posX, posY];
                set(statusText, 'String', sprintf('Bumper hit [%.2f, %.2f]', posX, posY));
            end
            last_bumper_state = bumper_hit;
        end

        % --- EMERGENCY HANDLING ---
        if is_emergency
            elapsed = toc(emergency_start_time);
            v = 0; w = 0;
            phase_names = {'REVERSE', '360 SPIN', '180 FLIP', 'FORWARD'};

            switch emergency_phase
                case 1
                    if elapsed < BUMPER_ESCAPE_DURATION, v = BUMPER_ESCAPE_SPEED;
                    else, emergency_phase = 2; emergency_start_time = tic; end
                case 2
                    if elapsed < BUMPER_360_DURATION, w = BUMPER_TURN_SPEED;
                    else, emergency_phase = 3; emergency_start_time = tic; end
                case 3
                    if elapsed < BUMPER_180_DURATION, w = BUMPER_TURN_SPEED;
                    else, emergency_phase = 4; emergency_start_time = tic; end
                case 4
                    if elapsed < BUMPER_FWD_DURATION, v = BUMPER_FWD_SPEED;
                    else
                        is_emergency = false;
                        set(statusText, 'String', sprintf('Recovered - Resuming %s', current_state));
                    end
            end

            if is_emergency
                set(statusText, 'String', sprintf('EMERGENCY: %s (%.1fs)', phase_names{emergency_phase}, elapsed));
            end

            msg.linear.x = double(v); msg.angular.z = double(w);
            send(velPub, msg);
            title(axCam, 'EMERGENCY', 'Color', 'r', 'FontSize', 14, 'FontWeight', 'bold');
            drawnow; pause(0.05); continue;
        end

        % --- VISION ---
        frame_counter = frame_counter + 1;
        obstacle_distance = inf;
        min_left = inf;
        min_right = inf;
        
        if mod(frame_counter, PROCESS_EVERY_N_FRAMES) == 0
            imgMsg = imgSub.LatestMessage;
            
            if ~isempty(imgMsg)
                img_hd = rosReadImage(imgMsg);
                img_display = imresize(img_hd, DISPLAY_SCALE);
                img_processing = imresize(img_hd, 0.5);
                detection_scale = 2;
                
                depth_available = false;
                depth_img = [];
                if toc(last_depth_process_time) > 0.5
                    depthMsg = depthSub.LatestMessage;
                    if ~isempty(depthMsg) && strcmp(depthMsg.encoding, '16UC1')
                        try
                            depth_uint16 = typecast(depthMsg.data, 'uint16');
                            depth_img = reshape(depth_uint16, depthMsg.width, depthMsg.height)';
                            depth_img = double(depth_img) / 1000;
                            depth_img = imresize(depth_img, 0.5);
                            depth_available = true;
                        catch
                        end
                    end
                    
                    if depth_available
                        img_h = size(depth_img, 1);
                        img_w = size(depth_img, 2);
                        roi_y = round(img_h * 0.7) : img_h;
                        roi_x = round(img_w * 0.35) : round(img_w * 0.65);
                        roi = depth_img(roi_y, roi_x);
                        valid = roi(roi > 0.1 & roi < 10.0);
                        if ~isempty(valid), obstacle_distance = min(valid); end
                        
                        left = depth_img(roi_y, 1:round(img_w*0.25));
                        right = depth_img(roi_y, round(img_w*0.75):end);
                        valid_left = left(left > 0.1 & left < 10.0);
                        valid_right = right(right > 0.1 & right < 10.0);
                        if ~isempty(valid_left), min_left = min(valid_left); end
                        if ~isempty(valid_right), min_right = min(valid_right); end
                        
                        set(dashboardHandles.DEPTH, 'String', sprintf('%.2f m', obstacle_distance));
                        last_depth_process_time = tic;
                    end
                end
                
                % ArUco marker detection
                if ~strcmp(current_state, 'COVERAGE_FOLLOW')
                    [ids, corners] = readArucoMarker(img_processing, 'DICT_6X6_250');
                    
                    if ~isempty(ids)
                        if ~iscell(corners) && size(corners, 1) == 4 && size(corners, 2) == 2
                            temp_corners = corners;
                            corners = cell(1, length(ids));
                            corners{1} = temp_corners;
                        end
                        
                        for i = 1:length(ids)
                            if ~isempty(detected_markers) && any([detected_markers.id] == ids(i))
                                continue;
                            end
                            
                            try
                                if iscell(corners)
                                    pts = corners{i};
                                else
                                    pts = corners(:,:,i);
                                end
                                
                                pts = pts * detection_scale;
                                center_x = mean(pts(:,1));
                                center_y = mean(pts(:,2));
                                pixel_width = max(pts(:,1)) - min(pts(:,1));
                                
                                z_depth = (MARKER_SIZE * fx) / pixel_width;
                                x_lateral = (center_x - cx) * z_depth / fx;
                                y_vertical = (center_y - cy) * z_depth / fy;

                                base_x_dist = z_depth * cos(cam_pitch) - y_vertical * sin(cam_pitch);
                                base_y_dist = -x_lateral;
                                marker_fwd = base_x_dist + cam_offset_x;
                                marker_left = base_y_dist + cam_offset_y;

                                mx = posX + marker_fwd * cos(theta) - marker_left * sin(theta);
                                my = posY + marker_fwd * sin(theta) + marker_left * cos(theta);
                                
                                confidence = min(1.0, 0.8 + 0.2 * (0.5 / z_depth));
                                
                                det_idx = length(detected_markers) + 1;
                                detected_markers(det_idx).id = ids(i);
                                detected_markers(det_idx).x = mx;
                                detected_markers(det_idx).y = my;
                                detected_markers(det_idx).confidence = confidence;
                                detected_markers(det_idx).detection_count = 1;
                                
                                set(statusText, 'String', sprintf('Marker %d accepted [%.2f, %.2f]', ids(i), mx, my));
                                fprintf('Marker %d at (%.2f, %.2f) | Dist: %.2fm\n', ids(i), mx, my, z_depth);
                                
                            catch ME
                                fprintf('Marker processing error: %s\n', ME.message);
                                continue;
                            end
                        end
                    end
                end
                
                % Display image with marker overlay
                imshow(img_display, 'Parent', axCam);
                hold(axCam, 'on');
                
                if exist('ids', 'var') && ~isempty(ids)
                    for i = 1:length(ids)
                        if iscell(corners)
                            pts_raw = corners{i};
                        else
                            pts_raw = corners(:,:,i);
                        end
                        
                        visual_scale = (1 / 0.5) * DISPLAY_SCALE;
                        pts_draw = pts_raw * visual_scale;
                        
                        plot(axCam, pts_draw([1:4,1], 1), pts_draw([1:4,1], 2), 'g-', 'LineWidth', 2);
                        text(axCam, mean(pts_draw(:,1)), mean(pts_draw(:,2))-20, ...
                            sprintf('ID: %d', ids(i)), 'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold', ...
                            'BackgroundColor', 'k');
                    end
                end
                hold(axCam, 'off');
                drawnow update;
                
                if obstacle_distance < OBSTACLE_THRESHOLD
                    title(axCam, sprintf('OBSTACLE %.2fm!', obstacle_distance), 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
                else
                    title(axCam, sprintf('%s | FPS: %d', strrep(current_state, '_', ' '), current_fps), 'Color', [0.8 0.9 1], 'FontSize', 11);
                end
                drawnow;
            end
        end

        % --- PATH GENERATION ---
        num_found = length(detected_markers);
        elapsed = toc(search_timer);

        if num_found >= target_count && ~strcmp(current_state, 'COVERAGE_FOLLOW')
            pts = [[detected_markers.x]', [detected_markers.y]'];
            dynamic_swath = computeDynamicSwath(detected_markers, CAMERA_HFOV);
            [coverage_path, mission_polygon] = generateBoustrophedonPathWithPolygon(pts, dynamic_swath, SAFETY_MARGIN);
            
            if ~isempty(mission_polygon)
                area_to_cover = polyarea(mission_polygon(:,1), mission_polygon(:,2));
                total_covered_area = 0;
                coverage_percentage = 0;
            end
            
            if ~isempty(coverage_path)
                path_idx = 1;
                current_state = 'COVERAGE_FOLLOW';
                set(statusText, 'String', sprintf('Coverage started - Area: %.2f m2, %d waypoints', area_to_cover, size(coverage_path,1)));
            end
        end

        % --- UPDATE COVERAGE AREA ---
        current_footprint = getCameraFootprintPolygon(posX, posY, theta, ...
            cam_offset_x, cam_offset_y, cam_offset_z, cam_pitch, cone_geometry.opt);
        
        if ~isempty(current_footprint)
            covered_area_polygons{end+1} = current_footprint;
            
            try
                if length(covered_area_polygons) > 1
                    all_points = vertcat(covered_area_polygons{:});
                    if ~isempty(all_points)
                        bounds = [min(all_points(:,1)) max(all_points(:,1)); 
                                 min(all_points(:,2)) max(all_points(:,2))];
                        grid_x = bounds(1,1):coverage_resolution:bounds(1,2);
                        grid_y = bounds(2,1):coverage_resolution:bounds(2,2);
                        [X, Y] = meshgrid(grid_x, grid_y);
                        
                        covered_mask = false(size(X));
                        for p = 1:length(covered_area_polygons)
                            poly = covered_area_polygons{p};
                            if size(poly,1) > 2
                                covered_mask = covered_mask | inpolygon(X, Y, poly(:,1), poly(:,2));
                            end
                        end
                        total_covered_area = sum(covered_mask(:)) * coverage_resolution^2;
                        
                        if area_to_cover > 0
                            coverage_percentage = (total_covered_area / area_to_cover) * 100;
                            coverage_percentage = min(100, coverage_percentage);
                        end
                    end
                elseif ~isempty(covered_area_polygons)
                    total_covered_area = polyarea(current_footprint(:,1), current_footprint(:,2));
                    if area_to_cover > 0
                        coverage_percentage = (total_covered_area / area_to_cover) * 100;
                    end
                end
            catch
            end
        end

        % --- NAVIGATION ---
        v = 0; w = 0;
        obstacle_ahead = (obstacle_distance < OBSTACLE_THRESHOLD);
        
        if obstacle_ahead && ~strcmp(current_state, 'COVERAGE_FOLLOW')
            v = 0;
            if min_left > min_right, w = 0.6;
            elseif min_right > min_left, w = -0.6;
            else, w = 0.6; end
        elseif obstacle_distance < OBSTACLE_SLOWDOWN && ~strcmp(current_state, 'COVERAGE_FOLLOW')
            v = 0.06; w = 0.3;
        else
            switch current_state
                case 'EXPLORE_DRIVE'
                    v = 0.12; w = 0.0;
                    if elapsed > 5, current_state = 'EXPLORE_SPIN'; search_timer = tic; end
                case 'EXPLORE_SPIN'
                    v = 0.0; w = 0.5;
                    if elapsed > 13, current_state = 'EXPLORE_DRIVE'; search_timer = tic; end
                case 'COVERAGE_FOLLOW'
                    if path_idx > size(coverage_path, 1)
                        v = 0; w = 0;
                        set(statusText, 'String', 'Goal reached! Coverage complete.');
                        annotationText = sprintf('MISSION COMPLETE\nCoverage: %.1f%% (%.2f/%.2f m2)\nMarkers: %d/%d', ...
                            coverage_percentage, total_covered_area, area_to_cover, num_found, target_count);
                        set(infoBox, 'String', annotationText);
                    else
                        target = coverage_path(path_idx, :);
                        dx = target(1) - posX; dy = target(2) - posY;
                        dist_to_goal = sqrt(dx^2 + dy^2);
                        angle_err = atan2(sin(atan2(dy,dx)-theta), cos(atan2(dy,dx)-theta));
                        
                        if dist_to_goal < 0.25
                            path_idx = path_idx + 1;
                        else
                            v = 0.12;
                            w = min(1.0, 0.8 * abs(angle_err)) * sign(angle_err);
                        end
                    end
            end
        end

        % --- MAP UPDATE ---
        if toc(LAST_PLOT_UPDATE) > PLOT_UPDATE_INTERVAL
            cla(axMap);
            hold(axMap, 'on'); grid(axMap, 'on'); axis(axMap, 'equal');
            xlabel(axMap, 'X (m)', 'Color', [0.7 0.8 0.9], 'FontSize', 9, 'FontWeight', 'bold');
            ylabel(axMap, 'Y (m)', 'Color', [0.7 0.8 0.9], 'FontSize', 9, 'FontWeight', 'bold');
            title(axMap, sprintf('Coverage Map | Progress: %.1f%%', coverage_percentage), ...
                'Color', [0.8 1 0.8], 'FontSize', 11, 'FontWeight', 'bold');
            
            if ~isempty(mission_polygon)
                fill(axMap, mission_polygon(:,1), mission_polygon(:,2), [0.1 0.15 0.2], ...
                    'FaceAlpha', 0.3, 'EdgeColor', [0.4 0.6 0.8], 'LineWidth', 2, 'LineStyle', '--');
                text(axMap, mean(mission_polygon(:,1)), mean(mission_polygon(:,2)), ...
                    sprintf('Area: %.2f m2', area_to_cover), ...
                    'Color', [0.6 0.8 1], 'FontSize', 9, 'HorizontalAlignment', 'center', ...
                    'BackgroundColor', [0 0 0 0.5]);
            end
            
            for p = 1:length(covered_area_polygons)
                poly = covered_area_polygons{p};
                if size(poly,1) > 2
                    fill(axMap, poly(:,1), poly(:,2), [0.2 0.7 0.3], ...
                        'FaceAlpha', 0.25, 'EdgeColor', [0.3 0.9 0.4], 'LineWidth', 0.5);
                end
            end
            
            [cone_x, cone_y] = getCameraConeProjection(posX, posY, theta, ...
                cam_offset_x, cam_offset_y, cam_offset_z, cam_pitch, cone_geometry);
            
            fill(axMap, cone_x, cone_y, [0.3 0.8 0.4], 'FaceAlpha', 0.2, ...
                'EdgeColor', [0.2 0.9 0.3], 'LineWidth', 1.5);
            plot(axMap, cone_x, cone_y, 'g-', 'LineWidth', 2);
            
            n_rays = 5;
            for i = 1:n_rays
                ratio = i / n_rays;
                ray_x = [posX, cone_x(round(end*ratio))];
                ray_y = [posY, cone_y(round(end*ratio))];
                plot(axMap, ray_x, ray_y, 'Color', [0.2 0.8 0.2], 'LineWidth', 0.8, 'LineStyle', ':');
            end
            
            if ~isempty(coverage_path)
                plot(axMap, coverage_path(:,1), coverage_path(:,2), ...
                    'Color', [0.3 0.6 1], 'LineWidth', 2, 'LineStyle', '-');
                
                if path_idx > 1
                    plot(axMap, coverage_path(1:path_idx-1,1), coverage_path(1:path_idx-1,2), ...
                        'Color', [0.2 0.9 0.2], 'LineWidth', 3);
                end
                
                if path_idx <= size(coverage_path, 1)
                    plot(axMap, coverage_path(path_idx,1), coverage_path(path_idx,2), ...
                        'o', 'Color', [1 0.8 0.2], 'MarkerSize', 14, ...
                        'MarkerFaceColor', [1 0.9 0.3], 'LineWidth', 2);
                end
            end
            
            if ~isempty(detected_markers)
                for j = 1:length(detected_markers)
                    if detected_markers(j).confidence > 0.8
                        marker_color = [0.2 1 0.3];
                        face_color = [0.1 0.9 0.2];
                    elseif detected_markers(j).confidence > 0.6
                        marker_color = [1 1 0.2];
                        face_color = [0.9 0.9 0.1];
                    else
                        marker_color = [1 0.6 0.2];
                        face_color = [0.9 0.5 0.1];
                    end
                    plot(axMap, detected_markers(j).x, detected_markers(j).y, ...
                        's', 'Color', marker_color, 'MarkerSize', 18, ...
                        'MarkerFaceColor', face_color, 'LineWidth', 2);
                    text(axMap, detected_markers(j).x+0.1, detected_markers(j).y, ...
                        sprintf('%d', detected_markers(j).id), ...
                        'Color', [0.9 1 0.9], 'FontSize', 10, 'FontWeight', 'bold', ...
                        'BackgroundColor', [0 0 0 0.6]);
                end
            end
            
            if ~isempty(bump_locations)
                scatter(axMap, bump_locations(:,1), bump_locations(:,2), ...
                    100, [1 0.5 0.1], '^', 'filled', 'MarkerEdgeColor', [0.8 0.4 0]);
            end
            
            plot(axMap, posX, posY, 'o', 'Color', [0.3 0.7 1], 'MarkerSize', 16, ...
                'MarkerFaceColor', [0.2 0.5 0.8]);
            quiver(axMap, posX, posY, 0.35*cos(theta), 0.35*sin(theta), ...
                'Color', [0.5 0.8 1], 'LineWidth', 3, 'MaxHeadSize', 0.4);
            
            text(axMap, min(xlim(axMap)) + 0.5, max(ylim(axMap)) - 0.5, ...
                sprintf('Coverage: %.1f%%\nScanned: %.2f m2\nTarget: %.2f m2', ...
                coverage_percentage, total_covered_area, area_to_cover), ...
                'Color', [0.8 1 0.8], 'FontSize', 9, 'BackgroundColor', [0 0 0 0.6], ...
                'EdgeColor', [0.3 0.7 0.3], 'LineWidth', 1);
            
            all_pts_x = [posX; [detected_markers.x]'; cone_x(:)];
            all_pts_y = [posY; [detected_markers.y]'; cone_y(:)];
            if ~isempty(all_pts_x) && ~isempty(mission_polygon)
                all_pts_x = [all_pts_x; mission_polygon(:,1)];
                all_pts_y = [all_pts_y; mission_polygon(:,2)];
            end
            if ~isempty(all_pts_x)
                margin = 0.5;
                xlim(axMap, [min(all_pts_x)-margin, max(all_pts_x)+margin]);
                ylim(axMap, [min(all_pts_y)-margin, max(all_pts_y)+margin]);
            end
            
            LAST_PLOT_UPDATE = tic;
        end
        
        msg.linear.x = double(v); msg.angular.z = double(w);
        send(velPub, msg);
        drawnow limitrate;
        waitfor(rate);
    end
catch ME
    set(statusText, 'String', sprintf('ERROR: %s', ME.message));
    fprintf('Error: %s\n', ME.message);
    fprintf('Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s at line %d\n', ME.stack(i).name, ME.stack(i).line);
    end
end

% Clean shutdown
msg.linear.x = 0; msg.angular.z = 0; send(velPub, msg);
fprintf('System shutdown complete.\n');

%% HELPER FUNCTIONS

function [path, polygon] = generateBoustrophedonPathWithPolygon(marker_positions, swath_width, safety_margin)
    if isempty(marker_positions)
        path = [];
        polygon = [];
        return;
    end
    
    min_x = min(marker_positions(:,1)) - safety_margin;
    max_x = max(marker_positions(:,1)) + safety_margin;
    min_y = min(marker_positions(:,2)) - safety_margin;
    max_y = max(marker_positions(:,2)) + safety_margin;
    
    width_x = max_x - min_x;
    width_y = max_y - min_y;
    
    if width_x < 0.5
        min_x = min_x - 0.3;
        max_x = max_x + 0.3;
    end
    if width_y < 0.5
        min_y = min_y - 0.3;
        max_y = max_y + 0.3;
    end
    
    polygon = [min_x, min_y; max_x, min_y; max_x, max_y; min_x, max_y];
    
    num_passes = max(3, ceil(width_y / swath_width));
    actual_swath = width_y / num_passes;
    
    waypoints = [];
    
    for i = 1:num_passes
        y_pos = min_y + (i - 0.5) * actual_swath;
        
        if mod(i, 2) == 1
            waypoints = [waypoints; min_x - 0.2, y_pos];
            waypoints = [waypoints; max_x + 0.2, y_pos];
        else
            waypoints = [waypoints; max_x + 0.2, y_pos];
            waypoints = [waypoints; min_x - 0.2, y_pos];
        end
    end
    
    center_x = (min_x + max_x) / 2;
    center_y = (min_y + max_y) / 2;
    waypoints = [waypoints; center_x, center_y];
    
    path = waypoints;
    keep_idx = [true; any(diff(path) ~= 0, 2)];
    path = path(keep_idx, :);
end

function [cone_world_x, cone_world_y] = getCameraConeProjection(...
    posX, posY, theta, cam_offset_x, cam_offset_y, cam_offset_z, cam_pitch, cone_geo)
    
    n_points = size(cone_geo.opt, 1);
    cone_world = zeros(n_points, 2);
    
    R_cam_to_base = [1, 0, 0; 0, cos(cam_pitch), -sin(cam_pitch); 0, sin(cam_pitch), cos(cam_pitch)];
    
    for i = 1:n_points
        corner_cam = cone_geo.opt(i, :)';
        corner_base = R_cam_to_base * corner_cam + [cam_offset_x; cam_offset_y; cam_offset_z];
        cone_world(i, 1) = posX + corner_base(2) * cos(theta) - corner_base(1) * sin(theta);
        cone_world(i, 2) = posY + corner_base(2) * sin(theta) + corner_base(1) * cos(theta);
    end
    
    cone_world_x = cone_world(:,1);
    cone_world_y = cone_world(:,2);
end

function footprint = getCameraFootprintPolygon(...
    posX, posY, theta, cam_offset_x, cam_offset_y, cam_offset_z, cam_pitch, footprint_corners_cam)
    
    n_corners = size(footprint_corners_cam, 1);
    footprint = zeros(n_corners, 2);
    
    R_cam_to_base = [1, 0, 0; 0, cos(cam_pitch), -sin(cam_pitch); 0, sin(cam_pitch), cos(cam_pitch)];
    
    for i = 1:n_corners
        corner_cam = footprint_corners_cam(i, :)';
        corner_base = R_cam_to_base * corner_cam + [cam_offset_x; cam_offset_y; cam_offset_z];
        footprint(i, 1) = posX + corner_base(2) * cos(theta) - corner_base(1) * sin(theta);
        footprint(i, 2) = posY + corner_base(2) * sin(theta) + corner_base(1) * cos(theta);
    end
end

function swath = computeDynamicSwath(markers, cameraHFOV)
    if isempty(markers)
        swath = 0.5;
        return;
    end
    
    positions = [[markers.x]', [markers.y]'];
    
    if size(positions, 1) == 1
        swath = 0.6;
        return;
    end
    
    centroid = mean(positions, 1);
    distances = vecnorm(positions - centroid, 2, 2);
    marker_spread = max(distances);
    
    if isfield(markers, 'confidence')
        confidences = [markers.confidence]';
        weighted_spread = sum(distances .* confidences) / sum(confidences);
        marker_spread = weighted_spread;
    end
    
    swath = max(0.4, min(1.2, marker_spread * 0.6));
end