close all;

[image_sub, velocity_pub, laser_sub] = initTurtleBot();

spinVelocity = 0.5;       % Angular velocity (rad/s)
forwardVelocity = 0.0;    % Linear velocity (m/s)

% TODO: check the resolution setting of the rpi camera
totalNumberOfImageRows = 480;
totalNumberOfIImageColumns = 640;

rowNumberForBeginningOfGroundPlaneInImage = 220;

% Find the white line and move to it so the robot is over top of it
velocityMessageSent = false;
while 1
    
    % Rotate robot
    if velocityMessageSent == false
        moveTurtleBot(velocity_pub, spinVelocity, forwardVelocity)
        velocityMessageSent = true;
    end
    

    cameraImage = getCameraImage(image_sub);
    
    [horizontalLineFound, lineMidpoint] = findHorizontalLine(cameraImage);
    
    if horizontalLineFound
        stopTurtleBot(velocity_pub);
        
        orientCameraTowardsPoint(lineMidpoint, velocity_pub);
        cameraImage = getCameraImage(image_sub);
        figure(6); imshow(cameraImage); title('image after rotation');
        
        moveTurtleBotToPoint(lineMidpoint, velocity_pub)
        break;
    end
end

% Follow the white line, while avoiding obstacles, and looking for the
% yellow poles
velocityMessageSent = false;
while 1
    % Rotate robot
    if velocityMessageSent == false
        moveTurtleBot(velocity_pub, spinVelocity, forwardVelocity)
        velocityMessageSent = true;
    end
    
    cameraImage = getCameraImage(image_sub);
    
    [verticalLineFound, furthestPointOnLine] = findVerticalLine(cameraImage);
    
    if verticalLineFound
        stopTurtleBot(velocity_pub);
        
        orientCameraTowardsPoint(furthestPointOnLine, velocity_pub);
        cameraImage = getCameraImage(image_sub);
        figure(12); imshow(cameraImage); title('image after rotation');
        
        moveTurtleBotToPoint(furthestPointOnLine, velocity_pub)
        break;
    end
    
end


deinitTurtleBot(velocity_pub);

function stopTurtleBot(velocity_pub)
    velocity_msg = rosmessage(velocity_pub);
    %stop the robot forward/backward movement
    velocity_msg.Linear.X = 0;
    velocity_msg.Angular.Z = 0;
    send(velocity_pub,velocity_msg);
end

function deinitTurtleBot(velocity_pub)
    stopTurtleBot(velocity_pub);
    rosshutdown;
end

function [image_sub, velocity_pub, laser_sub] = initTurtleBot()
    % Connect to Turtlebot
    % Connect to an External ROS Master
    % ip address of TurtleBot and Matlab, replace these values accordingly
    ip_TurtleBot = '141.215.204.232';    
    ip_Matlab = '141.215.217.75';
    
    setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
    setenv('ROS_IP', ip_Matlab)

    rosinit(ip_TurtleBot)

    %%% topics we are interested
    TurtleBot_Topic.vel = '/cmd_vel';
    TurtleBot_Topic.laser = '/scan';
    TurtleBot_Topic.picam = '/raspicam_node/image/compressed';
    TurtleBot_Topic.odom = '/odom';
    TurtleBot_Topic.imu = '/imu';

    % Set up the robot camera
    if ismember(TurtleBot_Topic.picam, rostopic('list'))
        image_sub = rossubscriber(TurtleBot_Topic.picam);
    end

    % Set up the robot control for rotating, moving forward and backwards
    if ismember(TurtleBot_Topic.vel, rostopic('list'))
        velocity_pub = rospublisher(TurtleBot_Topic.vel, 'geometry_msgs/Twist');
    end
    
    %%% Subscribe to the robot lidar data
    if ismember(TurtleBot_Topic.laser, rostopic('list'))
        laser_sub = rossubscriber(TurtleBot_Topic.laser);
    end
end

function moveTurtleBot(velocity_pub, spinVelocity, forwardVelocity)
    velocity_msg = rosmessage(velocity_pub);
    velocity_msg.Angular.Z = spinVelocity;
    velocity_msg.Linear.X = forwardVelocity;
    send(velocity_pub, velocity_msg);
end

function moveTurtleBotToPoint(point, velocity_pub)
    x_velocity = .1; % .1 meters/second
    % Move the robot to the line
    % forumala for distance is is z = y*focal_length/y'
    % y = distance from middle of camera lens to ground in meters
    % y' = |object row in image - center row of image| * rowsize
    % rowsize = height of image sensor/ number of rows in image (rpiMaxImageHeight)
    focal_length = 3.04e-3; % From specification
    y = 0.1158; % distance from middle of camera from ground TODO: Find correct value for this
    heightOfImageSensor = 2.76e-3; % From specification
    rowsize = heightOfImageSensor/totalNumberOfImageRows;
    y_prime = abs(point(2) - totalNumberOfImageRows/2) * rowsize;
    distanceToLine = (y*focal_length)/y_prime; 

    timeInSecondsToMoveForward = distanceToLine/x_velocity;
    disp('distance to line'); disp(distanceToLine);
    disp('timeInSecondsToMoveForward'); disp(timeInSecondsToMoveForward);

    % Move the robot for a certain amount of time to go the calculated
    % distance to the point
    messageSent = false;
    tic;
    while toc < timeInSecondsToMoveForward
        if messageSent == false
            moveTurtleBot(velocity_pub, 0, x_velocity)
            messageSent = true;
        end
    end
    
    stopTurtleBot(velocity_pub);
end

function [cameraImage] = getCameraImage(image_sub)
    % Read an image in from a camera
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    cameraImage = readImage(image_compressed);
end

function [horizontalLineFound, lineMidpoint] = findHorizontalLine(cameraImage)

    bw = rgb2gray(cameraImage); % Convert color to gray scale image
    bw_ground = bw(rowNumberForBeginningOfGroundPlaneInImage:end,:); % Only keep the image of the ground by using row operations on the array
    bwth = imbinarize(bw_ground, 0.6); % Binary image obtained by thresholding RGB
    % TODO: Determine if we should use sobel with horizontal orientation of
    % the edges to detect
    bwth_Canny = edge(bwth,'Canny'); % Detect edges using Canny
    [H,T,R] = hough(bwth_Canny,'RhoResolution',0.5,'ThetaResolution',0.5);
    numpeaks = 1; %Specify the number of peaks
    P  = houghpeaks(H,numpeaks);
    
    % Find the lines in the image
    % TODO: Determine if minlength should be longer for a horizontal line
    lines = houghlines(bwth_Canny,T,R,P,'FillGap',50,'MinLength',9); 
    
    % If we find a white line in the image
    if ~isempty(lines)
        figure(1); imshow(bwth_Canny); title('bw threshold canny'); hold on;
        max_len = 0;
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            
            plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
            plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
            plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

            % Determine the endpoints of the longest line segment
            len = norm(lines(k).point1 - lines(k).point2);
            if ( len > max_len)
               max_len = len;
               xy_long = xy;
               % Find the midpoint of the line which is the middle of the
               % line in camera
               midpoint = [((xy_long(1,1) + xy_long(2,1))/2), ((xy_long(1,2) + xy_long(2,2))/2)];
               plot(midpoint(1),midpoint(2),'x','LineWidth',2,'Color','blue');
            end
        end
        
        figure(2); imshow(cameraImage); title('original image');
        figure(3); imshow(bw); title('black and white');
        figure(4); imshow(bw_ground); title('bw ground plane');
        figure(5); imshow(bwth); title('bw threshold');
        
        horizontalLineFound = true;
        lineMidpoint = [midpoint(1), midpoint(2) + rowNumberForBeginningOfGroundPlaneInImage];
    else
        horizontalLineFound = false;
        lineMidpoint = [0,0];
    end   
end

function [verticalLineFound, lineMidpoint] = findVerticalLine(cameraImage)
    bw = rgb2gray(cameraImage); % Convert color to gray scale image
    bw_ground = bw(rowNumberForBeginningOfGroundPlaneInImage:end,40:600); % Only keep the image of the ground and an narrow horizontal field of view
    bwth = imbinarize(bw_ground, 0.6); % Binary image obtained by thresholding RGB
    bwth_Sobel = edge(bwth,'Sobel', 'vertical'); % Detect edges using Canny
    [H,T,R] = hough(bwth_Sobel,'RhoResolution',0.5,'ThetaResolution',0.5);
    numpeaks = 1; %Specify the number of peaks
    P  = houghpeaks(H,numpeaks);
    
    % Find the lines in the image
    lines = houghlines(bwth_Sobel,T,R,P,'FillGap',50, 'MinLength', 50); 
    
     if ~isempty(lines)
        figure(7); imshow(bwth_Sobel); title('bw threshold canny'); hold on;
        max_len = 0;
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            
            plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
            plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
            plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

            % Determine the endpoints of the longest line segment
            len = norm(lines(k).point1 - lines(k).point2);
            if ( len > max_len)
               max_len = len;
               xy_long = xy;
               % Find the midpoint of the line
               midpoint = [((xy_long(1,1) + xy_long(2,1))/2), ((xy_long(1,2) + xy_long(2,2))/2)];
               plot(midpoint(1), midpoint(2), 'x', 'LineWidth', 2, 'Color', 'blue');
            end
        end
        
        figure(8); imshow(cameraImage); title('original image');
        figure(9); imshow(bw); title('black and white');
        figure(10); imshow(bw_ground); title('bw ground plane');
        figure(11); imshow(bwth); title('bw threshold');
     
        verticalLineFound = true;
        lineMidpoint = [midpoint(1) + 40, midpoint(2) + rowNumberForBeginningOfGroundPlaneInImage];
     else
        verticalLineFound = false;
        lineMidpoint = [0,0];
     end
 end
    
 function orientCameraTowardsPoint(point, velocity_pub)
    FoV = 62.2; % Horizontal field of view in degrees
    degreesPerPixel = FoV/totalNumberOfIImageColumns; % degrees/pixel
    angleToRotateToCenterMidpointInCameraImage = (point(1) - totalNumberOfIImageColumns/2) * degreesPerPixel;

    radiansToCenterPointInCameraView = deg2rad(abs(angleToRotateToCenterMidpointInCameraImage));
    rotationSpeed = .1; % .1 radians/second
    % speed = distance/time, time = distance/speed
    timeInSecondsToRotate = radiansToCenterPointInCameraView/rotationSpeed;
    disp('timeInSecondsToRotate'); disp(timeInSecondsToRotate);

    if angleToRotateToCenterMidpointInCameraImage > 0 % point is to the right of center
        % rotate camera right which is - (negative) angular velocity
        rotationSpeed = -.1;  % .1 radians/second
    elseif angleToRotateToCenterMidpointInCameraImage < 0 % point is to the left of center
        %rotate camera left which is + (positive) angular velocity
        rotationSpeed = .1;  % .1 radians/second
    else
        % don't rotate because the camera is centered on the point already
        timeInSecondsToRotate = 0;
    end

    % Rotate for calculated time
    messageSent = false;
    tic;
    while toc < timeInSecondsToRotate
        if messageSent == false
            moveTurtleBot(velocity_pub, rotationSpeed, 0)
            messageSent = true;
        end
    end

    stopTurtleBot(velocity_pub);
end