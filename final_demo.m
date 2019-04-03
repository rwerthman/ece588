close all;

[image_sub, velocity_pub] = initTurtleBot();

velocity_msg = rosmessage(velocity_pub);

% Set some parameters that will be used in the processing loop.
% You can modify these values for different behavior.
spinVelocity = 0.5;       % Angular velocity (rad/s)
forwardVelocity = 0.0;    % Linear velocity (m/s)


% Find and white line and move to it
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
        figure(6); imshow(cameraImage); title('original image after rotation');
        
        moveTurtleBotToPoint(lineMidpoint, velocity_pub)
        break;
    end
end

% Follow white line
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
        figure(7); imshow(cameraImage); title('original image after rotation');
        moveTurtleBotToPoint(furthestPointOnLine, velocity_pub)
        break;
    end
    
end



rosshutdown;

function stopTurtleBot(velocity_pub)
    velocity_msg = rosmessage(velocity_pub);
    %stop the robot forward/backward movement
    velocity_msg.Linear.X = 0;
    velocity_msg.Angular.Z = 0;
    send(velocity_pub,velocity_msg);
end

function [image_sub, velocity_pub] = initTurtleBot()
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
end

function moveTurtleBot(velocity_pub, spinVelocity, forwardVelocity)
    velocity_msg = rosmessage(velocity_pub);
    velocity_msg.Angular.Z = spinVelocity;
    velocity_msg.Linear.X = forwardVelocity;
    send(velocity_pub, velocity_msg);
end

function moveTurtleBotToPoint(point, velocity_pub)
    velocity_msg = rosmessage(velocity_pub);
    % Move the robot to the line
    focal_length = 3.04e-3; %mm to m
    heightFromBottomOfCameraToGround = 0.1158; % in meters % xxx inches to bottom of camera from ground
    pix = 1.12e-6; %pixel size is 1.12um x 1.12um according to spec
    rpiMaxImageHeight = 480;
    distanceFromPointOnLineToBottomOfImage = rpiMaxImageHeight - (point(2));
    % forumala is z = y*focal_length/y'
    distanceToLine = (heightFromBottomOfCameraToGround*focal_length)/(distanceFromPointOnLineToBottomOfImage*pix); 

    velocity_msg.Linear.X = .1; % move robot .1 meters/second% speed = distance/time, time = distance/speed
    timeInSecondsToMoveForward = distanceToLine/velocity_msg.Linear.X;
    disp('distance to line'); disp(distanceToLine);
    disp('timeInSecondsToMoveForward'); disp(timeInSecondsToMoveForward);

    % Move the robot for a certain amount of time to go the calculated
    % distance to the point
    messageSent = false;
    tic;
    while toc < timeInSecondsToMoveForward
        if messageSent == false
            send(velocity_pub, velocity_msg);
            messageSent = true;
        end
    end
    
    % stop robot
    velocity_msg.Linear.X = 0;
    send(velocity_pub,velocity_msg);
end

function [cameraImage] = getCameraImage(image_sub)
    % Read an image in from a camera
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    cameraImage = readImage(image_compressed);
end

function [horizontalLineFound, lineMidpoint] = findHorizontalLine(cameraImage)

    bw = rgb2gray(cameraImage); % Convert color to gray scale image
    bw_ground = bw(220:end,:); % Only keep the image of the ground by using row operations on the array
    bwth = imbinarize(bw_ground, 0.6); % Binary image obtained by thresholding RGB
    bwth_Canny = edge(bwth,'Canny'); % Detect edges using Canny
    [H,T,R] = hough(bwth_Canny,'RhoResolution',0.5,'ThetaResolution',0.5);
    numpeaks = 1; %Specify the number of peaks
    P  = houghpeaks(H,numpeaks);
    
    % Find the lines in the image
    lines = houghlines(bwth_Canny,T,R,P,'FillGap',50,'MinLength',9); 
    
    % If we find a white line in the image
    if ~isempty(lines)
        velocity_msg.Angular.Z = 0; % Stop rotation
        send(velocity_pub, velocity_msg);
        
        figure(5); imshow(bwth_Canny); title('bw threshold canny'); hold on;
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
        
        figure(1); imshow(cameraImage); title('original image');
        figure(2); imshow(bw); title('black and white');
        figure(3); imshow(bw_ground); title('bw ground plane');
        figure(4); imshow(bwth); title('bw threshold');
    end
    
    if ~isempty(lines)
        horizontalLineFound = true;
        lineMidpoint = [xy(1,1),xy(1,2)+220];
    else
        horizontalLineFound = false;
        lineMidpoint = [0,0];
    end
end

function [verticalLineFound, furthestPointOnLine] = findVerticalLine(cameraImage)
    bw = rgb2gray(cameraImage); % Convert color to gray scale image
    bw_ground = bw(215:end,40:600); % Only keep the image of the ground by using row operations on the array
    bwth = imbinarize(bw_ground, 0.6); % Binary image obtained by thresholding RGB
    bwth_Sobel = edge(bwth,'Sobel', 'vertical'); % Detect edges using Canny
    [H,T,R] = hough(bwth_Sobel,'RhoResolution',0.5,'ThetaResolution',0.5);
    numpeaks = 1; %Specify the number of peaks
    P  = houghpeaks(H,numpeaks);
    
    % Find the lines in the image
    lines = houghlines(bwth_Sobel,T,R,P,'FillGap',50, 'MinLength', 50); 
    
        
        figure(5); imshow(bwth_Sobel); title('bw threshold canny'); hold on;
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
        
        figure(1); imshow(cameraImage); title('original image');
        figure(2); imshow(bw); title('black and white');
        figure(3); imshow(bw_ground); title('bw ground plane');
        figure(4); imshow(bwth); title('bw threshold');
        
        if ~isempty(lines)
            verticalLineFound = true;
            furthestPointOnLine = [xy(1,1)+40,xy(1,2)+215];
        else
            verticalLineFound = false;
            furthestPointOnLine = [0,0];
        end
 end
    
 function orientCameraTowardsPoint(point, velocity_pub)
    velocity_msg = rosmessage(velocity_pub);
    FoV = 62.2; %degrees
    horizontalResolution = 640; % pixels
    centerHorizontalPixel = horizontalResolution/2;
    degreesPerPixel = FoV/horizontalResolution; % degrees/pixel
    angleToRotateToCenterMidpointInCameraImage = (point(1) - centerHorizontalPixel) * degreesPerPixel;

    radiansToCenterPointInCameraView = deg2rad(abs(angleToRotateToCenterMidpointInCameraImage));
    rotationSpeed = .1; % .1 radians/second
    % speed = distance/time, time = distance/speed
    timeInSecondsToRotate = radiansToCenterPointInCameraView/rotationSpeed;
    disp('timeInSecondsToRotate'); disp(timeInSecondsToRotate);

    if angleToRotateToCenterMidpointInCameraImage > 0 % point is to the right of center
        % rotate camera right which is - (negative) angular velocity
        velocity_msg.Angular.Z = -rotationSpeed; % .1 radians/second
    elseif angleToRotateToCenterMidpointInCameraImage < 0 % point is to the left of center
        %rotate camera left which is + (positive) angular velocity
        velocity_msg.Angular.Z = rotationSpeed; % .1 radians/second
    else
        % don't rotate because the camera is centered on the midpoint
        timeInSecondsToRotate = 0;
    end

    % Rotate for calculated time
    messageSent = false;
    tic;
    while toc < timeInSecondsToRotate
        if messageSent == false
             send(velocity_pub, velocity_msg);
             messageSent = true;
         end
    end

    velocity_msg.Angular.Z = 0; % stop the robot angular rotation
    send(velocity_pub,velocity_msg);
end