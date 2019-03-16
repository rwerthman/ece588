% Connect to Turtlebot
% Connect to an External ROS Master
% ip address of TurtleBot and Matlab, replace these values accordingly
ip_TurtleBot = '141.215.204.232';    
ip_Matlab = '141.215.219.73';      

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
velocity_msg = rosmessage(velocity_pub);

% Set some parameters that will be used in the processing loop.
% You can modify these values for different behavior.
spinVelocity = 0.5;       % Angular velocity (rad/s)
forwardVelocity = 0.0;    % Linear velocity (m/s)

velocityMessageSent = false;
amountOfTimesToRepeat = 3;
while 1
    % Rotate robot
    if velocityMessageSent == false
        velocity_msg.Angular.Z = spinVelocity;
        velocity_msg.Linear.X = forwardVelocity;
        send(velocity_pub, velocity_msg);
        velocityMessageSent = true;
    end
    
    % Read an image in from a camera
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    cameraImage = readImage(image_compressed);
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
        
        % put midpoint of line in the center of camera
        FoV = 62.2; %degrees
        horizontalResolution = 640; % pixels
        centerHorizontalPixel = horizontalResolution/2;
        degreesPerPixel = FoV/horizontalResolution; % degrees/pixel
        angleToRotateToCenterMidpointInCameraImage = (midpoint(1) - centerHorizontalPixel) * degreesPerPixel;
        
        radiansToCenterMidpoint = deg2rad(abs(angleToRotateToCenterMidpointInCameraImage));
        rotationSpeed = .1; % .1 radians/second
        % speed = distance/time, time = distance/speed
        timeInSecondsToRotate = radiansToCenterMidpoint/rotationSpeed;
        disp('timeInSecondsToRotate'); disp(timeInSecondsToRotate);
        
        if angleToRotateToCenterMidpointInCameraImage > 0 % point is to the right of center
            % rotate camera right which is - (negative) angular velocity
            velocity_msg.Angular.Z = -rotationSpeed; % .1 radians/second
        elseif angleToRotateToCenterMidpointInCameraImage < 0 % point is to the left of center
            %rotate camera left which is + (positive) angular velocity
            velocity_msg.Angular.Z = rotationSpeed; % .1 radians/second
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
        
        % Move the robot to the line
        focal_length = 3.04e-3; %mm to m
        cam_height = 0.1143; % in meters % 4.5 inches to middle of camera from ground
        pix = 1.12e-6; %pixel size is 1.12um x 1.12um according to spec
        distanceToLine = (cam_height*focal_length)/((midpoint(2)+220)*pix); % forumala is z = y*focal_length/y'
        disp('distance to midpoint'); disp(distanceToLine);
        velocity_msg.Linear.X = .1; % move robot .1 meters/second
        % speed = distance/time, time = distance/speed
        timeInSecondsToMoveForward = distanceToLine/velocity_msg.Linear.X;
        disp('timeInSecondsToMoveForward'); disp(timeInSecondsToMoveForward);
         
        % Move the robot for a certain amount of time to go the calculated distance to the line
        messageSent = false;
        tic;
        while toc < timeInSecondsToMoveForward
         if messageSent == false
             send(velocity_pub, velocity_msg);
             messageSent = true;
         end
        end
        
        velocity_msg.Linear.X = 0; %stop robot
        send(velocity_pub,velocity_msg);
        
        % TODO: Rotate until we find a white line that is almost vertical across
        % the camera image.  We might be able to use the edge operators 
        % to take in an direction of the line to detect.
        % BW = edge(I,method,threshold,direction)
        amountOfTimesToRepeat = amountOfTimesToRepeat - 1;
        velocityMessageSent = false;
    end
    
    if amountOfTimesToRepeat == 0
        break;
    end
end

% stop the robot rotation
velocity_msg.Angular.Z = 0;
send(velocity_pub, velocity_msg);

%stop the robot forward/backward movement
velocity_msg.Linear.X = 0;
send(velocity_pub,velocity_msg);

rosshutdown;