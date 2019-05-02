close all;

[image_sub, velocity_pub, laser_sub] = initTurtleBot();

findWhiteLineAndMoveToIt(image_sub, velocity_pub);

% Find vertical line to follow
velocityMessageSent = false;
while 1
    % Rotate the robot to find the vertical line
    if velocityMessageSent == false
        moveTurtleBot(velocity_pub, .5, 0)
        velocityMessageSent = true;
    end
    
    cameraImage = getCameraImage(image_sub); 
    [verticalLineFound, midpointOnLine] = findVerticalLine(cameraImage, velocity_pub);
    
    if verticalLineFound
        stopTurtleBot(velocity_pub);
        break;
    end
end

while 1
    cameraImage = getCameraImage(image_sub);
    [verticalLineFound, midpointOnLine] = findVerticalLine(cameraImage, velocity_pub);
    if verticalLineFound
        orientCameraTowardsPoint(midpointOnLine, velocity_pub, image_sub);
        [obstacleWasDetected] = detectObstacle(laser_sub, .5);
         % TODO Are there any obstacles between us and the point
        if ~obstacleWasDetected
           % An obstacle was not detected so keep moving towards point
           moveTurtleBotToPoint(midpointOnLine, velocity_pub);
        else
           moveAroundObstacle(velocity_pub, laser_sub, .5)
           moveTurtleBotToPoint(midpointOnLine, velocity_pub);
        end
    else
        % Look for center
        break;
    end
end

velocityMessageSent = false;
while 1
    if velocityMessageSent == false
        moveTurtleBot(velocity_pub, .5, 0)
        velocityMessageSent = true;
    end
    
    cameraImage = getCameraImage(image_sub);
    [centerFound, midpoint] = findCenter(cameraImage, velocity_pub);
    
    if centerFound
        orientCameraTowardsPoint(midpoint, velocity_pub);
        
        moveTurtleBotForGivenTime(velocity_pub, 2, 0, .5);
    end
    
end


deinitTurtleBot(velocity_pub);

function findWhiteLineAndMoveToIt(image_sub, velocity_pub)
    % Find the white line and move to it so the robot is over top of it
    velocityMessageSent = false;
    horizontalLineFound = false;
    lineMidpoint = [0,0];
    while ~horizontalLineFound
        % Rotate robot to find a line
        if velocityMessageSent == false
            moveTurtleBot(velocity_pub, .5, 0)
            velocityMessageSent = true;
        end

        cameraImage = getCameraImage(image_sub);

        [horizontalLineFound, lineMidpoint] = findHorizontalLine(cameraImage, velocity_pub);
    end
    
    stopTurtleBot(velocity_pub);   
    orientCameraTowardsPoint(lineMidpoint, velocity_pub, image_sub);      
    moveTurtleBotToPoint(lineMidpoint, velocity_pub)
end

% TODO: Use HSV and classifier to find yellow poles
function [turtlebotIsOrientedTowardsCenter] = orientedTowardsCenter(cameraImage)
    turtlebotIsOrientedTowardsCenter = true;
end

function [distanceToPoint] = calculateDistanceToPoint(point)
    % Forumala for distance is is z = y*focal_length/y'
    % y = distance from middle of camera lens to ground in meters
    % y' = |object row in image - center row of image| * rowsize
    % rowsize = height of image sensor/ number of rows in image (rpiMaxImageHeight)
    focal_length = 3.04e-3; % From specification
    y = 112e-3; % distance from middle of camera from ground
    heightOfImageSensor = 2.76e-3; % From specification
    rowsize = heightOfImageSensor/480;
    y_prime = abs(point(2) - 480/2) * rowsize;
    distanceToPoint = (y*focal_length)/y_prime;
    
    % Calculated value seems to be off by a factor of around x for each calculation.
    % We need to multiply the calculated distance by this factor to get
    % a more accurate distance value
    % errorFactor = 6.3;
    % distanceToPoint = distanceToPoint * errorFactor;
end

function moveTurtleBotToPoint(point, velocity_pub)
    x_velocity = .1; % .1 meters/second
    [distanceToPoint] = calculateDistanceToPoint(point);
    timeInSecondsToMoveForward = distanceToPoint/x_velocity;
    disp('distance to line'); disp(distanceToPoint);
    disp('timeInSecondsToMoveForward'); disp(timeInSecondsToMoveForward);   
    moveTurtleBotForGivenTime(velocity_pub, timeInSecondsToMoveForward, 0, x_velocity);
end

function [cameraImage] = getCameraImage(image_sub)
    % Read an image in from a camera
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    cameraImage = readImage(image_compressed);
end

function [horizontalLineFound, lineMidpoint] = findHorizontalLine(cameraImage, velocity_pub)

    bw = rgb2gray(cameraImage); % Convert color to gray scale image
    bw_ground = bw(220:end,:); % Only keep the image of the ground by using row operations on the array
    bwth = imbinarize(bw_ground, 0.6); % Binary image obtained by thresholding RGB
    
    bwth_Canny = edge(bwth,'Canny'); % Detect edges using Canny
    [H,T,R] = hough(bwth_Canny,'RhoResolution',0.5,'ThetaResolution',0.5);
    numpeaks = 1; %Specify the number of peaks
    P  = houghpeaks(H,numpeaks);
    
    lines = houghlines(bwth_Canny,T,R,P,'FillGap', 20,'MinLength', 50); 
    
    % If we find a white line in the image
    if ~isempty(lines)
        max_len = 0;
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
           
            % Determine the endpoints of the longest line segment
            len = norm(lines(k).point1 - lines(k).point2);
            if ( len > max_len)
               max_len = len;
               xy_long = xy;
               % Find the midpoint of the line which is the middle of the
               % line in camera
               midpoint = [((xy_long(1,1) + xy_long(2,1))/2), ((xy_long(1,2) + xy_long(2,2))/2)];
            end
        end
        
        stopTurtleBot(velocity_pub);
        
        figure; imshow(bwth_Canny); title('findHorizontalLine bw threshold canny'); hold on;
        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
        plot(midpoint(1),midpoint(2),'x','LineWidth',2,'Color','blue');
        
%         figure; imshow(cameraImage); title('findHorizontalLine original image');
%         figure; imshow(bw); title('findHorizontalLine black and white');
        figure; imshow(bw_ground); title('findHorizontalLine bw ground plane');
%         figure; imshow(bwth); title('findHorizontalLine bw threshold');
        
        horizontalLineFound = true;
        lineMidpoint = [midpoint(1), midpoint(2) + 220];
    else
        horizontalLineFound = false;
        lineMidpoint = [0,0];
    end   
end

function [verticalLineFound, lineMidpoint] = findVerticalLine(cameraImage, velocity_pub)
    bw = rgb2gray(cameraImage); % Convert color to gray scale image
    bw_ground = bw(300:end,:); % Only keep the image of the ground and a narrow horizontal field of view
    bwth = imbinarize(bw_ground, 0.52); % Binary image obtained by thresholding RGB
    %bwth_Sobel = edge(bwth,'Sobel', 'vertical'); % Detect only vertical edges
    bwth_Sobel = edge(bwth,'Canny');
    [H,T,R] = hough(bwth_Sobel,'RhoResolution',0.5,'ThetaResolution',0.5);
    numpeaks = 1; %Specify the number of peaks/lines to get values for
    P  = houghpeaks(H,numpeaks);
    
    % Find the lines in the image
    lines = houghlines(bwth_Sobel,T,R,P,'FillGap', 5, 'MinLength', 50); 
    
     if ~isempty(lines)
        max_len = 0;
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
           
            % Determine the endpoints of the longest line segment
            len = norm(lines(k).point1 - lines(k).point2);
            if ( len > max_len)
               max_len = len;
               xy_long = xy;
               % Find the midpoint of the line
               midpoint = [((xy_long(1,1) + xy_long(2,1))/2), ((xy_long(1,2) + xy_long(2,2))/2)];
            end
        end
        
        stopTurtleBot(velocity_pub);
        
        figure; imshow(bwth_Sobel); title('findVerticalLine bw threshold canny'); hold on;
        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
        plot(midpoint(1), midpoint(2), 'x', 'LineWidth', 2, 'Color', 'blue');
        
%         figure; imshow(cameraImage); title('findVerticalLine original image');
%         figure; imshow(bw); title('findVerticalLine black and white');
        figure; imshow(bw_ground); title('findVerticalLine bw ground plane');
%         figure; imshow(bwth); title('findVerticalLine bw threshold');
     
        verticalLineFound = true;
        lineMidpoint = [midpoint(1), midpoint(2) + 300];
     else
        verticalLineFound = false;
        lineMidpoint = [0,0];
     end
 end
    
 function orientCameraTowardsPoint(point, velocity_pub, image_sub)
    FoV = 62.2; % Horizontal field of view in degrees
    degreesPerPixel = FoV/640; % degrees/pixel
    angleToRotateToCenterMidpointInCameraImage = (point(1) - 640/2) * degreesPerPixel;

    radiansToCenterPointInCameraView = deg2rad(abs(angleToRotateToCenterMidpointInCameraImage));
    rotationSpeed = .1; % .1 radians/second
    % speed = distance/time, time = distance/speed
    timeInSecondsToRotate = radiansToCenterPointInCameraView/rotationSpeed;
    
    disp('angleToRotateToCenterMidpointInCameraImage'); disp(angleToRotateToCenterMidpointInCameraImage);
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
    
    moveTurtleBotForGivenTime(velocity_pub, timeInSecondsToRotate, rotationSpeed, 0);
    
    %cameraImage = getCameraImage(image_sub);
    %figure; imshow(cameraImage); title('image after rotation');
 end