[image_sub, velocity_pub, laser_sub, imu_sub] = initTurtleBot();

% imu_data = receive(imu_sub);
% robot_Orientation = [imu_data.Orientation.W, imu_data.Orientation.X, imu_data.Orientation.Y, imu_data.Orientation.Z];
% robot_Rotationa = quat2rotm(robot_Orientation);
% [r1a,r2a,r3a] = quat2angle(robot_Orientation);
% 
% moveTurtleBotForGivenTime(velocity_pub, 15, -0.1, 0);

% imu_data = receive(imu_sub);
% robot_Orientation = [imu_data.Orientation.W, imu_data.Orientation.X, imu_data.Orientation.Y, imu_data.Orientation.Z];
% robot_Rotationb = quat2rotm(robot_Orientation);
% [r1b,r2b,r3b] = quat2angle(robot_Orientation);

moveTurtleBot(velocity_pub, 0, .1);
distanceThreshold = 0.3; % in meters
[obstacleWasDetected] = detectObstacle(laser_sub, distanceThreshold);

while ~obstacleWasDetected
    [obstacleWasDetected] = detectObstacle(laser_sub, distanceThreshold);
end

stopTurtleBot(velocity_pub);
moveAroundObstacle(velocity_pub, laser_sub, distanceThreshold);

deinitTurtleBot(velocity_pub);