% TODO: look at simple_obstacle_avoidance.m file
function [obstacleWasDetected] = detectObstacle(laser_sub, distanceThreshold)
    obstacleWasDetected = false;
    scan_data = receive(laser_sub);
    
    cameraFoVAngles = [1, 2, 3, 358, 359, 360];
    cameraFoVDistances = zeros(1, length(cameraFoVAngles));
    
    for i = 1 : length(cameraFoVAngles)
        cameraFoVDistances(i) = scan_data.Ranges(cameraFoVAngles(i));
    end
    
    if mean(cameraFoVDistances) < distanceThreshold && mean(cameraFoVDistances) ~= 0
        obstacleWasDetected = true;
    else
        obstacleWasDetected = false;
    end
    
    disp("mean"); disp(mean(cameraFoVDistances));
    disp("distanceThreshold"); disp(distanceThreshold);
    
end