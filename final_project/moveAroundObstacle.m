function moveAroundObstacle(velocity_pub, laser_sub, distanceThreshold)
    repeat = true;
    while repeat
         % 1. Rotate to the right 90 degrees ~= 1.5 radians
         moveTurtleBotForGivenTime(velocity_pub, 15, -0.1, 0);
         % 2. Move forward a certain distance like .9 meters
         moveTurtleBotForGivenTime(velocity_pub, 9, 0, 0.1);
         % 3. Rotate left 90 degrees
         moveTurtleBotForGivenTime(velocity_pub, 15, 0.1, 0);

         if detectObstacle(laser_sub, distanceThreshold)
             repeat = true;
         else
             repeat = false;
         end
    end
    
    
    repeat = true;
    while repeat
        % 2. Move forward a certain distance like .9 meters
        moveTurtleBotForGivenTime(velocity_pub, 9, 0, 0.1);

        % 3. Rotate left 90 degrees
        moveTurtleBotForGivenTime(velocity_pub, 15, 0.1, 0);

        if detectObstacle(laser_sub, distanceThreshold)
            % 1. Rotate to the right 90 degrees ~= 1.5 radians
            moveTurtleBotForGivenTime(velocity_pub, 15, -0.1, 0);
            repeat = true;
       else
            repeat = false;
        end
    end
    
    % 2. Move forward a certain distance like .9 meters
    moveTurtleBotForGivenTime(velocity_pub, 9, 0, 0.1);
    
    % 1. Rotate to the right 90 degrees ~= 1.5 radians
    moveTurtleBotForGivenTime(velocity_pub, 15, -0.1, 0);
    
    
end