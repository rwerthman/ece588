function moveTurtleBotForGivenTime(velocity_pub, timeToMoveSeconds, rotationSpeed, linearSpeed)
    messageSent = false;
    tic;
    while toc < timeToMoveSeconds
        if messageSent == false
            moveTurtleBot(velocity_pub, rotationSpeed, linearSpeed)
            messageSent = true;
        end
    end
    
    stopTurtleBot(velocity_pub);
 end