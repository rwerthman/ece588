function stopTurtleBot(velocity_pub)
    velocity_msg = rosmessage(velocity_pub);
    %stop the robot forward/backward movement
    velocity_msg.Linear.X = 0;
    velocity_msg.Angular.Z = 0;
    send(velocity_pub,velocity_msg);
end