function moveTurtleBot(velocity_pub, spinVelocity, forwardVelocity)
    velocity_msg = rosmessage(velocity_pub);
    velocity_msg.Angular.Z = spinVelocity;
    velocity_msg.Linear.X = forwardVelocity;
    send(velocity_pub, velocity_msg);
end