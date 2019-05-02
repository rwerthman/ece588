function deinitTurtleBot(velocity_pub)
    stopTurtleBot(velocity_pub);
    rosshutdown;
end