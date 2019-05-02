function [image_sub, velocity_pub, laser_sub, imu_sub] = initTurtleBot()
    % Connect to Turtlebot
    % Connect to an External ROS Master
    % ip address of TurtleBot and Matlab, replace these values accordingly
    ip_TurtleBot = '141.215.214.246';    
    ip_Matlab = '141.215.202.219';
    
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
    
    %%% Subscribe to the robot lidar data
    if ismember(TurtleBot_Topic.laser, rostopic('list'))
        laser_sub = rossubscriber(TurtleBot_Topic.laser);
    end
    
    %%% read imu
    if ismember(TurtleBot_Topic.imu, rostopic('list'))
        imu_sub = rossubscriber(TurtleBot_Topic.imu, 'sensor_msgs/Imu');
    end
end