close all;

[image_sub, velocity_pub, laser_sub] = initTurtleBot();

figure;
scan_data = receive(laser_sub);
plot(scan_data);

cameraImage = getCameraImage(image_sub);
figure; imshow(cameraImage); hold on;

cameraFoVLeftSideAngles = 1:1:30;
cameraFoVLeftSideDistance = zeros(1, length(cameraFoVLeftSideAngles));

cameraFoVRightSideAngles = 331:1:360;
cameraFoVRightSideDistance = zeros(1, length(cameraFoVRightSideAngles));

rows = zeros(1, length(cameraFoVLeftSideAngles));
columns = zeros(1, length(cameraFoVLeftSideAngles));

for i = 1 : length(cameraFoVLeftSideAngles)
    cameraFoVLeftSideDistance(i) = scan_data.Ranges(cameraFoVLeftSideAngles(i));
end

for i = 1 : length(cameraFoVRightSideAngles)
    cameraFoVRightSideDistance(i) = scan_data.Ranges(cameraFoVRightSideAngles(i));
end

for i = 1 : length(cameraFoVLeftSideAngles)
    [row, column] = Registration(cameraFoVLeftSideAngles(i), cameraFoVLeftSideDistance(i));
    rows(i) = row;
    columns(i) = column;
end

plot(columns + 320, 240 - rows, 'o');

for i = 1 : length(cameraFoVRightSideAngles)
    [row, column] = Registration(cameraFoVRightSideAngles(i), cameraFoVRightSideDistance(i));
    rows(i) = row;
    columns(i) = column;
end

plot(columns + 320, 240 - rows, 'o');
