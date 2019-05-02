close all;

% Quiz 3 problem 1
% Read in the camera image
cameraImage = imread('CameraImage.jpg');
figure; imshow(cameraImage); hold on;

% Get the image dimensions
cameraImageSize = size(cameraImage);

% Calculate the middle pixel of the camera
columnMidpoint = cameraImageSize(2)/2;
rowMidpoint = cameraImageSize(1)/2;

% Get the lidar range data
% Each index is a degree and each value at the index is a distance in
% meters
[lidarRangeData] = getLidarRangeData();

% Field of view is about 62 degrees
% Angles from middle of camera to the left of the camera, about 30 degrees
cameraFoVLeftSideAngles = 1:1:30;
cameraFoVLeftSideDistance = zeros(1, length(cameraFoVLeftSideAngles));

% Angles from right of camera to center of camera, about 30 degrees
cameraFoVRightSideAngles = 331:1:360;
cameraFoVRightSideDistance = zeros(1, length(cameraFoVRightSideAngles));

% Initialize arrays to store the rows and columns calculated from the
% registration formualas for each angle and distance
rowsLeft = zeros(1, length(cameraFoVLeftSideAngles));
columnsLeft = zeros(1, length(cameraFoVLeftSideAngles));
rowsRight = zeros(1, length(cameraFoVLeftSideAngles));
columnsRight = zeros(1, length(cameraFoVLeftSideAngles));

% Get the distance information for each angle
for i = 1 : length(cameraFoVLeftSideAngles)
    cameraFoVLeftSideDistance(i) = lidarRangeData(cameraFoVLeftSideAngles(i));
end

for i = 1 : length(cameraFoVRightSideAngles)
    cameraFoVRightSideDistance(i) = lidarRangeData(cameraFoVRightSideAngles(i));
end

for i = 1 : length(cameraFoVLeftSideAngles)
    [row, column] = Registration(cameraFoVLeftSideAngles(i), cameraFoVLeftSideDistance(i), cameraImageSize(2), cameraImageSize(1));
    rowsLeft(i) = row;
    columnsLeft(i) = column;
end

% Plot the data to the left of the camera center
% c = zeros(30, 3);
% c(:,3) = cameraFoVLeftSideDistance;
% scatter(columnsLeft + columnMidpoint, rowMidpoint - rowsLeft, 100, c, 'filled');
scatter(columnsLeft + columnMidpoint, rowMidpoint - rowsLeft, 100, cameraFoVLeftSideDistance, 'filled');

for i = 1 : length(cameraFoVRightSideAngles)
    [row, column] = Registration(cameraFoVRightSideAngles(i), cameraFoVRightSideDistance(i), cameraImageSize(2), cameraImageSize(1));
    rowsRight(i) = row;
    columnsRight(i) = column;
end

% Plot the data to the right of the camera center
% c = zeros(30, 3);
% c(:,3) = cameraFoVRightSideDistance;
% scatter(columnsRight + columnMidpoint, rowMidpoint - rowsRight, 100, c, 'filled');
scatter(columnsRight + columnMidpoint, rowMidpoint - rowsRight, 100, cameraFoVRightSideDistance, 'filled');