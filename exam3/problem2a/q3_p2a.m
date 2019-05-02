% Quiz 3 problem 2
close all;
C = imread('CameraImage.jpg'); %read image
C = imresize(C, [480, 640]);
figure; imshow(C);
figure; imshow(C); hold on;

% Population p1, the orange top
orangeTop = C(10:60, 309:349, :);
%figure; imshow(orangeTop);

% Population p1, orange bottom
orangeBottom = C(312:362, 291:331, :);
%figure; imshow(orangeBottom);

% Population p1, orange bottom
orangeBottom2 = C(402:452, 287:327, :);
%figure; imshow(orangeBottom2);

% Population p2, the purple wall left
purpleWallLeft = C(1:480, 1:192, :);
%figure; imshow(purpleWallLeft);

% Population p2, purple wall right
purpleWallRight = C(1:480, 449:640, :);
%figure; imshow(purpleWallRight);

% Take the image information from all of the image populations and put them into 1 matrix
population1 = [orangeTop; orangeBottom; orangeBottom2];
population2 = [purpleWallLeft; purpleWallRight];

[eigenVector1, eigenValue1, populationMean1] = getEigenVectorValueAndMean2a(population1);
[eigenVector2, eigenValue2, populationMean2] = getEigenVectorValueAndMean2a(population2);

% Run the PCA classifier on all the pixels in the image.
numberOfPixelsInP1 = 0;
numberOfPixelsInP2 = 0;
pixelsInP1 = [0,0];
pixelsInP2 = [0,0];

for i = 1:size(C,1)
    for j = 1:size(C,2)
        pixel = C(i,j, :);
        [p1val] = classifyPixel2a(pixel, eigenVector1, eigenValue1, populationMean1);
        [p2val] = classifyPixel2a(pixel, eigenVector2, eigenValue2, populationMean2);
        
        if (p1val < p2val)
            % disp('Pixel belongs to population 1');
            numberOfPixelsInP1 = numberOfPixelsInP1 + 1;
            pixelsInP1 = [pixelsInP1; [i,j]];
        elseif (p1val > p2val)
            % disp('Pixel belongs to popluation 2');
            numberOfPixelsInP2 = numberOfPixelsInP2 + 1;
            pixelsInP2 = [pixelsInP2; [i,j]];
        end
    end
end

% Plot the pixels on the image.  Color them based on their classified
% population.
scatter(pixelsInP1(:, 2), pixelsInP1(:, 1), [], [0, 0, 1]);
scatter(pixelsInP2(:, 2), pixelsInP2(:, 1), [], [1, 0, 0]);
