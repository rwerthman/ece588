% Quiz 3 problem 2b
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

% Population p1, golden band
goldenBand = C(189:239,295:335, :);

% Population p2, the purple wall left
purpleWallLeft = C(77:127, 70:110, :);
%figure; imshow(purpleWallLeft);

% Population p2, purple wall right
purpleWallRight = C(70:120, 450:490, :);
%figure; imshow(purpleWallRight);

% Population p2, floor left
floorLeft = C(289:339, 88:128, :);

% Population p2, floor right
floorRight = C(175:225, 514:554, :);

% Take the image information from all of the image populations and put them into 1 matrix
population1 = [orangeTop; orangeBottom; orangeBottom2; goldenBand];
population2 = [purpleWallLeft; purpleWallRight; floorLeft; floorRight];

combinedPopluation = [population1;population2];

[eigenVector, eigenValue, populationMean] = getEigenVectorValueAndMean2b(combinedPopluation);

% [transformedPopulation1] = transformPopluation(population1, eigenValue, eigenVector, populationMean);
% [transformedPopulation2] = transformPopluation(population2, eigenValue, eigenVector, populationMean);
% 
% figure; hold on;
% scatter(transformedPopulation1(:,1), [1:size(transformedPopulation1, 1)], [], [0, 0, 1]);
% scatter(transformedPopulation2(:,1), [1:size(transformedPopulation2, 1)], [], [1, 0, 0]);

% Run the PCA classifier on all the pixels in the image.
numberOfPixelsInP1 = 0;
numberOfPixelsInP2 = 0;
pixelsInP1 = [0,0];
pixelsInP2 = [0,0];

for i = 1:size(C,1)
    for j = 1:size(C,2)
        pixel = C(i,j, :);
        [pval] = classifyPixel2b(pixel, eigenValue, eigenVector, populationMean);
        if (pval > 0)
            % disp('Pixel belongs to population 1');
            numberOfPixelsInP1 = numberOfPixelsInP1 + 1;
            pixelsInP1 = [pixelsInP1; [i,j]];
        else
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

function [transformedPopulation] = transformPopluation(population, eigenValue, eigenVector, populationMean)

    population = double(population);
    
    population(:,:,1) = population(:,:,1) - populationMean(:,:,1);
    population(:,:,2) = population(:,:,2) - populationMean(:,:,2);
    population(:,:,3) = population(:,:,3) - populationMean(:,:,3);
    
    transformedPopulation = [0];
    
    for i = 1:size(population, 1)
        for j = 1:size(population, 2)
            pixel = [population(i,j,1); population(i,j,2); population(i,j,3)];
            transformedPopulation = [transformedPopulation; dot(pixel, eigenVector.')/sqrt(eigenValue)];
        end
    end
end
