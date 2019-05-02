% Design a pixel by pixel pillar classifier using principal component analysis like the one discussed in class
close all;
C = imread('ObjectDetection.PNG'); %read image
figure; imshow(C);

% Population p1, the yellow pillar
pillarBox = C(130:192, 270:314, :);
figure; imshow(pillarBox);

% Mean
meanPillarBox = mean(mean(pillarBox));
disp("mean pillar box"); disp(meanPillarBox);

% Covariance
v12 = cov(pimg(:,:,1), pimg(:,:,2));
v13 = cov(pimg(:,:,1), pimg(:,:,3));
v23 = cov(pimg(:,:,2), pimg(:,:,3));

v = [v12(1,1) v12(1,2) v13(1,2);
    v12(2,1) v12(2,2) v23(1,2);
    v13(2,1) v23(2,1) v23(2,2)];

% Eigen vectors and eigen values
% each column in ev is an eigen vector
[ev,lv]=eig(v);

% Second largest eigen vector and eigen value
secondLargestEigenVector = ev(:, 2);
secondLargestEigenValue = lv(2,2);

pixel = C(130, 270, :);

valp1 = ((pixel - meanPillarBox) * secondLargestEigenVector')/sqrt(secondLargestEigenValue);
