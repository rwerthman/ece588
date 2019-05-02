close all;

% Using the Hue component of the image, design a door detector
rgb_I = imread('ScreenShot.jpg');
figure; imshow(rgb_I);
hsv_I = rgb2hsv(rgb_I);

figure; imshow(hsv_I(:,:,1));
threshold_hsv_H_above = hsv_I(:,:,1) < .199;
threshold_hsv_H_below = hsv_I(:,:,1) > .08;
clamped_hsv_H = and(threshold_hsv_H_above, threshold_hsv_H_below);
figure; imshow(clamped_hsv_H);
[bw_Canny_H] = edge(clamped_hsv_H ,'Canny'); %Detect edges using Canny
figure; imshow(bw_Canny_H);

% TODO: Identify squares based on edges

% Using the Saturation component of the image, design a white marker detector
close all;
rgb_I = imread('ScreenShot.jpg');
figure; imshow(rgb_I);
hsv_I = rgb2hsv(rgb_I);

threshold_hsv_S = hsv_I(:,:,2) < .01; % The closer to 0 saturation, the whiter it is.
figure; imshow(hsv_I(:,:,2));
figure; imshow(threshold_hsv_S);
[bw_Canny_S] = edge(threshold_hsv_S ,'Canny'); %Detect edges using Canny
figure; imshow(bw_Canny_S);

