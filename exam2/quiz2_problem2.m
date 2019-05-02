close all;
% quiz 2 problem 2a
C = imread('combined_lanes.jpg'); %read image
bw=rgb2gray(C); %Convert to Grayscale image
bw_ground = bw(1150:end,:);
bw_Sobel = edge(bw_ground,'Sobel'); %Detect edges using Sobel

figure(1); subplot(2,1,1); imshow(bw_Sobel);
subplot(2,1,2); imshow(bw);

% quiz2 problem 2b
focal_length = 3.6e-3; % convert focal length to meters
pixel_size = 1.12e-6; % convert pixel size to meters

x_prime = 876;
x_origin_camera = 1640;
x = (x_origin_camera-x_prime)*pixel_size;
z = (x*focal_length)/x_prime;