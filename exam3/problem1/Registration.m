function [row, column] = Registration(gamma_L, dL, totalNumberOfColumns, totalNumberOfRows)

dL = dL * 1000;
delta_y = 0; % separation between LIDAR and Camera along y-axis from top view
delta_x = 65; % separation between LIDAR and Camera along x-axis from side view

HC = 112; % Camera height from ground in mm
HL = 177; % Lidar height from ground in mm

f = 3.04; % focal length of Pi camera in mm

beta = 0; % LIDAR only detects in the horizontal plane so the angle to object from the lidar is 0
    
numerator_tangammac = dL * cosd(beta) * sind(gamma_L) + delta_y;
denominator_tangammac = dL * cosd(beta) * cosd(gamma_L) - delta_x;

gamma_C = atan2d(numerator_tangammac, denominator_tangammac);

numerator_tanalpha = ((HC - HL) + dL * sind(beta)) * cosd(gamma_C);
denominator_tanalpha = dL * cosd(beta) * cosd(gamma_L) - delta_x;

alpha = atan2d(numerator_tanalpha, denominator_tanalpha);

rsize = (2.76/totalNumberOfRows);
csize = (3.68/totalNumberOfColumns);

row = -f * sind(alpha) / rsize;
column = -f * sind(gamma_C)/ csize;

end
    