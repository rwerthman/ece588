function [value] = classifyPixel2a(pixel, eigenVector, eigenValue, populationMean)

    pixel = double(pixel);
    
    % Subtract the population mean from the pixel
    pixelMinusMean = pixel - populationMean;

    % Convert the pixel to a vector so we can do the dot product with it
    pixel = [pixelMinusMean(:,:,1); pixelMinusMean(:,:,2); pixelMinusMean(:,:,3)];
    
    % Classify the pixel
    value = (dot(pixel, eigenVector.'))/sqrt(eigenValue);
end