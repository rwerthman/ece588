function [eigenVector, eigenValue, populationMean] = getEigenVectorValueAndMean2b(population)

    population = double(population);
    % Get the mean of the population/image for each color component
    populationMean = mean(mean(population));
    
    % Calculate the covariance between 2 random variables at a time
    % Red with blue, red with green, ...
    v12 = cov(population(:,:,1), population(:,:,2));
    v13 = cov(population(:,:,1), population(:,:,3));
    v23 = cov(population(:,:,2), population(:,:,3));
    
    % Creat a covariance matrix of the 3 random variables RGB
    v = [v12(1,1) v12(1,2) v13(1,2);
        v12(2,1) v12(2,2) v23(1,2);
        v13(2,1) v23(2,1) v23(2,2)];
    
    % Get the 2 largest eigen vectors
    [ev,lv] = eig(v);
    eigenVector = ev(:,3);
    eigenValue = lv(3,3);
end