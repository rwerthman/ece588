close all;
C = imread('Univ-of-MI-Dearborn-IAVS-HighBay.jpg'); %read image
bw=rgb2gray(C); %Convert to Grayscale image

bw_Canny = edge(bw,'Canny'); %Detect edges using Canny

squareSideLength = 30;
% reference https://www.mathworks.com/help/images/ref/imfindcircles.html
accumulator = zeros(size(bw_Canny)); % Set up a an accumlator to track the votes for each candidate pixel

[m,n] = size(accumulator);
for x = 1:n
    for y = 1:m
        if bw_Canny(y,x) == 1 % if the pixel is a candidate pixel
            if (y+squareSideLength) <= m % check points below
               if any(bw_Canny(y:y+squareSideLength,x)) % if all the points below are candidate pixels
                    accumulator(y, x) = accumulator(y, x) + sum(bw_Canny(y:y+squareSideLength,x));
               end
            end
            if (x+squareSideLength) <= n % check points to the right
                if any(bw_Canny(y,x:x+squareSideLength))
                  accumulator(y, x) = accumulator(y, x) + sum(bw_Canny(y,x:x+squareSideLength));
                end
            end
            if (x+squareSideLength) <= n && (y+squareSideLength) <= m % 45 degrees to the right and down
                if any(bw_Canny(y+squareSideLength,x:x+squareSideLength))
                    accumulator(y, x) = accumulator(y, x) + sum(bw_Canny(y+squareSideLength,x:x+squareSideLength));
                end
                if any(bw_Canny(y:y+squareSideLength,x+squareSideLength))
                    accumulator(y, x) = accumulator(y, x) + sum(bw_Canny(y:y+squareSideLength,x));
                end
            end
        end
        
    end
end

voteThreshold = 80;
binWithVotesGreaterThanThreshold = accumulator > voteThreshold;
[row, col] = find(binWithVotesGreaterThanThreshold);

%figure(1); imshow(bw_Canny);
figure(2); imshow(C);
hold on

%draw squares
for i = 1 : length(row)
    rectangle('Position', [col(i),row(i),squareSideLength,squareSideLength], 'EdgeColor', 'b');
end


% draw center of squares
plot(col,row, '-or','LineStyle', 'none');