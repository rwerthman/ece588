close all;
% quiz 2 problem 1b

C = imread('IndoorImages_LineMarkings.jpg'); %read image
bw=rgb2gray(C); %Convert to Grayscale imag

bws=im2single(bw); %Covert gray scale uint8 image into single precision image
singlerow=bws(165,:)*2^15;

backward_approx_singlerow = filter([1 -1],[1],singlerow);

forward_approx_singlerow = zeros(size(singlerow));
for i = 1:length(forward_approx_singlerow)
    if i < length(singlerow)
        forward_approx_singlerow(i) = singlerow(i+1) - singlerow(i);
    end
end

%subplot(211); plot(backward_approx_singlerow(2:end-1));
figure(1);
plot(singlerow);
hold on;
X = find(abs(backward_approx_singlerow(2:end-1)) > .5); % threshold the values at .5
for x = i:length(X)
    plot(backward_approx_singlerow(X(i)), '-o'); % only plot the values greater than .5
end
%plot(prctile(backward_approx_singlerow(2:end-1),[60:100],'all'));
