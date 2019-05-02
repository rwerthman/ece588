
% Problem 3a code that kind of works
close all;
figure(1);
imshow(ones(2464,3280,3));
col = [1:1:3280];
row = (4/3)*col+10;
line(col,row);
title('Quiz2 Problem3a');
ylabel('row');
xlabel('col');

hold on;

plot(1,1,'o');
text(1,1,'origin');
plot(col(1), row(1));
text(col(1), row(1), 'intercept');




