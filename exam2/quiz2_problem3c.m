% quiz 2 problem 3c
% p = xcos(theta)+ysin(theta)
close all;
figure(2);
col = [1:1:3280];
row = (4/3)*col+10;

theta = [-90:1:90];
p1 = col(1)*cos(theta)+row(1)*sin(theta);
p2 = col(100)*cos(theta)+row(100)*sin(theta);
p3 = col(50)*cos(theta)+row(50)*sin(theta);
p4 = col(150)*cos(theta)+row(150)*sin(theta);

plot(theta,p1,'r',theta,p2,'g',theta,p3,'b',theta,p4,'k');
xlabel('theta');ylabel('p');