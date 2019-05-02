% Problem 3b
figure(1);
col = [1:1:3280];
row = (4/3)*col+10;
m=[-1000:0.5:1000];
c1=-col(10)*m+row(10);
c2=-col(100)*m+row(100);
c3=-col(50)*m+row(50);
c4=-col(150)*m+row(150);

plot(m,c1,'r',m,c2,'g',m,c3,'b',m,c4,'k');
xlabel('m');ylabel('c');