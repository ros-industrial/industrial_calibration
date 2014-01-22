field_results;
for i=1:61, for j=1:52,fi=(i-1)*53+j; ex=f(fi,7);ey=f(fi,8);ez=f(fi,9);z(j,i) = sqrt(ex*ex+ey*ey+ez*ez); end; end;
x = 0:5:300;
y = 0:5:255;

figure(1);
mesh(x,y,z);
xlabel('Feet');
ylabel('Feet');
zlabel('Inches');
title('Localization Accuracy for Hanger (inches)')
print -djpg heatmesh.jpg

figure(2);
contourf(x,y,z);
colorbar();
xlabel('Feet');
ylabel('Feet');
title('Localization Accuracy in Inches for Hanger')
print -djpg heatimage.jpg

camera_vectors;
figure(3);
hold off;
h=quiver3(cv(1:18,1),cv(1:18,2),cv(1:18,3),cv(1:18,4)+.1,cv(1:18,5)+.1,cv(1:18,6),2.0);
set (h, "maxheadsize", 0.5);
[m,n] = size(cv);
hold on;
h=quiver3(cv(19:m,1),cv(19:m,2),cv(19:m,3),cv(19:m,4)+.1,cv(19:m,5)+.1,cv(19:m,6),.25);
set (h, "maxheadsize", 0.5);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Locations of Cameras in Hanger')
print -djpg cameralocs.jpg
