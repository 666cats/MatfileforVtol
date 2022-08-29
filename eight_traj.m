clc;
clear all;
i=1;
for t=0:0.01:2*pi
    x=cos(t);
    y=sin(t)*cos(t);
    z=cos(t);
    eight_path(i,:)=[x y z];
    i=i+1;
end
plot3(eight_path(:,1),eight_path(:,2),eight_path(:,3))