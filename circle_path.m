clc;
clear all;
i=1;

for t=1:-0.01:0.01
    x=1;
    y=0;
    z=t;
    circle(i,:)=[x y z];
    i=i+1;
end

for t=0:0.01:2*pi
    x=cos(t);
    y=sin(t);
    z=cos(t)*0;
    circle(i,:)=[x y z];
    i=i+1;
end


plot3(circle(:,1),circle(:,2),circle(:,3))