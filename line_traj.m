clc;
clear all;
i=1;

for t=0:0.01:1
    x=t;
    y=0;
    z=0;
    line_path(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:pi/2
    x=1+sin(t);
    y=0;
    z=cos(t)-1;
    line_path(i,:)=[x y z];
    i=i+1;
end

for t=-0.01:-0.01:-1
    x=2;
    y=0;
    z=-1+t;
    line_path(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:pi/2
    x=3-cos(t);
    y=0;
    z=-2-sin(t);
    line_path(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:1
    x=3+t;
    y=0;
    z=-3;
    line_path(i,:)=[x y z];
    i=i+1;
end


for t=0.01:0.01:2*pi
    x=4+sin(t);
    y=cos(t)-1;
    z=-3;
    line_path(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:2*pi
    x=4.01+sin(t);
    y=cos(t)-1.01;
    z=-3;
    line_path(i,:)=[x y z];
    i=i+1;
end



save('line_path.mat',"line_path");
% line_path=20*line_path;
plot3(line_path(:,1),line_path(:,2),line_path(:,3))