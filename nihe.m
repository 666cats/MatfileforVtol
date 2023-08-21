clc;
clear all;
i=1;

for t=0:0.01:1
    x=t;
    y=0;
    z=0;
    nihepath(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:pi/2
    x=1+sin(t);
    y=0;
    z=cos(t)-1;
    nihepath(i,:)=[x y z];
    i=i+1;
end

for t=-0.01:-0.01:-1
    x=2;
    y=0;
    z=-1+t;
    nihepath(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:pi/2
    x=3-cos(t);
    y=0;
    z=-2-sin(t);
    nihepath(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:1
    x=3+t;
    y=0;
    z=-3;
    nihepath(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:pi/2
    x=4+sin(t);
    y=cos(t)-1;
    z=-3;
    nihepath(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:1
    x=5;
    y=-1-t;
    z=-3;
    nihepath(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:pi/2
    x=1+4*cos(t);
    y=-2*sin(t)-2;
    z=-2-cos(t);
    nihepath(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:2.5*pi
    x=1-0.75*sin(t);
    y=-3.25-0.75*cos(t);
    z=-2+0.5*t/2/pi;
    nihepath(i,:)=[x y z];
    i=i+1;
end

for t=0.01:0.01:1
    x=0.25;
    y=-3.25+3.5*t;
    z=-1.375;
    nihepath(i,:)=[x y z];
    i=i+1;
end

save('nihepath.mat',"nihepath");
% nihepath=20*nihepath;
plot3(nihepath(:,1),nihepath(:,2),nihepath(:,3))