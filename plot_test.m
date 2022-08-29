clc
clear all

w=0.6;
h=0.15;
l=0.4;

vtol(:,1)=[0,-w,-h];
vtol(:,2)=[0,-w,h];
vtol(:,3)=[0,w,h];
vtol(:,4)=[0,w,-h];
vtol(:,5)=[0,-w,-h];
vtol(:,6)=[l,0,0];
vtol(:,7)=[0,w,h];
vtol(:,8)=[0,w,-h];
vtol(:,9)=[l,0,0];
vtol(:,10)=[0,-w,h];

figure(1)
plot3(vtol(1,:),vtol(2,:),vtol(3,:),'k','LineWidth',1)
axis equal
