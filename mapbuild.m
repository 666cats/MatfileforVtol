clc
clear all
map3D = occupancyMap3D;
[xGround,yGround,zGround] = meshgrid(-10:110,-80:10,10);
xyzGround = [xGround(:) yGround(:) zGround(:)];
occval = 1;
setOccupancy(map3D,xyzGround,occval)

% [xBuilding1,yBuilding1,zBuilding1] = meshgrid(20:30,50:60,-30:0);
% [xBuilding2,yBuilding2,zBuilding2] = meshgrid(50:60,10:30,-40:0);
% [xBuilding3,yBuilding3,zBuilding3] = meshgrid(40:60,50:60,-50:0);
% [xBuilding4,yBuilding4,zBuilding4] = meshgrid(70:80,35:45,-60:0);

[xBuilding1,yBuilding1,zBuilding1] = meshgrid(30:35,5:10,-10:10);
[xBuilding2,yBuilding2,zBuilding2] = meshgrid(45:50,5:10,-10:10);
[xBuilding3,yBuilding3,zBuilding3] = meshgrid(30:35,-10:-5,-10:10);
[xBuilding4,yBuilding4,zBuilding4] = meshgrid(45:50,-10:-5,-10:10);
[xBuilding5,yBuilding5,zBuilding5] = meshgrid(30:50,5:10,-45:-10);
[xBuilding6,yBuilding6,zBuilding6] = meshgrid(30:50,-10:-5,-45:-10);
[xBuilding7,yBuilding7,zBuilding7] = meshgrid(30:35,-5:5,-45:-10);
[xBuilding8,yBuilding8,zBuilding8] = meshgrid(45:50,-5:5,-45:-10);


[xBuilding9,yBuilding9,zBuilding9] = meshgrid(80:87,-15:-5,-70:10);
[xBuilding10,yBuilding10,zBuilding10] = meshgrid(97:102,-6:6,-90:10);
[xBuilding11,yBuilding11,zBuilding11] = meshgrid(95:105,-35:-25,-55:10);

[xBuilding12,yBuilding12,zBuilding12] = meshgrid(68:72,-75:-65,-50:10);
[xBuilding13,yBuilding13,zBuilding13] = meshgrid(68:72,-75:-73,-55:-50);
[xBuilding14,yBuilding14,zBuilding14] = meshgrid(68:72,-67:-65,-55:-50);
[xBuilding15,yBuilding15,zBuilding15] = meshgrid(68:72,-75:-65,-57:-55);


[xBuilding16,yBuilding16,zBuilding16] = meshgrid(0:4,-80:-65,-25:10);

[xBuilding17,yBuilding17,zBuilding17] = meshgrid(2:8,-50:-30,-32:-30);
[xBuilding18,yBuilding18,zBuilding18] = meshgrid(2:8,-50:-30,-25:-23);
[xBuilding19,yBuilding19,zBuilding19] = meshgrid(2:3,-50:-30,-30:-25);
[xBuilding20,yBuilding20,zBuilding20] = meshgrid(7:8,-50:-30,-30:-25);
[xBuilding21,yBuilding21,zBuilding21] = meshgrid(2:8,-50:-46,-23:10);
[xBuilding22,yBuilding22,zBuilding22] = meshgrid(2:8,-34:-30,-23:10);

[xBuilding23,yBuilding23,zBuilding23] = meshgrid(28:37,-49:-55,-40:10);
[xBuilding24,yBuilding24,zBuilding24] = meshgrid(15:25,-75:-65,-50:10);

[xBuilding25,yBuilding25,zBuilding25] = meshgrid(40:50,-55:-40,-10:10);
[xBuilding26,yBuilding26,zBuilding26] = meshgrid(25:35,-23:-10,-20:10);

xyzBuildings = [xBuilding1(:) yBuilding1(:) zBuilding1(:);...
                xBuilding2(:) yBuilding2(:) zBuilding2(:);...
                xBuilding3(:) yBuilding3(:) zBuilding3(:);...
                xBuilding4(:) yBuilding4(:) zBuilding4(:);
                xBuilding5(:) yBuilding5(:) zBuilding5(:);...
                xBuilding6(:) yBuilding6(:) zBuilding6(:);...
                xBuilding7(:) yBuilding7(:) zBuilding7(:);...
                xBuilding8(:) yBuilding8(:) zBuilding8(:);
                xBuilding9(:) yBuilding9(:) zBuilding9(:);...
                xBuilding10(:) yBuilding10(:) zBuilding10(:);...
                xBuilding11(:) yBuilding11(:) zBuilding11(:);...
                xBuilding12(:) yBuilding12(:) zBuilding12(:);
                xBuilding13(:) yBuilding13(:) zBuilding13(:);...
                xBuilding14(:) yBuilding14(:) zBuilding14(:);...
                xBuilding15(:) yBuilding15(:) zBuilding15(:);...
                xBuilding16(:) yBuilding16(:) zBuilding16(:);
                xBuilding17(:) yBuilding17(:) zBuilding17(:);...
                xBuilding18(:) yBuilding18(:) zBuilding18(:);...
                xBuilding19(:) yBuilding19(:) zBuilding19(:);...
                xBuilding20(:) yBuilding20(:) zBuilding20(:);
                xBuilding21(:) yBuilding21(:) zBuilding21(:);...
                xBuilding22(:) yBuilding22(:) zBuilding22(:);...
                xBuilding23(:) yBuilding23(:) zBuilding23(:);...
                xBuilding24(:) yBuilding24(:) zBuilding24(:);
                xBuilding25(:) yBuilding25(:) zBuilding25(:);...
                xBuilding26(:) yBuilding26(:) zBuilding26(:);
                ];

obs = 0.65;
updateOccupancy(map3D,xyzBuildings,obs)
show(map3D)

% if exist("citymap.mat",'file')
%     delete("citymap.mat")
% end
% 
% filePath = fullfile(pwd,"citymap.ot");
% exportOccupancyMap3D(map3D,filePath)

save("citymap.mat","map3D")