
ModelParams=getModelParams();
load("citymap.mat")


%% import an plot track
% use normal ORCA Track
load line_path.mat
track=15*line_path';
track(3,:)=track(3,:)*1;

% load eight_path.mat
% track=20*eight_path';
% track(3,:)=track(3,:)*1;
% track(1,:)=track(1,:)*0;

% load nihepath.mat
% track=20*nihepath';
% track(3,:)=track(3,:)*1;
[traj, borders] =splinify(track);
tl = traj.ppx.breaks(end);

figure(1);

plot3(track(1,259:1244),track(2,259:1244),track(3,259:1244),'-b');
hold on
%     show(map3D);
set(gca,'Ydir','reverse');
set(gca,'Zdir','reverse');
plot3(X_his(1,56:end),X_his(2,56:end),X_his(3,56:end),'--','Color','green','LineWidth',1.5)
xlabel('x/m')
ylabel('y/m')
zlabel('z/m')
title('实际跟踪轨迹与原轨迹')
axis equal
% [traj, borders] =splinify(track);
hold off


% figure(2)
% 
% plot(0.05:0.05:length(X_his(1,56:end))*0.05,X_his(7,56:end));
% hold on
% plot(0.05:0.05:length(X_his(1,56:end))*0.05,X_his(8,56:end));
% plot(0.05:0.05:length(X_his(1,56:end))*0.05,X_his(9,56:end));
% xlabel('时间/s')
% ylabel('速度/(m/s)')
% hold off

figure(2)

plot(0.05:0.05:length(X_his(1,56:end))*0.05,X_his(4,56:end),'-r',LineWidth=1.5);
hold on
plot(0.05:0.05:length(X_his(1,56:end))*0.05,X_his(5,56:end),'-g',LineWidth=1.5);
plot(0.05:0.05:length(X_his(1,56:end))*0.05,X_his(6,56:end),'-c',LineWidth=1.5);
legend('roll','pitch','yaw');
xlabel('时间/s')
ylabel('姿态/°')
title('VTOL欧拉角变化')
hold off
