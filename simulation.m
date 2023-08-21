% Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%% MPCC Simulation Script
clear
close all
clc

%% add spline library
addpath('splines');
%% Load Parameters

MPC_vars = getMPC_vars();
ModelParams=getModelParams();
GammaArray=getGammaArray(ModelParams);
% choose optimization interface options: 'Yalmip','CVX','hpipm','quadprog'
MPC_vars.interface = 'quadprog';


nx = ModelParams.sx;
nu = ModelParams.su;
N = MPC_vars.N;
Ts = MPC_vars.Ts;
%% import an plot track
% use normal ORCA Track
% load line_path.mat
% track=15*line_path';
% track(3,:)=track(3,:)*1;

load eight_path.mat
track=20*eight_path';
track(3,:)=track(3,:)*1;
track(1,:)=track(1,:)*0;

% load nihepath.mat
% track=20*nihepath';
% track(3,:)=track(3,:)*1;
%% Simulation lenght and plotting
simN = 600;
%0=no plots, 1=plot predictions
plotOn = 1;
%0=real time iteration, 1=fixed number of QP iterations, 2=fixed number of damped QP iterations
QP_iter = 2;
% number of cars
%% Fit spline to track
% TODO spline function only works with regular spaced points.
% Fix add function which given any center line and bound generates equlally
% space tracks.
[traj, borders] =splinify(track);
tl = traj.ppy.breaks(end);

% store all data in one struct
TrackMPC = struct('traj',traj,'borders',borders,'track_center',track,'tl',tl);
%% Define starting position
startIdx = 1; %point (in terms of track centerline array) allong the track 
% where the car starts, on the center line, aligned with the track, driving
% straight with vx0
%since the used bicycle model is not well defined for slow velocities use vx0 > 0.5
vx0=8;
vy0=0;
vz0=0;

trackWidth=10;%临时占位值
% find theta that coresponds to the 10th point on the centerline
[theta, ~] = findTheta([track(1,startIdx),track(2,startIdx),track(3,startIdx)],track,traj.ppx.breaks,trackWidth,startIdx);


x0 = [track(1,startIdx),track(2,startIdx),track(3,startIdx),.... % point on centerline
      0,atan2(ppval(traj.dppz,theta),sqrt(ppval(traj.dppx,theta).^2+ppval(traj.dppy,theta).^2)),atan2(ppval(traj.dppy,theta),ppval(traj.dppx,theta)),... % Euler angle
      vx0,vy0,vz0,0,0,0,theta]'; %driving straight with vx0, and correct theta progress
    
% the find theta function performs a local search to find the projection of
% the position onto the centerline, therefore, we use the start index as an
% starting point for this local search
last_closestIdx = startIdx;
%% First initial guess
x = repmat(x0,1,N+1); % all points identical to current measurment
% first inital guess, all points on centerline aligned with centerline
% spaced as the car would drive with vx0
for i = 2:N+1
    theta_next = x(ModelParams.stateindex_mu,i-1)+Ts*vx0;
    phi_next = 0;
    t_next=-atan2(ppval(traj.dppz,theta_next),sqrt(ppval(traj.dppx,theta_next).^2+ppval(traj.dppy,theta_next).^2));
    psi_next=atan2(ppval(traj.dppy,theta_next),ppval(traj.dppx,theta_next));
    % phi_next can jump by two pi, make sure there are no jumps in the
    % initial guess
    if (x(ModelParams.stateindex_theta,i-1)-t_next) < -pi
        t_next = t_next-2*pi;
    end
    if (x(ModelParams.stateindex_theta,i-1)-t_next) > pi
        t_next = t_next+2*pi;
    end
    if (x(ModelParams.stateindex_psi,i-1)-psi_next) < -pi
        psi_next = psi_next-2*pi;
    end
    if (x(ModelParams.stateindex_psi,i-1)-psi_next) > pi
        psi_next = psi_next+2*pi;
    end
    x(:,i) = [ppval(traj.ppx,theta_next),ppval(traj.ppy,theta_next),ppval(traj.ppz,theta_next)... % point on centerline
              phi_next,t_next,psi_next... % aligned with centerline
              vx0,vy0,vz0 ,0,0,0,theta_next]'; %driving straight with vx0, and correct theta progress
end

u = zeros(nu,N); % zero inputs
uprev = zeros(nu,1); % last input is zero
%% Ohter cars

%% Initialize logging arrays
X_log = zeros(nx*(N+1),simN);
U_log = zeros(nu*N,simN);
B_log = zeros(nu*N,simN);
qpTime_log = zeros(1,simN);
X_his=[];
U_his=[];
%% initializtion
% solve problem 5 times without applying input
% inspiered by sequential quadratic programming (SQP)
for i = 1:5
    % formulate MPCC problem and solve it
    Iter_damping = 0.75; % 0 no damping
    [x_up, u_up, exitflag,info] = optimizer_mpcc(TrackMPC,MPC_vars,ModelParams, x, u, x0, uprev,GammaArray);
    x = Iter_damping*x + (1-Iter_damping)*x_up;
    u = Iter_damping*u + (1-Iter_damping)*u_up;

    if plotOn == 1
        % plot predictions
        PlotPrediction(x,track,traj,ModelParams)
    end
end
  

% xnow=x(:,1);
% pic_num=1;
% for i=1:N
% %     PlotPrediction(x(:,i),track,traj,ModelParams);
%     xnow = SimTimeStep(xnow,u(:,i),Ts,ModelParams,GammaArray)';
%     PlotPrediction(xnow,track,traj,ModelParams);
%     F=getframe(gcf);
%     I=frame2im(F);
%     [I,map]=rgb2ind(I,256);
%     if pic_num==1
%         imwrite(I,map,'test2.gif','gif','Loopcount',inf,'DelayTime',0.2);
%     else
%         imwrite(I,map,'test2.gif','gif','WriteMode','append','DelayTime',0.2);
%     end
%     pic_num = pic_num + 1;
% end
%% Simulation
global pic_num
pic_num=1;
for i = 1: simN
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% MPCC-Call %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % augment state and inputs by shifting previus optimal solution
    [x,u] = augState(x,u,x0,MPC_vars,ModelParams,tl,GammaArray,traj);
    %  formulate MPCC problem and solve it
    if QP_iter == 0
        [x, u, exitflag,info] = optimizer_mpcc(TrackMPC,MPC_vars,ModelParams, x, u, x0, uprev,GammaArray);
        qpTime_log(i) = info.QPtime;
    elseif QP_iter == 1
        % doing multiple "SQP" steps
        for k = 1:2
            [x, u, exitflag,info] = optimizer_mpcc(TrackMPC,MPC_vars,ModelParams, x, u, x0, uprev,GammaArray);
            qpTime_log(i) = qpTime_log(i) + info.QPtime;
        end
    elseif QP_iter == 2
        % doing multiple damped "SQP" steps
        for k = 1:2
            Iter_damping = 0.75; % 0 no damping
            [x_up, u_up, exitflag,info] = optimizer_mpcc(TrackMPC,MPC_vars,ModelParams, x, u, x0, uprev,GammaArray);
            x = Iter_damping*x + (1-Iter_damping)*x_up;
            u = Iter_damping*u + (1-Iter_damping)*u_up;
            qpTime_log(i) = qpTime_log(i) + info.QPtime;
        end
    else
        error('invalid QP_iter value')
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% simulate system %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     x0 = SimTimeStep(x(:,1),u(:,1),Ts,ModelParams,GammaArray)';

%     for k=1:13
%         if x0(k)<MPC_vars.bounds(k,1)
%             x0(k)=0.95*MPC_vars.bounds(k,1);
%         elseif x0(k)>MPC_vars.bounds(k,2)
%             x0(k)=0.95*MPC_vars.bounds(k,2);
%         end
%     end
      
    x0=x(:,2);
    x0 = unWrapX0(x0);
    [ theta, last_closestIdx] = findTheta(x0,track,traj.ppx.breaks,trackWidth,last_closestIdx);
    x0(ModelParams.stateindex_mu) = theta;
    uprev = u(:,1);
    
    X_his=[X_his,x0];
    U_his=[U_his,uprev];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% plotting and logging %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if plotOn == 1
        PlotPrediction(x,track,traj,ModelParams)
        F=getframe(gcf);
        I=frame2im(F);
        [I,map]=rgb2ind(I,256);
        if pic_num==1
            imwrite(I,map,'test2.gif','gif','Loopcount',inf,'DelayTime',0.2);
        else
            imwrite(I,map,'test2.gif','gif','WriteMode','append','DelayTime',0.2);
        end
        pic_num = pic_num + 1;
    end
    
    % log predictions and time
%     X_log(:,i) = reshape(x,(N+1)*7,1);
%     U_log(:,i) = reshape(u,(N)*3,1);
%     B_log(:,i) = reshape(b,N*4,1);
    
    
end


% PlotLog( X_log,U_log,Y,track,track2,simN,Ts)

%% Generating Stats
a = 1;
for i=1:simN-1
    if X_log(ModelParams.stateindex_theta,i+1) - X_log(ModelParams.stateindex_theta,i) < -0.9*tl
        LapStart(a) = i;
        a = a+1;
    end
end

if length(LapStart) > 1
    LapTime = diff(LapStart)*Ts;
else
    LapTime = NaN;
end

disp('------------------------------------')
disp(['Lap Time(s): ',num2str(LapTime)])
disp('------------------------------------')
disp(['Mean Computation Time: ',num2str(mean(qpTime_log))])
disp(['Max Computation Time: ',num2str(max(qpTime_log))])
disp(['Min Computation Time: ',num2str(min(qpTime_log))])
disp('------------------------------------')
