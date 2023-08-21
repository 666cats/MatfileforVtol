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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [  ] = PlotPrediction( x,track,traj,ModelParams )
    
    load("citymap.mat")

    tl = traj.ppx.breaks(end);
    
    figure(1);
    
%     plot3(track(1,259:end),track(2,259:end),track(3,259:end));%不需要前一水平飞行阶段
    plot3(track(1,:),track(2,:),track(3,:),'-r');
    hold on
    show(map3D);
    set(gca,'Ydir','reverse');
    set(gca,'Zdir','reverse');
    plot3(ppval(traj.ppx,mod(x(13,:),tl)),ppval(traj.ppy,mod(x(13,:),tl)),ppval(traj.ppz,mod(x(13,:),tl)),':k')
    plot3(x(1,:),x(2,:),x(3,:),'-g','LineWidth',1)
    vtolbox(x(:,1),ModelParams.W/2,ModelParams.L/2,ModelParams.H/2)
    xlabel('x/m')
    ylabel('y/m')
    zlabel('z/m')
    title('')
    view(-28,33)
    axis equal
    light("Style","local","Position",[50 50 -100]);
    hold off


    pause(0.20)

end

