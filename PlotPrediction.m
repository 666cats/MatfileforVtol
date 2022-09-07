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
    
    
    tl = traj.ppx.breaks(end);
    
    figure(1);
    plot3(track(1,:),track(2,:),track(3,:));
    hold on
    set(gca,'Ydir','reverse');
    set(gca,'Zdir','reverse');
    plot3(ppval(traj.ppx,mod(x(13,:),tl)),ppval(traj.ppy,mod(x(13,:),tl)),ppval(traj.ppz,mod(x(13,:),tl)),':k')
    plot3(x(1,:),x(2,:),x(3,:),'-g')
    vtolbox(x(:,1),ModelParams.W/2,ModelParams.L/2,ModelParams.H/2)
    axis equal
    hold off


    pause(0.001)

end

