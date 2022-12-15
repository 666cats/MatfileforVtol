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

function [  ] = PlotPrediction1(u, att_de,fm,x,x_his,ModelParams,Ts )
    
    N=size(fm(1,:));
    N=N(2);
    
    figure(1);
    plot3(x(1,:),x(2,:),x(3,:),'-g')
    hold on
    set(gca,'Ydir','reverse');
    set(gca,'Zdir','reverse');
    vtolbox(x(:,1),ModelParams.W/2,ModelParams.L/2,ModelParams.H/2)
    axis equal
%     xlim([-50 50])
%     ylim([-50 50])
%     zlim([-50 0])
    hold off
    
    figure(2)
    subplot(3,1,1)
    plot([0:N-1]*Ts,fm(1,:))
    xlabel('time [s]')
    ylabel('fxb')
    subplot(3,1,2)
    plot([0:N-1]*Ts,fm(2,:))

    xlabel('time [s]')
    ylabel('fyb')
    subplot(3,1,3)

    plot([0:N-1]*Ts,fm(3,:))
    xlabel('time [s]')
    ylabel('fzb')

    figure(3)
    subplot(3,1,1)
    plot([0:N-1]*Ts,10*fm(4,:))
    hold on
    plot([0:N-1]*Ts,att_de(1,:),'-g')
    plot([0:N-1]*Ts,x_his(4,:),'-r')
    xlabel('time [s]')
    ylabel('l')
    legend('moment','o_de','Euler')
    hold off
    subplot(3,1,2)
    plot([0:N-1]*Ts,fm(5,:))
    hold on
    plot([0:N-1]*Ts,att_de(2,:),'-g')
    plot([0:N-1]*Ts,x_his(5,:),'-r')
    xlabel('time [s]')
    ylabel('m')
    hold off
    subplot(3,1,3)
    plot([0:N-1]*Ts,fm(6,:))
    hold on
    plot([0:N-1]*Ts,att_de(3,:),'-g')
    plot([0:N-1]*Ts,x_his(6,:),'-r')
    xlabel('time [s]')
    ylabel('n')
    hold off
    
    figure(4)
    subplot(5,1,1)
    plot([0:N-1]*Ts,u(1,:))
    xlabel('time [s]')
    ylabel('al')
    
    subplot(5,1,2)
    plot([0:N-1]*Ts,u(2,:))
    xlabel('time [s]')
    ylabel('ar')
    
    subplot(5,1,3)
    plot([0:N-1]*Ts,u(3,:))
    xlabel('time [s]')
    ylabel('c')
    
    subplot(5,1,4)
    plot([0:N-1]*Ts,u(4,:))
    xlabel('time [s]')
    ylabel('t')

    subplot(5,1,5)
    plot([0:N-1]*Ts,u(5,:))
    xlabel('time [s]')
    ylabel('t')

    
    figure(5);
    subplot(2,1,1)
    plot(x_his(1,:),x_his(2,:),'-g')
    hold on
    set(gca,'Ydir','reverse');
    set(gca,'Zdir','reverse');
    axis equal
%     xlim([-50 50])
%     ylim([-50 50])
%     zlim([-50 0])
    hold off
    subplot(2,1,2)
    plot([0:N-1]*Ts,x_his(7,:),'-g')
    
    pause(0.01)
    


end

