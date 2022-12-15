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

function [xTemp,uTemp] = augState(x,u,x0,MPC_vars,ModelParams,tl,GammaArray,traj)
    
    nx = ModelParams.nx;
    nu = ModelParams.nu;
    N = MPC_vars.N;
    Ts = MPC_vars.Ts;
    indexmu = ModelParams.stateindex_mu;

    xTemp = zeros(nx,N+1);
    uTemp = zeros(nu,N);
    
    xTemp(:,1) = x0;
    uTemp(:,1) = u(:,2);
    for j=2:N-1
        xTemp(:,j) = x(:,j+1);
        uTemp(:,j) = u(:,j+1);
    end
    j = N;
    xTemp(:,j) = x(:,j+1);
    uTemp(:,j) = u(:,j);
    
    j = N+1;
    xTemp(:,j) = SimTimeStep(x(:,N+1),u(:,N),Ts,ModelParams,GammaArray);
    
%     for k=1:13
%         if xTemp(k,j)<MPC_vars.bounds(k,1)
%             xTemp(k,j)=0.95*MPC_vars.bounds(k,1);
%         elseif xTemp(k,j)>MPC_vars.bounds(k,2)
%             xTemp(k,j)=0.95*MPC_vars.bounds(k,2);
%         end
%     end
    

%     vx0=8;
%     vy0=0;
%     vz0=0;
%     
%     theta_next=x(ModelParams.stateindex_mu,j)+x(ModelParams.stateindex_vx,j)*Ts;
%     phi_next = 0;
%     t_next=-atan2(ppval(traj.dppz,theta_next),sqrt(ppval(traj.dppx,theta_next).^2+ppval(traj.dppy,theta_next).^2));
%     psi_next=atan2(ppval(traj.dppy,theta_next),ppval(traj.dppx,theta_next));
%     xTemp(:,j) = [ppval(traj.ppx,theta_next),ppval(traj.ppy,theta_next),ppval(traj.ppz,theta_next)... % point on centerline
%           x(ModelParams.stateindex_phi,j),x(ModelParams.stateindex_theta,j),x(ModelParams.stateindex_psi,j)... % aligned with centerline
%           x(ModelParams.stateindex_vx,j),x(ModelParams.stateindex_vy,j),x(ModelParams.stateindex_vz,j)... 
%           x(ModelParams.stateindex_omega_x,j),x(ModelParams.stateindex_omega_y,j),x(ModelParams.stateindex_omega_z,j),theta_next]'; %driving straight with vx0, and correct theta progress
    
%     xTemp(:,j) = 
    
    if (xTemp(ModelParams.stateindex_theta,1)-xTemp(ModelParams.stateindex_theta,2)) < -pi
        xTemp(ModelParams.stateindex_theta,2:end) = xTemp(ModelParams.stateindex_theta,2:end)-2*pi;
    end
    if (xTemp(ModelParams.stateindex_theta,1)-xTemp(ModelParams.stateindex_theta,2)) > pi
        xTemp(ModelParams.stateindex_theta,2:end) = xTemp(ModelParams.stateindex_theta,2:end)+2*pi;
    end
    if (xTemp(ModelParams.stateindex_psi,1)-xTemp(ModelParams.stateindex_psi,2)) < -pi
        xTemp(ModelParams.stateindex_psi,2:end) = xTemp(ModelParams.stateindex_psi,2:end)-2*pi;
    end
    if (xTemp(ModelParams.stateindex_psi,1)-xTemp(ModelParams.stateindex_psi,2)) > pi
        xTemp(ModelParams.stateindex_psi,2:end) = xTemp(ModelParams.stateindex_psi,2:end)+2*pi;
    end
        if (xTemp(ModelParams.stateindex_phi,1)-xTemp(ModelParams.stateindex_phi,2)) < -pi
        xTemp(ModelParams.stateindex_phi,2:end) = xTemp(ModelParams.stateindex_phi,2:end)-2*pi;
    end
    if (xTemp(ModelParams.stateindex_phi,1)-xTemp(ModelParams.stateindex_phi,2)) > pi
        xTemp(ModelParams.stateindex_phi,2:end) = xTemp(ModelParams.stateindex_phi,2:end)+2*pi;
    end


    
    if xTemp(indexmu,1) - xTemp(indexmu,2) < -0.75*tl
        xTemp(indexmu,2:end) = xTemp(indexmu,2:end)-tl;
    end
    
end