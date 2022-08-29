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

function [xTemp,uTemp] = augState(x,u,x0,MPC_vars,ModelParams,tl,GammaArray)
    
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