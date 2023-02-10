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

function MPC_vars = getMPC_vars()



    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % MPC settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % prediction horizon
    MPC_vars.N = 50;%50
    % sampling time
    MPC_vars.Ts = 0.05;
    % use bounds on all opt variables (TODO implement selective bounds)
    MPC_vars.fullBound = 1;  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % state-input scaling %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % normalization matricies (scale states and inputs to ~ +/- 1
%     MPC_vars.Tx = diag(1./[1e3,1e3,1e3]);
%     MPC_vars.Tu = diag(1./[1,1,1,1000,1000,1000,10]);
% 
%     MPC_vars.invTx = diag([1,1,2*pi,2*pi,2*pi,10,10,10,pi,pi,pi,10]);
%     MPC_vars.invTu = diag([1,1,1,1000,1000,1000,10]);
% 
%     MPC_vars.TDu = eye(7);
%     MPC_vars.invTDu = eye(7);
    % identity matricies if inputs should not be normalized
    MPC_vars.Tx = eye(13);
    MPC_vars.Tu = eye(6);
    
    MPC_vars.invTx = eye(13);
    MPC_vars.invTu = eye(6);
    
    MPC_vars.TDu = eye(6);
    MPC_vars.invTDu = eye(6);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % state-input bounds %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % bounds for non-nomalized state-inputs
    MPC_vars.bounds = [-1e3,-1e3,-1e3,-20,-20,-20,0,-1,-3,-1,-1,-1,0,       -3,-3,-3, 0,0, -0.1,  -1,-1,-1,-1,-1,0;
                                1e3, 1e3, 1e3, 20, 20, 20, 15, 1, 3,  1, 1, 1,1e3,         3, 3, 3,20,20,20,       1, 1, 1,1,1,10;]'; 
    % bounds for nomalized state-inputs (bounds can be changed by changing
    % % normalization)
%     MPC_vars.bounds = [-1e4,-1e4,-3, 0.25,-3,-1,   0,    -1,-1, 0  ,  -0.25 ,-0.1,-10;
%                         1e4, 1e4, 3,   10, 3, 1, 1e4,     1, 1,10  ,   0.25 , 0.1, 10]'; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Cost Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    MPC_vars.qC = 1; % contouring cost
    MPC_vars.qCNmult= 1000; % increase of terminal contouring cost
    MPC_vars.qL= 1000; % lag cost
    MPC_vars.qVtheta= 0.5; % theta maximization cost

    MPC_vars.rda_l=1e-2;
    MPC_vars.rda_r=1e-2;
    MPC_vars.rda_c=1e-2;
    MPC_vars.rm_l=1e-2;
    MPC_vars.rm_r=1e-2;
    MPC_vars.rm_t=1e-2;
    MPC_vars.rVtheta=1e-4;

    MPC_vars.rdda_l=0.1;
    MPC_vars.rdda_r=0.1;
    MPC_vars.rdda_c=0.1;
    MPC_vars.rdm_l=0.1;
    MPC_vars.rdm_r=0.1;
    MPC_vars.rdm_t=0.1;
    MPC_vars.rdVtheta=0.1;


    MPC_vars.q_eta = 250; % cost on soft constraints (TODO implement soft constraints)

    MPC_vars.costScale = 0.01; % scaling of the cost for better numerics



end
