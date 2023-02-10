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

function [X,U,dU,info] = getMPCmatrices(traj,MPC_vars,ModelParams,Xhor,Uhor,x0,u0,GammaArray)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% For each stage in the horizon compute the necessary system and cost %%%%%
% matricies and safe them in a array of structs 'stage' %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% cost scaling for numerics
costScale = MPC_vars.costScale;
% init stage struct
stage = struct([]);
%% Generate MPCC problem
% initial state (including previus input for input rate cost)

% state is augmented by the inputs and rate inputs are introduced to
% formulate rate input costs and constraints while retaining a block
% spares formulation
% given x_k+1 = A x_k + B u_k
% do the following state augmentation
% s_k = [x_k,u_k-1], v_k = du_k
% with the following linear system
% s_k+1 = [A B;0 I] s_k + [B;I] v_k

stage(1).x0 = x0;
stage(1).u0 = u0;
%这里的x0是u0更新后的x0，所以严格来说u0是u-1

for i = 1:MPC_vars.N
    Xk = Xhor(:,i);
    Uk = Uhor(:,i);
    % generate quadratic state(-input) cost
    stage(i).Qk = costScale*generateH(traj,MPC_vars,ModelParams,Xk,i);
    % quadratic rate input cost 
    stage(i).Rk = costScale*2*diag([MPC_vars.rdda_l,MPC_vars.rdda_r,MPC_vars.rdda_c,MPC_vars.rdm_l,MPC_vars.rdm_r,MPC_vars.rdVtheta]);
    % linear state(-input) cost
    stage(i).fk = costScale*generatef(traj,MPC_vars,ModelParams,Xk,i);
    % linearized dynamics
    [stage(i).Ak,stage(i).Bk,stage(i).gk] = getEqualityConstraints(Xk,Uk,MPC_vars,ModelParams,GammaArray);
    % linearized track constraints
    %  [stage(i).Ck, stage(i).ug, stage(i).lg] = getInequalityConstraints(borders(max(i-1,1),:),MPC_vars,ModelParams);
    % bounds
    [stage(i).lb, stage(i).ub] = getBounds(MPC_vars,ModelParams);
end
% terminal stage
i = MPC_vars.N+1; 
Xk = Xhor(:,i);
% generate quadratic state(-input) cost
stage(i).Qk = costScale*generateH(traj,MPC_vars,ModelParams,Xk,i);
% quadratic rate input cost 
stage(i).Rk = costScale*2*diag([MPC_vars.rdda_l,MPC_vars.rdda_r,MPC_vars.rdda_c,MPC_vars.rdm_l,MPC_vars.rdm_r,MPC_vars.rdVtheta]);
% linear state(-input) cost
stage(i).fk = costScale*generatef(traj,MPC_vars,ModelParams,Xk,i);
% linearized track constraints
% [stage(i).Ck, stage(i).ug, stage(i).lg] = getInequalityConstraints(borders(i-1,:),MPC_vars,ModelParams);
% bounds
[stage(i).lb, stage(i).ub] = getBounds(MPC_vars,ModelParams);
%% Call solver interface
if strcmp(MPC_vars.interface, 'Yalmip')
    % yalmip based interface (very slow)
    [X,U,dU,info] = YalmipInterface(stage,MPC_vars,ModelParams);
elseif strcmp(MPC_vars.interface, 'CVX')
    % CVX based interface (slow)
    [X,U,dU,info] = CVXInterface(stage,MPC_vars,ModelParams);
elseif strcmp(MPC_vars.interface, 'hpipm')
    % hpipm interface (prefered)
    [X,U,dU,info] = hpipmInterface(stage,MPC_vars,ModelParams);
elseif strcmp(MPC_vars.interface, 'quadprog')
    % quadprog interface (replace quadprog with a better solver if possible)
    [X,U,dU,info] = QuadProgInterface(stage,MPC_vars,ModelParams);
else
    error('invalid optimization interface')
end

end

% GENERATING Q
function Qk = generateH(pathinfo,MPC_vars,ModelParams,Xk,i)
    % get linearized contouring and lag errors
    Qtilde = generateQtilde(pathinfo,MPC_vars,ModelParams,Xk,i);


    % make Qtilde symetric (not symetric due to numerical issues)
%     Qtilde = 0.5 *(Qtilde+Qtilde');
    % Qk = contouring-lag error and real-input cost
    Qk = 2*blkdiag(Qtilde,diag([MPC_vars.rda_l,MPC_vars.rda_r,MPC_vars.rda_c,MPC_vars.rm_l,MPC_vars.rm_r,MPC_vars.rVtheta]));
    % scale cost
    Qk = blkdiag(MPC_vars.invTx,MPC_vars.invTu)*Qk*blkdiag(MPC_vars.invTx,MPC_vars.invTu) + 1e-12*eye(19);
end

% compute linear contouring and lag errors
function Qtilde = generateQtilde(pathinfo,MPC_vars,ModelParams,Xk,i)
    if i == MPC_vars.N+1
        Q = diag([MPC_vars.qCNmult*MPC_vars.qC, MPC_vars.qL]);
    else
        Q = diag([MPC_vars.qC, MPC_vars.qL]);
    end
        
    theta_virt=mod(Xk(end),pathinfo.ppx.breaks(end));
    [grad_eC, grad_eL] = getErrorGradient(pathinfo, theta_virt, ModelParams,Xk(1), Xk(2),Xk(3));
    errorgrad = [grad_eC; grad_eL]; 
    Qtilde = errorgrad'*Q*errorgrad; 
end

function [grad_eC, grad_eL] = getErrorGradient(pathinfo, theta_virt, ModelParams, x_phys, y_phys,z_phys)

    [deC_dtheta, deL_dtheta,deC_dx,deC_dy,deC_dz,deL_dx,deL_dy,deL_dz] = getderror_dtheta(pathinfo, theta_virt, x_phys, y_phys,z_phys);
    
    grad_eC = [ deC_dx,deC_dy,deC_dz, zeros(1, ModelParams.sx-4), deC_dtheta];
    grad_eL = [deL_dx,deL_dy,deL_dz, zeros(1, ModelParams.sx-4), deL_dtheta];
end

function [deC_dtheta, deL_dtheta, deC_dx,deC_dy,deC_dz,deL_dx,deL_dy,deL_dz] = getderror_dtheta(pathinfo, theta_virt, x_phys, y_phys,z_phys)
    
    dx=ppval(pathinfo.dppx,theta_virt);
    dy=ppval(pathinfo.dppy,theta_virt);
    dz=ppval(pathinfo.dppz,theta_virt);
    ddx=ppval(pathinfo.ddppx,theta_virt);
    ddy=ppval(pathinfo.ddppy,theta_virt);
    ddz=ppval(pathinfo.ddppz,theta_virt);
    x_virt=ppval(pathinfo.ppx,theta_virt);
    y_virt=ppval(pathinfo.ppy,theta_virt);
    z_virt=ppval(pathinfo.ppz,theta_virt);
    Dx=x_phys-x_virt;
    Dy=y_phys-y_virt;
    Dz=z_phys-z_virt;

    temp1=1/(dx.^2+dy.^2+dz.^2);
    temp2=(dy*Dz-dz*Dy).^2+(dz*Dx-dx*Dz).^2+(dx*Dy-dy*Dx).^2;

    dsqrttemp1_dtheta=-sqrt(temp1).^3*(dx*ddx+dy*ddy+dz*ddz);
    dtemp1_dtheta=-temp1.^2*(dx*ddx+dy*ddy+dz*ddz);
    
    deL_dx=sqrt(temp1)*dx;
    deL_dy=sqrt(temp1)*dy;
    deL_dz=sqrt(temp1)*dz;
    deL_dtheta=dsqrttemp1_dtheta*(dx*Dx+dy*Dy+dz*Dz)+sqrt(temp1)*(ddx*Dx+ddy*Dy+ddz*Dz-dx*dx-dy*dy-dz*dz);

    deC_dx=2*temp1*(dy*dy*Dx+dz*dz*Dx-dx*dz*Dz-dx*dy*Dy);
    deC_dy=2*temp1*(dx*dx*Dy+dz*dz*Dy-dx*dy*Dx-dz*dy*Dz);
    deC_dz=2*temp1*(dx*dx*Dz+dy*dy*Dz-dx*dz*Dx-dy*dz*Dy);

    temp3=ddy*Dz-dy*dz-ddz*Dy+dz*dy;
    temp4=ddz*Dx-dx*dz-ddx*Dz+dx*dz;
    temp5=ddx*Dy-dx*dy-ddy*Dx+dy*dx;
    dtemp2_dtheta=temp3*(dy*Dz-dz*Dy)+temp4*(dz*Dx-dx*Dz)+temp5*(dx*Dy-dy*Dx);
    deC_dtheta=2*(dtemp1_dtheta*temp2+dtemp2_dtheta*temp1);



end


% GENERATING f
function f = generatef(pathinfo,MPC_vars,ModelParams,Xk,i)

    x_phys = Xk(1);
    y_phys = Xk(2);
    z_phys = Xk(3);

    theta_virt=mod(Xk(end),pathinfo.ppx.breaks(end));%保证路线的循环
    [eC, eL] = getErrors(pathinfo, theta_virt,x_phys,y_phys,z_phys);
    e=[eC;eL];
    [grad_eC, grad_eL] = getErrorGradient(pathinfo, theta_virt, ModelParams, x_phys, y_phys ,z_phys);
    grad_e = [grad_eC; grad_eL];
    if i == MPC_vars.N+1
        Q = diag([MPC_vars.qCNmult*MPC_vars.qC, MPC_vars.qL]);
    else
        Q = diag([MPC_vars.qC, MPC_vars.qL]);
    end
  
   fx=2*e'*Q*grad_e - 2*Xk'*grad_e'*Q*grad_e; 
%     fx= - 2*Xk'*grad_e'*Q*grad_e; 
    fT = [fx, zeros(1,ModelParams.su-1), -MPC_vars.qVtheta];
    f=fT';
    
    f = blkdiag(MPC_vars.invTx,MPC_vars.invTu)*f;
end

function [eC, eL] = getErrors(pathinfo, theta_virt,x_phys,y_phys,z_phys)
    dx=ppval(pathinfo.dppx,theta_virt); % d x / d theta
    dy=ppval(pathinfo.dppy,theta_virt); % d y / d theta
    dz=ppval(pathinfo.dppz,theta_virt);
    % virtual positions
    x_virt=ppval(pathinfo.ppx,theta_virt);
    y_virt=ppval(pathinfo.ppy,theta_virt);
    z_virt=ppval(pathinfo.ppz,theta_virt);

    Dx=x_phys-x_virt;
    Dy=y_phys-y_virt;
    Dz=z_phys-z_virt;

    temp1=1/sqrt(dx.^2+dy.^2+dz.^2);
    temp2=sqrt((dy*Dz-dz*Dy).^2+(dz*Dx-dx*Dz).^2+(dx*Dy-dy*Dx).^2);
    
    eC=temp1.^2*temp2.^2;
    eL=temp1*(dx*Dx+dy*Dy+dz*Dz);
end

% EQUALITY CONSTRAINTS
function [Ak,Bk,gk] = getEqualityConstraints(Xk,Uk,MPC_vars,ModelParams,GammaArray) 

    nx = ModelParams.nx;
    nu = ModelParams.nu;
    % linearize and discretize nonlinear bicycle model
    [Ad, Bd, gd]=DiscretizedLinearizedModel(Xk,Uk,ModelParams,MPC_vars.Ts,GammaArray);
    % constructing augmented system with state-input scaling
    Ak = [MPC_vars.Tx*Ad*MPC_vars.invTx MPC_vars.Tx*Bd*MPC_vars.invTu; zeros(nu,nx) eye(nu)];
    Bk = [MPC_vars.Tx*Bd*MPC_vars.invTu;eye(nu)];
    gk = [MPC_vars.Tx*gd;zeros(nu,1)];
    
end

% INEQUALITY CONSTRAINTS


% BOUNDS
function [lb, ub]=getBounds(MPC_vars,ModelParams)

lb = MPC_vars.bounds(:,1);
ub = MPC_vars.bounds(:,2);


end
