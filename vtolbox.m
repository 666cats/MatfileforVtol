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

function [] = vtolbox(x0,w,l,h)
    phi=x0(4);
    theta=x0(5);
    psi=x0(6);
    Rmatrix=[cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                -sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta)];
    vtol(:,1)=[0,-w,-h];
    vtol(:,2)=[0,-w,h];
    vtol(:,3)=[0,w,h];
    vtol(:,4)=[0,w,-h];
    vtol(:,5)=[0,-w,-h];
    vtol(:,6)=[l,0,0];
    vtol(:,7)=[0,w,h];
    vtol(:,8)=[0,w,-h];
    vtol(:,9)=[l,0,0];
    vtol(:,10)=[0,-w,h];
    vtol=Rmatrix*vtol;
    vtol=[x0(1);x0(2);x0(3)]+vtol;

    plot3(vtol(1,:),vtol(2,:),vtol(3,:),'k','LineWidth',1)
    
end
