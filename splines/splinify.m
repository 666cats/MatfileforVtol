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

function [traj, borders] = splinify(track)
%[traj, borders] = splinify(track)
%calculate the required spline and derivatives

Tr=track;%(track.outer+track.inner)/2;
Xt=Tr(1,:);
Yt=Tr(2,:);
Zt=Tr(3,:);

[traj.ppx, traj.ppy,traj.ppz,~]=normalizedSplineInterp(Xt,Yt,Zt,1,'y');
%traj.ppx = spline(1:length(Xt),Xt);

%calculate derivatives of desired trajectory
traj.dppx=getSplineDerivatives(traj.ppx);
traj.dppy=getSplineDerivatives(traj.ppy);
traj.dppz=getSplineDerivatives(traj.ppz);
traj.ddppx=getSplineDerivatives(traj.dppx);
traj.ddppy=getSplineDerivatives(traj.dppy);
traj.ddppz=getSplineDerivatives(traj.dppz);

%compute borders of track spline approximation
% [borders.pplx,borders.pply]=computeCenter(traj.ppx,traj.ppy,track.inner(1,:),track.inner(2,:),1);%红色线条内圈
% [borders.pprx,borders.ppry]=computeCenter(traj.ppx,traj.ppy,track.outer(1,:),track.outer(2,:),1);%红色线条外圈

% [borders.pplx,borders.pply]=computeCenter(traj.ppx,traj.ppy,track.inner(1,:),track.inner(2,:),1);
% [borders.pprx,borders.ppry]=computeCenter(traj.ppx,traj.ppy,track.outer(1,:),track.outer(2,:),1);


% borders.dpplx=getSplineDerivatives(borders.pplx);
% borders.dpply=getSplineDerivatives(borders.pply);
% borders.dpprx=getSplineDerivatives(borders.pprx);
% borders.dppry=getSplineDerivatives(borders.ppry);

%compute center (for compatibility and tests)
[borders.ppcx,borders.ppcy,borders.ppcz]=computeCenter(traj.ppx,traj.ppy,traj.ppz,Tr(1,:),Tr(2,:),Tr(3,:),1);
borders.dppcx=getSplineDerivatives(borders.ppcx);
borders.dppcy=getSplineDerivatives(borders.ppcy);
borders.dppcz=getSplineDerivatives(borders.ppcz);
borders.trackwidth=0.37;
end