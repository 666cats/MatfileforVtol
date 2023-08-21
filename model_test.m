clear;clc;clear all;
Ts=0.05;
ModelParams=getModelParams();
GammaArray=getGammaArray(ModelParams);
x0=[0;0;0;0.4;0;0;8;0;0;0;0;0];
global f_m;
global f_m_end
global de_att
att_de=[]
u_his=[]
x_his=[]

for i=1:500
u=Pi_test(x0,Ts,i);
x0= SimTimeStep1(x0,u,Ts,ModelParams,GammaArray)';

phi=x0(4);
theta=x0(5);
RMatrix=[1,sin(phi)*tan(theta),cos(phi)*tan(theta);0,cos(phi),-sin(phi);0,sin(phi)*sec(theta),cos(phi)*sec(theta)];
Euler_b=inv(RMatrix)*x0(4:6)

f_m_end=[f_m_end,f_m(:,end)];
att_de=[att_de,de_att];
u_his=[u_his,u];
x_his=[x_his,x0];
PlotPrediction1(u_his,att_de,f_m_end,x0,x_his,ModelParams,Ts)
end

