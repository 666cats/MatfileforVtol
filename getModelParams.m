function ModelParams=getModelParams()

ModelParams.sx=13; %12+1
ModelParams.su=6; %5+1
ModelParams.nx=13; %12+1
ModelParams.nu=6; %5+1

ModelParams.W=1.2;
ModelParams.L=1.2;
ModelParams.H=0.4;


ModelParams.stateindex_x=1;
ModelParams.stateindex_y=2;
ModelParams.stateindex_z=3;
ModelParams.stateindex_phi=4;
ModelParams.stateindex_theta=5;
ModelParams.stateindex_psi=6;
ModelParams.stateindex_vx=7;
ModelParams.stateindex_vy=8;
ModelParams.stateindex_vz=9;
ModelParams.stateindex_omega_x=10;
ModelParams.stateindex_omega_y=11;
ModelParams.stateindex_omega_z=12;
ModelParams.stateindex_mu=13;

ModelParams.inputindex_delta_aileron_l=1;
ModelParams.inputindex_delta_aileron_r=2;
ModelParams.inputindex_canard=3;
ModelParams.inputindex_motor_l=4;
ModelParams.inputindex_motor_r=5;
ModelParams.inputindex_vmu=6;
% ModelParams.inputindex_motor_t=5;

%% 机械结构常数
ModelParams.m=1.2; %总质量
ModelParams.Sa=0.09; %主翼面积
ModelParams.Sc=0.07; %鸭翼面积
ModelParams.rho=1.225; %空气密度
ModelParams.ct_s=0.002; %悬停电机推力系数
% ModelParams.ct_t=0.002; %推力电机推力系数
ModelParams.g=9.81; %重力因子
ModelParams.theta_a=0; %主翼安装角
ModelParams.theta_c=0.0698; %副翼安装角
ModelParams.lamda_1=0.67;%等效空速因子vx
ModelParams.lamda_2=0.33; %等效空速因子vm
ModelParams.kmotor=0.03;%后洗速度和PWM的线性因子

%% 离气动面和电机的距离
%左主翼面
ModelParams.b2sx_l=-0.3;
ModelParams.b2sy_l=-0.135;
ModelParams.b2sz_l=0;
%右主翼面
ModelParams.b2sx_r=0.3;
ModelParams.b2sy_r=0.135;
ModelParams.b2sz_r=0;
%鸭翼(假设鸭翼只参与俯仰)
ModelParams.b2cx=0;
ModelParams.b2cy=0.1858;
ModelParams.b2cz=-0.05;

%左悬停电机
ModelParams.b2px_l=-0.3;
ModelParams.b2py_l=-0.085;
ModelParams.b2pz_l=0;
%右悬停电机
ModelParams.b2px_r=0.3;
ModelParams.b2py_r=0.085;
ModelParams.b2pz_r=0;
%推力电机
% ModelParams.b2px_t=0;
% ModelParams.b2py_t=0.44;
% ModelParams.b2pz_t=0;

%% 升力系数
ModelParams.wing_cl0=0.8;
ModelParams.wing_cd0=0.01;
ModelParams.wing_clm=5.539;
ModelParams.wing_cdm=0.1910;
ModelParams.wing_aileron_clm=2.5;
ModelParams.wing_aileron_cdm=0.08;

ModelParams.canard_cl0=0.8;
ModelParams.canard_cd0=0.01;
ModelParams.canard_clm=5.539;
ModelParams.canard_cdm=0.1910;

%% 转动惯量
ModelParams.Jxx=0.01668;
ModelParams.Jxz=-0.00006;
ModelParams.Jyy=0.01600;
ModelParams.Jzz=0.03224;

end



