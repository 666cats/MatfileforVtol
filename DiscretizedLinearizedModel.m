function [Ad, Bd, gd]=DiscretizedLinearizedModel(Xbar_k,Ubar_k,ModelParams,Ts,GammaArray)

sx=ModelParams.sx-1;
su=ModelParams.su-1;

mass=ModelParams.m; %总质量
Sa=ModelParams.Sa; %主翼面积
Sc=ModelParams.Sc; %鸭翼面积
rho=ModelParams.rho; %空气密度
ct_s=ModelParams.ct_s;
% ct_t=ModelParams.ct_t; %电机推力系数
g=ModelParams.g; %重力因子
theta_a=ModelParams.theta_a; %主翼安装角
theta_c=ModelParams.theta_c; %鸭翼安装角
l1=ModelParams.lamda_1;
l2=ModelParams.lamda_2;
kmotor=ModelParams.kmotor;

wing_cl0=ModelParams.wing_cl0;
wing_cd0=ModelParams.wing_cd0;
wing_clm=ModelParams.wing_clm;
wing_cdm=ModelParams.wing_cdm;
wing_a_clm=ModelParams.wing_aileron_clm;
wing_a_cdm=ModelParams.wing_aileron_cdm;

c_cl0=ModelParams.canard_cl0;
c_cd0=ModelParams.canard_cd0;
c_clm=ModelParams.canard_clm;
c_cdm=ModelParams.canard_cdm;

ga1=GammaArray.gamma1;
ga2=GammaArray.gamma2;
ga3=GammaArray.gamma3;
ga4=GammaArray.gamma4;
ga5=GammaArray.gamma5;
ga6=GammaArray.gamma6;
ga7=GammaArray.gamma7;
ga8=GammaArray.gamma8;

phi=Xbar_k(ModelParams.stateindex_phi);
theta=Xbar_k(ModelParams.stateindex_theta);
psi=Xbar_k(ModelParams.stateindex_psi);
v_x=Xbar_k(ModelParams.stateindex_vx);
v_y=Xbar_k(ModelParams.stateindex_vy);
v_z=Xbar_k(ModelParams.stateindex_vz);
o_x=Xbar_k(ModelParams.stateindex_omega_x);
o_y=Xbar_k(ModelParams.stateindex_omega_y);
o_z=Xbar_k(ModelParams.stateindex_omega_z);
mu=Xbar_k(ModelParams.stateindex_mu);

dal=Ubar_k(ModelParams.inputindex_delta_aileron_l);
dar=Ubar_k(ModelParams.inputindex_delta_aileron_r);
ml=Ubar_k(ModelParams.inputindex_motor_l);
mr=Ubar_k(ModelParams.inputindex_motor_r);
% mt=Ubar_k(ModelParams.inputindex_motor_t);
dc=Ubar_k(ModelParams.inputindex_canard);
vmu=Ubar_k(ModelParams.inputindex_vmu);

%左主翼面
b2sx_l=ModelParams.b2sx_l;
b2sy_l=ModelParams.b2sy_l;
b2sz_l=ModelParams.b2sz_l;
%右主翼面
b2sx_r=ModelParams.b2sx_r;
b2sy_r=ModelParams.b2sy_r;
b2sz_r=ModelParams.b2sz_r;
%鸭翼(假设鸭翼只参与俯仰)
b2cx=ModelParams.b2cx;
b2cy=ModelParams.b2cy;
b2cz=ModelParams.b2cz;

%左悬停电机
b2px_l=ModelParams.b2px_l;
b2py_l=ModelParams.b2py_l;
b2pz_l=ModelParams.b2pz_l;
%右悬停电机
b2px_r=ModelParams.b2px_r;
b2py_r=ModelParams.b2py_r;
b2pz_r=ModelParams.b2pz_r;
%推力电机
% b2px_t=ModelParams.b2px_t;
% b2py_t=ModelParams.b2py_t;
% b2pz_t=ModelParams.b2pz_t;


o=[phi;theta;psi];
v=[v_x;v_y;v_z];
omega=[o_x;o_y;o_z];

vi=sqrt(v_x*v_x+v_y*v_y+v_z*v_z);%air velocity without motor
vml=kmotor*ml;
vmr=kmotor*mr;
va_l=sqrt((l1*v_x+l2*vml)*(l1*v_x+l2*vml)+v_y*v_y+v_z*v_z);
va_r=sqrt((l1*v_x+l2*vmr)*(l1*v_x+l2*vmr)+v_y*v_y+v_z*v_z);
b2s_l=[b2sx_l;b2sy_l;b2sz_l];
b2s_r=[b2sx_r;b2sy_r;b2sz_r];
b2c=[b2cx;b2cy;b2cz];
b2p_l=[b2px_l;b2py_l;b2pz_l];
b2p_r=[b2px_r;b2py_r;b2pz_r];
% b2p_t=[b2px_t;b2py_t;b2pz_t];


Rb2s=zeros(3,3); % the rotation matrix of body 2 surface(main Rb2cwing)
Rb2s(1,1)=cos(theta_a);
Rb2s(1,3)=-sin(theta_a);
Rb2s(2,2)=1;
Rb2s(3,1)=sin(theta_a);
Rb2s(3,3)=cos(theta_a);

ac=theta_c+dc;
Rb2c=zeros(3,3); %the rotation matrix of body 2 canard
Rb2c(1,1)=cos(ac);
Rb2c(1,3)=-sin(ac);
Rb2c(2,2)=1;
Rb2c(3,1)=sin(ac);
Rb2s(3,3)=cos(ac);

vc=Rb2c*v;
aoa_c=atan(vc(3)/vc(1));% angle of attack(canard)
vsi=Rb2s*v;
aoa_s=atan(vsi(3)/vsi(1)); % angle of attack(main wing)
aos=asin(v_y/vi); %angle of sideslip

% the rotation matrix of wind_frame 2 body_frame
Rw2b=[cos(aos)*cos(aoa_s) -sin(aos)*cos(aoa_s) -sin(aoa_s);
    sin(aos) cos(aos) 0;
    cos(aos)*sin(aoa_s) -sin(aos)*sin(aoa_s) cos(aoa_s)];

cls_l=wing_cl0+wing_clm*aoa_s+wing_a_clm*dal;
cls_r=wing_cl0+wing_clm*aoa_s+wing_a_clm*dar;
cds_l=wing_cd0+wing_cdm*abs(aoa_s)+wing_a_cdm*abs(dal);
cds_r=wing_cd0+wing_cdm*abs(aoa_s)+wing_a_cdm*abs(dar);
clc=c_cl0+c_clm*aoa_c;
cdc=c_cd0+c_cdm*abs(aoa_c);

%% The Force Part

% the force of main wing
fs_l=[-(1/2)*rho*Sa*va_l*va_l*cds_l;
    0;
    -(1/2)*rho*Sa*va_l*va_l*cls_l];
fs_r=[-(1/2)*rho*Sa*va_r*va_r*cds_r;
    0;
    -(1/2)*rho*Sa*va_r*va_r*cls_r];
fsb_l=Rw2b*fs_l;
fsb_r=Rw2b*fs_r;
fsb=fsb_l+fsb_r;

% the force of canard
fc=[-(1/2)*rho*Sc*vi*vi*cdc;
    0;
    -(1/2)*rho*Sc*vi*vi*clc];
fcb=Rw2b*fc;

%the force of gravity
fgb=[-mass*g*sin(theta);
    mass*g*cos(theta)*sin(phi);
    mass*g*cos(theta)*cos(phi)];

% the force of motor
fpb_l=ct_s*[vml*vml-vi*vi;0;0];
fpb_r=ct_s*[vmr*vmr-vi*vi;0;0];
% fpb_t=ct_t*[vmt*vmt-vi*vi;0;0];
% fpb=fpb_l+fpb_r+fpb_t;
fpb=fpb_l+fpb_r;

fxb=fsb(1)+fcb(1)+fpb(1)+fgb(1);
fyb=fsb(2)+fcb(2)+fpb(2)+fgb(2);
fzb=fsb(3)+fcb(3)+fpb(3)+fgb(3);
%% The Moment Part

ms_l=cross(b2s_l,fsb_l);
ms_r=cross(b2s_r,fsb_r);
mc=cross(b2c,fcb);
mp_l=cross(b2p_l,fpb_l);
mp_r=cross(b2p_r,fpb_r);
% mp_t=cross(b2p_t,fpb_t);

% mtotal=ms_l+ms_r+mc+mp_l+mp_r+mp_t;
mtotal=ms_l+ms_r+mc+mp_l+mp_r;
l=mtotal(1);
m=mtotal(2);
n=mtotal(3);

%% formula of model
fm=[cos(theta)*cos(psi)*v_x+(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v_y+(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*v_z;
    cos(theta)*sin(psi)*v_x+(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v_y+(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*v_z;
    -sin(theta)*v_x+sin(phi)*cos(theta)*v_y+cos(phi)*cos(theta)*v_z;
    o_x+sin(phi)*tan(theta)*o_y+cos(phi)*tan(theta)*o_z;
    cos(phi)*o_y-sin(phi)*o_z;
    sin(phi)*sec(theta)*o_y+cos(phi)*sec(theta)*o_z;
    o_z*v_y-o_y*v_z+fxb/mass;
    o_x*v_z-o_z*v_x+fyb/mass;
    o_y*v_x-o_x*v_y+fzb/mass;
    ga1*o_x*o_y-ga2*o_y*o_z+ga3*l+ga4*n;
    ga5*o_x*o_z-ga6*(o_x*o_x-o_z*o_z)+m/ModelParams.Jyy;
    ga7*o_x*o_y-ga1*o_y*o_z+ga4*l+ga8*n;];

%% Derivatives of the position

dfm1_dphi=-sin(phi)*(v_z*cos(psi)*sin(theta)-v_y*sin(psi))+cos(phi)*(v_y*cos(psi)*sin(theta)+v_z*sin(psi));
dfm1_dtheta=v_z*cos(theta)*cos(phi)*cos(psi)-v_x*cos(psi)*sin(theta)+v_y*cos(theta)*cos(psi)*sin(phi);
dfm1_dpsi=-v_x*cos(theta)*sin(psi)+sin(phi)*(v_z*cos(psi)-v_y*sin(theta)*sin(psi))+cos(phi)*(-v_y*cos(psi)-v_z*sin(theta)*sin(psi));
dfm1_dvx=cos(theta)*cos(psi);
dfm1_dvy=cos(psi)*sin(theta)*sin(phi)-cos(phi)*sin(psi);
dfm1_dvz=cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi);

dfm2_dphi=v_y*cos(phi)*sin(theta)*sin(psi)-sin(phi)*((v_y-v_z)*cos(psi)+v_z*sin(theta)*sin(psi));
dfm2_dtheta=v_z*cos(theta)*cos(phi)*sin(psi)+(-v_x*sin(theta)+v_y*cos(theta)*sin(phi))*sin(psi);
dfm2_dpsi=cos(psi)*(v_x*cos(theta)+v_y*sin(theta)*sin(phi))+cos(phi)*(v_z*cos(psi)*sin(theta)-(v_y-v_z)*sin(psi));
dfm2_dvx=cos(theta)*sin(psi);
dfm2_dvy=cos(phi)*cos(psi)+sin(theta)*sin(phi)*sin(psi);
dfm2_dvz=cos(phi)*(-cos(psi)+sin(theta)*sin(psi));

dfm3_dphi=-v_z*cos(theta)*sin(phi);
dfm3_dtheta=-v_x*cos(theta)+v_y*cos(theta)*cos(theta)-sin(theta)*(v_z*cos(phi)+v_y*sin(theta));
dfm3_dpsi=0;
dfm3_dvx=-sin(theta);
dfm3_dvy=cos(theta)*sin(theta);
dfm3_dvz=cos(theta)*cos(phi);

%% Derivatives of the EulerAngle

dfm4_dphi=o_y*cos(phi)*tan(theta)-o_z*sin(phi)*tan(theta);
dfm4_dtheta=o_z*cos(phi)*sec(theta)*sec(theta)+o_y*sec(theta)*sec(theta)*sin(phi);
dfm4_dox=1;
dfm4_doy=sin(phi)*tan(theta);
dfm4_doz=cos(phi)*tan(theta);

dfm5_dphi=-o_z*cos(phi)-o_y*sin(phi);
dfm5_dtheta=0;
dfm5_dox=0;
dfm5_doy=cos(phi);
dfm5_doz=-sin(phi);

dfm6_dphi=sec(theta)*(o_y*cos(phi)-o_z*sin(phi));
dfm6_dtheta=sec(theta)*(o_z*cos(phi)+o_y*sin(phi))*tan(theta);
dfm6_dox=0;
dfm6_doy=sec(theta)*sin(phi);
dfm6_doz=cos(phi)*sec(theta);

%% Derivatives of the velocity

%----The garivity part----%

% dmgx_dphi=0;
% dmgx_dtheta=-g*mass*cos(theta);
% dmgy_dphi=g*mass*cos(theta)*cos(phi);
% dmgy_dtheta=-g*mass*sin(theta)*sin(phi);
% dmgz_dphi=-g*mass*cos(theta)*sin(phi);
% dmgz_dtheta=-g*mass*cos(phi)*sin(theta);

%----The aerodynamic force part----%

temp1=(v_z*cos(theta_a)+v_x*sin(theta_a))/(v_x*cos(theta_a)-v_z*sin(theta_a));
cos_beta=sqrt(1-(v_y*v_y)/(vi*vi));
cos_alpha=1/sqrt(1+temp1^2);
sin_beta=v_y/vi;
sin_alpha=temp1*cos_alpha;

dcosbdvx=(v_x*v_y*v_y)/(sqrt((v_x^2+v_z^2)/vi^2)*vi^4);
dcosbdvy=-v_y*(sqrt((v_x^2+v_z^2)/vi^2))^(3/2)/(v_x^2+v_z^2);
dcosbdvz=v_z*v_y*v_y/(sqrt((v_x^2+v_z^2)/vi^2)*vi^4);

temp2=(v_x*v_x+v_z*v_z)/(v_x*cos(theta_a)-v_z*sin(theta_a))^2;
dcosadvx=-v_z*(v_z*cos(theta_a)+v_x*sin(theta_a))/(v_x*v_x+v_z*v_z)/(-v_x*cos(theta_a)+v_z*sin(theta_a))/sqrt(temp2);
dcosadvy=0;
dcosadvz=-v_x*(v_z*cos(theta_a)+v_x*sin(theta_a))/(v_x*cos(theta_a)-v_z*sin(theta_a))^3/temp2^(3/2);

dsinbdvx=v_x*v_y/vi^(3/2);
dsinbdvy=(v_x*v_x+v_z*v_z)/vi^(3/2);
dsinbdvz=v_y*v_z/vi^(3/2);

dsinadvx=-v_z/(v_x*v_x+v_z*v_z)/temp2^(1/2);
dsinadvy=0;
dsinadvz=v_x/(v_x*v_x+v_z*v_z)/temp2^(1/2);

dadvx=-v_z/(v_x*v_x+v_z*v_z);
dadvy=0;
dadvz=v_x/(v_x*v_x+v_z*v_z);


dvaldvx=l1*(v_x*l1+l2*vml)/va_l;
dvaldvy=v_y/va_l;
dvaldvz=v_z/va_l;
dvardvx=l1*(v_x*l1+l2*vmr)/va_r;
dvardvy=v_y/va_r;
dvardvz=v_z/va_r;

dfsixdvx_l=-(rho*Sa*va_l*cds_l*dvaldvx+0.5*va_l*va_l*rho*Sa*wing_cdm*dadvx);
dfsixdvz_l=-(rho*Sa*va_l*cds_l*dvaldvz+0.5*va_l*va_l*rho*Sa*wing_cdm*dadvy);
dfsixdvy_l=-(rho*Sa*va_l*cds_l*dvaldvy+0.5*va_l*va_l*rho*Sa*wing_cdm*dadvz);

dfsizdvx_l=-(rho*Sa*va_l*cls_l*dvaldvx+0.5*va_l*va_l*rho*Sa*wing_clm*dadvx);
dfsizdvy_l=-(rho*Sa*va_l*cls_l*dvaldvy+0.5*va_l*va_l*rho*Sa*wing_clm*dadvy);
dfsizdvz_l=-(rho*Sa*va_l*cls_l*dvaldvz+0.5*va_l*va_l*rho*Sa*wing_clm*dadvz);

dfsixdvx_r=-(rho*Sa*va_r*cds_r*dvardvx+0.5*va_r*va_r*rho*Sa*wing_cdm*dadvx);
dfsixdvz_r=-(rho*Sa*va_r*cds_r*dvardvy+0.5*va_r*va_r*rho*Sa*wing_cdm*dadvy);
dfsixdvy_r=-(rho*Sa*va_r*cds_r*dvardvz+0.5*va_r*va_r*rho*Sa*wing_cdm*dadvz);

dfsizdvx_r=-(rho*Sa*va_r*cls_r*dvardvx+0.5*va_r*va_r*rho*Sa*wing_clm*dadvx);
dfsizdvy_r=-(rho*Sa*va_r*cls_r*dvardvy+0.5*va_r*va_r*rho*Sa*wing_clm*dadvy);
dfsizdvz_r=-(rho*Sa*va_r*cls_r*dvardvz+0.5*va_r*va_r*rho*Sa*wing_clm*dadvz);


dfsibxdvx_l=dcosbdvx*cos_alpha*fs_l(1)+dcosadvx*cos_beta*fs_l(1)+cos_beta*cos_alpha*dfsixdvx_l-dfsizdvx_l*sin_alpha-fs_l(3)*dsinadvx;
dfsibydvx_l=dfsixdvx_l*sin_beta+dsinbdvx*fs_l(1);
dfsibzdvx_l=dcosbdvx*sin_alpha*fs_l(1)+cos_beta*dsinadvx*fs_l(1)+cos_beta*sin_alpha*dfsixdvx_l+dcosadvx*fs_l(3)+dfsizdvx_l*cos_alpha;

dfsibxdvy_l=dcosbdvy*cos_alpha*fs_l(1)+dcosadvy*cos_beta*fs_l(1)+cos_beta*cos_alpha*dfsixdvy_l-dfsizdvy_l*sin_alpha-dsinadvy*fs_l(3);
dfsibydvy_l=dfsixdvy_l*sin_beta+dsinbdvy*fs_l(1);
dfsibzdvy_l=dcosbdvy*sin_alpha*fs_l(1)+cos_beta*dsinadvy*fs_l(1)+cos_beta*sin_alpha*dfsixdvy_l+dcosadvy*fs_l(3)+dfsizdvy_l*cos_alpha;

dfsibxdvz_l=dcosbdvz*cos_alpha*fs_l(1)+dcosadvz*cos_beta*fs_l(1)+cos_beta*cos_alpha*dfsixdvz_l-dfsizdvz_l*sin_alpha-dsinadvz*fs_l(3);
dfsibydvz_l=dfsixdvz_l*sin_beta+dsinbdvz*fs_l(1);
dfsibzdvz_l=dcosbdvz*sin_alpha*fs_l(1)+cos_beta*dsinadvz*fs_l(1)+cos_beta*sin_alpha*dfsixdvz_l+dcosadvz*fs_l(3)+dfsizdvz_l*cos_alpha;

dfsibxdvx_r=dcosbdvx*cos_alpha*fs_r(1)+dcosadvx*cos_beta*fs_r(1)+cos_beta*cos_alpha*dfsixdvx_r-dfsizdvx_r*sin_alpha-fs_r(3)*dsinadvx;
dfsibydvx_r=dfsixdvx_r*sin_beta+dsinbdvx*fs_r(1);
dfsibzdvx_r=dcosbdvx*sin_alpha*fs_r(1)+cos_beta*dsinadvx*fs_r(1)+cos_beta*sin_alpha*dfsixdvx_r+dcosadvx*fs_r(3)+dfsizdvx_r*cos_alpha;

dfsibxdvy_r=dcosbdvy*cos_alpha*fs_r(1)+dcosadvy*cos_beta*fs_r(1)+cos_beta*cos_alpha*dfsixdvy_r-dfsizdvy_r*sin_alpha-dsinadvy*fs_r(3);
dfsibydvy_r=dfsixdvy_r*sin_beta+dsinbdvy*fs_r(1);
dfsibzdvy_r=dcosbdvy*sin_alpha*fs_r(1)+cos_beta*dsinadvy*fs_r(1)+cos_beta*sin_alpha*dfsixdvy_r+dcosadvy*fs_r(3)+dfsizdvy_r*cos_alpha;

dfsibxdvz_r=dcosbdvz*cos_alpha*fs_r(1)+dcosadvz*cos_beta*fs_r(1)+cos_beta*cos_alpha*dfsixdvz_r-dfsizdvz_r*sin_alpha-dsinadvz*fs_r(3);
dfsibydvz_r=dfsixdvz_r*sin_beta+dsinbdvz*fs_r(1);
dfsibzdvz_r=dcosbdvz*sin_alpha*fs_r(1)+cos_beta*dsinadvz*fs_r(1)+cos_beta*sin_alpha*dfsixdvz_r+dcosadvz*fs_r(3)+dfsizdvz_r*cos_alpha;

dfsixdda_l=-0.5*wing_a_cdm*rho*Sa*va_l*va_l;
dfsizdda_l=-0.5*wing_a_clm*rho*Sa*va_l*va_l;
dfsixdda_r=-0.5*wing_a_cdm*rho*Sa*va_r*va_r;
dfsizdda_r=-0.5*wing_a_clm*rho*Sa*va_r*va_r;

dfsibxdda_l=cos_beta*cos_alpha*dfsixdda_l-sin_alpha*dfsizdda_l;
dfsibydda_l=sin_beta*dfsixdda_l;
dfsibzdda_l=cos_beta*sin_alpha*dfsibxdda_l+cos_alpha*dfsizdda_l;

dfsibxdda_r=cos_beta*cos_alpha*dfsixdda_r-sin_alpha*dfsizdda_r;
dfsibydda_r=sin_beta*dfsixdda_r;
dfsibzdda_r=cos_beta*sin_alpha*dfsibxdda_r+cos_alpha*dfsizdda_r;


%----The canard force part----%

dacdvx=dadvx;
dacdvy=0;
dacdvz=dadvz;
dacddc=1;

cos_alpha_c=1/sqrt(1+(vc(3)/vc(1))^2);
sin_alpha_c=(vc(3)/vc(1))*cos_alpha_c;

tempc1=sqrt((v_x*v_x+v_z*v_z)/vc(1)/vc(1));
dsinacdvx=-v_z/(v_x*v_x+v_z*v_z)/tempc1;
dsinacdvy=0;
dsinacdvz=v_x/(v_x*v_x+v_z*v_z)/tempc1;
dsinacddc=1/tempc1;

tempc2=-vc(3)/vc(1);
dcosacdvx=dsinacdvx*tempc2;
dcosacdvy=0;
dcosacdvz=dsinacdvz*tempc2;
dcosacddc=dsinacddc*tempc2;

dvidvx=v_x/vi;
dvidvy=v_y/vi;
dvidvz=v_z/vi;

dfcxdvx=-(rho*Sc*vi*cdc*dvidvx+0.5*vi*vi*rho*Sc*c_cdm*dacdvx);
dfcxdvy=-(rho*Sc*vi*cdc*dvidvy+0.5*vi*vi*rho*Sc*c_cdm*dacdvy);
dfcxdvz=-(rho*Sc*vi*cdc*dvidvz+0.5*vi*vi*rho*Sc*c_cdm*dacdvz);

dfczdvx=-(rho*Sc*vi*clc*dvidvx+0.5*vi*vi*rho*Sc*c_clm*dacdvx);
dfczdvy=-(rho*Sc*vi*clc*dvidvy+0.5*vi*vi*rho*Sc*c_clm*dacdvy);
dfczdvz=-(rho*Sc*vi*clc*dvidvz+0.5*vi*vi*rho*Sc*c_clm*dacdvz);

dfcxddc=-0.5*c_cdm*rho*Sc*vi*vi*dacddc;
dfczddc=-0.5*c_clm*rho*Sc*vi*vi*dacddc;

dfcbxdvx=dcosbdvx*cos_alpha_c*fc(1)+dcosacdvx*cos_beta*fc(1)+cos_beta*cos_alpha_c*dfcxdvx-dfczdvx*sin_alpha_c-fc(3)*dsinacdvx;
dfcbydvx=dacdvx*sin_beta+dsinbdvx*fc(1);
dfcbzdvx=dcosbdvx*sin_alpha_c*fc(1)+cos_beta*dsinacdvx*fc(1)+cos_beta*sin_alpha_c*dfcxdvx+dcosacdvx*fc(3)+dfczdvx*cos_alpha_c;

dfcbxdvy=dcosbdvy*cos_alpha_c*fc(1)+dcosacdvy*cos_beta*fc(1)+cos_beta*cos_alpha_c*dfcxdvy-dfczdvy*sin_alpha_c-fc(3)*dsinacdvy;
dfcbydvy=dacdvy*sin_beta+dsinbdvy*fc(1);
dfcbzdvy=dcosbdvy*sin_alpha_c*fc(1)+cos_beta*dsinacdvy*fc(1)+cos_beta*sin_alpha_c*dfcxdvy+dcosacdvy*fc(3)+dfczdvy*cos_alpha_c;

dfcbxdvz=dcosbdvz*cos_alpha_c*fc(1)+dcosacdvz*cos_beta*fc(1)+cos_beta*cos_alpha_c*dfcxdvz-dfczdvz*sin_alpha_c-fc(3)*dsinacdvz;
dfcbydvz=dacdvz*sin_beta+dsinbdvz*fc(1);
dfcbzdvz=dcosbdvz*sin_alpha_c*fc(1)+cos_beta*dsinacdvz*fc(1)+cos_beta*sin_alpha_c*dfcxdvz+dcosacdvz*fc(3)+dfczdvz*cos_alpha_c;

dfcbxddc=cos_beta*(dcosacddc*fc(1)+dfcxddc*cos_alpha_c)-(dfczddc*sin_alpha_c+fc(3)*dsinacddc);
dfcbyddc=dfcxddc*sin_beta;
dfcbzddc=cos_beta*(dsinacddc*fc(1)+dfcxddc*sin_alpha_c)+(dfczddc*cos_alpha_c+fc(3)*dcosacddc);


%----The motor force part----%

dpbdvx_l=-2*ct_s*v_x;
dpbdvy_l=-2*ct_s*v_y;
dpbdvz_l=-2*ct_s*v_z;
dpbddml=2*ct_s*kmotor*kmotor*ml;

dpbdvx_r=-2*ct_s*v_x;
dpbdvy_r=-2*ct_s*v_y;
dpbdvz_r=-2*ct_s*v_z;
dpbddmr=2*ct_s*kmotor*kmotor*mr;

% dpbdvx_t=-2*ct_t*vx;
% dpbdvy_t=-2*ct_t*vy;
% dpbdvz_t=-2*ct_t*vz;
% dpbddmt=2*ct_t*mt*mt;

%---- The total force part----%

% dfxbdvx=dfsibxdvx_l+dfsibxdvx_r+dfcbxdvx+dpbdvx_l+dpbdvx_r+dpbdvx_t;
% dfxbdvy=dfsibxdvy_l+dfsibxdvy_r+dfcbxdvy+dpbdvy_l+dpbdvy_r+dpbdvy_t;
% dfxbdvz=dfsibxdvz_l+dfsibxdvz_r+dfcbxdvz+dpbdvz_l+dpbdvz_r+dpbdvz_t;
dfxbdvx=dfsibxdvx_l+dfsibxdvx_r+dfcbxdvx+dpbdvx_l+dpbdvx_r;
dfxbdvy=dfsibxdvy_l+dfsibxdvy_r+dfcbxdvy+dpbdvy_l+dpbdvy_r;
dfxbdvz=dfsibxdvz_l+dfsibxdvz_r+dfcbxdvz+dpbdvz_l+dpbdvz_r;
dfxbdda_l=dfsibxdda_l;
dfxbdda_r=dfsibxdda_r;
dfxbddc=dfcbxddc;
dfxbdml=dpbddml;
dfxbdmr=dpbddmr;
% dfxbdmt=dpbddmt;

dfybdvx=dfsibydvx_l+dfsibydvx_r+dfcbydvx;
dfybdvy=dfsibydvy_l+dfsibydvy_r+dfcbydvy;
dfybdvz=dfsibydvz_l+dfsibydvz_r+dfcbydvz;
dfybdda_l=dfsibydda_l;
dfybdda_r=dfsibydda_r;
dfybddc=dfcbyddc;

dfzbdvx=dfsibzdvx_l+dfsibzdvx_r+dfcbzdvx;
dfzbdvy=dfsibzdvy_l+dfsibzdvy_r+dfcbzdvy;
dfzbdvz=dfsibzdvz_l+dfsibzdvz_r+dfcbzdvz;
dfzbdda_l=dfsibzdda_l;
dfzbdda_r=dfsibzdda_r;
dfzbddc=dfcbzddc;

dfm7_dvx=dfxbdvx/mass;
dfm7_dvy=dfxbdvy/mass+o_z;
dfm7_dvz=dfxbdvz/mass-o_y;
dfm7_dox=0;
dfm7_doy=-v_z;
dfm7_doz=v_y;
dfm7_dphi=0;
dfm7_dtheta=-g*cos(theta);
dfm7_dda_l=dfxbdda_l/mass;
dfm7_dda_r=dfxbdda_r/mass;
dfm7_ddc=dfxbddc/mass;
dfm7_dml=dfxbdml/mass;
dfm7_dmr=dfxbdmr/mass;
% dfm7_dmt=dfxbdmt/mass;

dfm8_dvx=dfybdvx/mass-o_z;
dfm8_dvy=dfybdvy/mass;
dfm8_dvz=dfybdvz/mass+o_x;
dfm8_dphi=g*cos(theta)*cos(phi);
dfm8_dtheta=-g*sin(theta)*sin(phi);
dfm8_dox=v_z;
dfm8_doy=0;
dfm8_doz=-v_x;
dfm8_dda_l=dfybdda_l/mass;
dfm8_dda_r=dfybdda_r/mass;
dfm8_ddc=dfybddc/mass;

dfm9_dvx=dfzbdvx/mass+o_y;
dfm9_dvy=dfzbdvy/mass-o_x;
dfm9_dvz=dfzbdvz/mass;
dfm9_dox=-v_y;
dfm9_doy=v_x;
dfm9_doz=0;
dfm9_dphi=-g*cos(theta)*sin(theta);
dfm9_dtheta=-g*cos(phi)*sin(theta);
dfm9_dda_l=dfzbdda_l/mass;
dfm9_dda_r=dfzbdda_r/mass;
dfm9_ddc=dfzbddc/mass;

%% Derivatives of the moment

dfsibdvx_l=[dfsibxdvx_l;dfsibydvx_l;dfsibzdvx_l];
dfsibdvy_l=[dfsibxdvy_l;dfsibydvy_l;dfsibzdvy_l];
dfsibdvz_l=[dfsibxdvz_l;dfsibydvz_l;dfsibzdvx_l];
dfsibdda_l=[dfsibxdda_l;dfsibydda_l;dfsibzdda_l];

dfsibdvx_r=[dfsibxdvx_r;dfsibydvx_r;dfsibzdvx_r];
dfsibdvy_r=[dfsibxdvy_r;dfsibydvy_r;dfsibzdvy_r];
dfsibdvz_r=[dfsibxdvz_r;dfsibydvz_r;dfsibzdvx_r];
dfsibdda_r=[dfsibxdda_r;dfsibydda_r;dfsibzdda_r];

dfcbdvx=[dfcbxdvx;dfcbydvx;dfcbzdvx];
dfcbdvy=[dfcbxdvy;dfcbydvy;dfcbzdvy];
dfcbdvz=[dfcbxdvz;dfcbydvz;dfcbzdvz];
dfcbddc=[dfcbxddc;dfcbyddc;dfcbzddc];

dfpbdvx_l=[dpbdvx_l;0;0];
dfpbdvy_l=[dpbdvy_l;0;0];
dfpbdvz_l=[dpbdvz_l;0;0];
dfpbddml=[dpbddml;0;0];

dfpbdvx_r=[dpbdvx_r;0;0];
dfpbdvy_r=[dpbdvy_r;0;0];
dfpbdvz_r=[dpbdvz_r;0;0];
dfpbddmr=[dpbddmr;0;0];

% dfpbdvx_t=[dpbdvx_t;0;0];
% dfpbdvy_t=[dpbdvy_t;0;0];
% dfpbdvz_t=[dpbdvz_t;0;0];
% dfpbddmt=[dpbddmt;0;0];


dmsidvx_l=cross(b2s_l,dfsibdvx_l);
dmsidvy_l=cross(b2s_l,dfsibdvy_l);
dmsidvz_l=cross(b2s_l,dfsibdvz_l);
dmsidda_l=cross(b2s_l,dfsibdda_l);

dmsidvx_r=cross(b2s_r,dfsibdvx_r);
dmsidvy_r=cross(b2s_r,dfsibdvy_r);
dmsidvz_r=cross(b2s_r,dfsibdvz_r);
dmsidda_r=cross(b2s_r,dfsibdda_r);

dmcdvx=cross(b2c,dfcbdvx);
dmcdvy=cross(b2c,dfcbdvy);
dmcdvz=cross(b2c,dfcbdvz);
dmcddc=cross(b2c,dfcbddc);

dmpdvx_l=cross(b2p_l,dfpbdvx_l);
dmpdvy_l=cross(b2p_l,dfpbdvy_l);
dmpdvz_l=cross(b2p_l,dfpbdvz_l);
dmpddml=cross(b2p_l,dfpbddml);

dmpdvx_r=cross(b2p_r,dfpbdvx_r);
dmpdvy_r=cross(b2p_r,dfpbdvy_r);
dmpdvz_r=cross(b2p_r,dfpbdvz_r);
dmpddmr=cross(b2p_r,dfpbddmr);

% dmpdvx_t=cross(b2p_t,dfpbdvx_t);
% dmpdvy_t=cross(b2p_t,dfpbdvy_t);
% dmpdvz_t=cross(b2p_t,dfpbdvz_t);
% dmpddmt=cross(b2p_t,dfpbddmt);

% dldvx=dmsidvx_l(1)+dmsidvx_r(1)+dmcdvx(1)+dmpdvx_l(1)+dmpdvx_r(1)+dmpdvx_t(1);
% dldvy=dmsidvy_l(1)+dmsidvy_r(1)+dmcdvy(1)+dmpdvy_l(1)+dmpdvy_r(1)+dmpdvy_t(1);
% dldvz=dmsidvz_l(1)+dmsidvz_r(1)+dmcdvz(1)+dmpdvz_l(1)+dmpdvz_r(1)+dmpdvz_t(1);
% dldda_l=dmsidda_l(1);
% dldda_r=dmsidda_r(1);
% dlddc=dmcddc(1);
% dlddml=dmpddml(1);
% dlddmr=dmpddmr(1);
% dlddmt=dmpddmt(1);

dldvx=dmsidvx_l(1)+dmsidvx_r(1)+dmcdvx(1)+dmpdvx_l(1)+dmpdvx_r(1);
dldvy=dmsidvy_l(1)+dmsidvy_r(1)+dmcdvy(1)+dmpdvy_l(1)+dmpdvy_r(1);
dldvz=dmsidvz_l(1)+dmsidvz_r(1)+dmcdvz(1)+dmpdvz_l(1)+dmpdvz_r(1);
dldda_l=dmsidda_l(1);
dldda_r=dmsidda_r(1);
dlddc=dmcddc(1);
dlddml=dmpddml(1);
dlddmr=dmpddmr(1);

% dmdvx=dmsidvx_l(2)+dmsidvx_r(2)+dmcdvx(2)+dmpdvx_l(2)+dmpdvx_r(2)+dmpdvx_t(2);
% dmdvy=dmsidvy_l(2)+dmsidvy_r(2)+dmcdvy(2)+dmpdvy_l(2)+dmpdvy_r(2)+dmpdvy_t(2);
% dmdvz=dmsidvz_l(2)+dmsidvz_r(2)+dmcdvz(2)+dmpdvz_l(2)+dmpdvz_r(2)+dmpdvz_t(2);
% dmdda_l=dmsidda_l(2);
% dmdda_r=dmsidda_r(2);
% dmddc=dmcddc(2);
% dmddml=dmpddml(2);
% dmddmr=dmpddmr(2);
% dmddmt=dmpddmt(2);

dmdvx=dmsidvx_l(2)+dmsidvx_r(2)+dmcdvx(2)+dmpdvx_l(2)+dmpdvx_r(2);
dmdvy=dmsidvy_l(2)+dmsidvy_r(2)+dmcdvy(2)+dmpdvy_l(2)+dmpdvy_r(2);
dmdvz=dmsidvz_l(2)+dmsidvz_r(2)+dmcdvz(2)+dmpdvz_l(2)+dmpdvz_r(2);
dmdda_l=dmsidda_l(2);
dmdda_r=dmsidda_r(2);
dmddc=dmcddc(2);
dmddml=dmpddml(2);
dmddmr=dmpddmr(2);

% dndvx=dmsidvx_l(3)+dmsidvx_r(3)+dmcdvx(3)+dmpdvx_l(3)+dmpdvx_r(3)+dmpdvx_t(3);
% dndvy=dmsidvy_l(3)+dmsidvy_r(3)+dmcdvy(3)+dmpdvy_l(3)+dmpdvy_r(3)+dmpdvy_t(3);
% dndvz=dmsidvz_l(3)+dmsidvz_r(3)+dmcdvz(3)+dmpdvz_l(3)+dmpdvz_r(3)+dmpdvz_t(3);
% dndda_l=dmsidda_l(3);
% dndda_r=dmsidda_r(3);
% dnddc=dmcddc(3);
% dnddml=dmpddml(3);
% dnddmr=dmpddmr(3);
% dnddmt=dmpddmt(3);

dndvx=dmsidvx_l(3)+dmsidvx_r(3)+dmcdvx(3)+dmpdvx_l(3)+dmpdvx_r(3);
dndvy=dmsidvy_l(3)+dmsidvy_r(3)+dmcdvy(3)+dmpdvy_l(3)+dmpdvy_r(3);
dndvz=dmsidvz_l(3)+dmsidvz_r(3)+dmcdvz(3)+dmpdvz_l(3)+dmpdvz_r(3);
dndda_l=dmsidda_l(3);
dndda_r=dmsidda_r(3);
dnddc=dmcddc(3);
dnddml=dmpddml(3);
dnddmr=dmpddmr(3);

dfm10_dvx=ga3*dldvx+ga4*dndvx;
dfm10_dvy=ga3*dldvy+ga4*dndvy;
dfm10_dvz=ga3*dldvz+ga4*dndvz;
dfm10_dox=ga1*o_y;
dfm10_doy=ga1*o_x-ga2*o_z;
dfm10_doz=-ga2*o_y;
dfm10_dda_l=ga3*dldda_l+ga4*dndda_l;
dfm10_dda_r=ga3*dldda_r+ga4*dndda_r;
dfm10_ddc=ga3*dlddc+ga4*dnddc;
dfm10_ddml=ga3*dlddml+ga4*dnddml;
dfm10_ddmr=ga3*dlddmr+ga4*dnddmr;
% dfm10_ddmt=ga3*dlddmt+ga4*dnddmt;

dfm11_dvx=dmdvx/ModelParams.Jyy;
dfm11_dvy=dmdvy/ModelParams.Jyy;
dfm11_dvz=dmdvz/ModelParams.Jyy;
dfm11_dox=-2*ga6*o_x+ga5*o_z;
dfm11_doy=0;
dfm11_doz=ga5*o_x+2*ga6*o_z;
dfm11_dda_l=dmdda_l/ModelParams.Jyy;
dfm11_dda_r=dmdda_r/ModelParams.Jyy;
dfm11_ddc=dmddc/ModelParams.Jyy;
dfm11_ddml=dmddml/ModelParams.Jyy;
dfm11_ddmr=dmddmr/ModelParams.Jyy;
% dfm11_ddmt=dmddmt/ModelParams.Jyy;

dfm12_dvx=ga4*dldvx+ga8*dndvx;
dfm12_dvy=ga4*dldvy+ga8*dndvy;
dfm12_dvz=ga4*dldvz+ga8*dndvz;
dfm12_dox=0;
dfm12_doy=-ga1*o_z+ga7*o_z;
dfm12_doz=-ga1*o_y+ga7*o_y;
dfm12_dda_l=ga4*dldda_l+ga8*dndda_l;
dfm12_dda_r=ga4*dldda_r+ga8*dndda_r;
dfm12_ddc=ga4*dlddc+ga8*dnddc;
dfm12_ddml=ga4*dlddml+ga8*dnddml;
dfm12_ddmr=ga4*dlddmr+ga8*dnddmr;
% dfm12_ddmt=ga4*dlddmt+ga8*dnddmt;


%% Jacobians

Ac=[0 0 0 dfm1_dphi dfm1_dtheta dfm1_dpsi dfm1_dvx dfm1_dvy dfm1_dvz 0 0 0;
    0 0 0 dfm2_dphi dfm2_dtheta dfm2_dpsi dfm2_dvx dfm2_dvy dfm2_dvz 0 0 0;
    0 0 0 dfm3_dphi dfm3_dtheta dfm3_dpsi dfm3_dvx dfm3_dvy dfm3_dvz 0 0 0;
    0 0 0 dfm4_dphi dfm4_dtheta 0 0 0 0 dfm4_dox dfm4_doy dfm4_doz;
    0 0 0 dfm5_dphi dfm5_dtheta 0 0 0 0 dfm5_dox dfm5_doy dfm5_doz;
    0 0 0 dfm6_dphi dfm6_dtheta 0 0 0 0 dfm6_dox dfm6_doy dfm6_doz;
    0 0 0 dfm7_dphi dfm7_dtheta 0 dfm7_dvx dfm7_dvy dfm7_dvz dfm7_dox dfm7_doy dfm7_doz;
    0 0 0 dfm8_dphi dfm8_dtheta 0 dfm8_dvx dfm8_dvy dfm8_dvz dfm8_dox dfm8_doy dfm8_doz;
    0 0 0 dfm9_dphi dfm9_dtheta 0 dfm9_dvx dfm9_dvy dfm9_dvz dfm9_dox dfm9_doy dfm9_doz;
    0 0 0 0 0 0 dfm10_dvx dfm10_dvy dfm10_dvz dfm10_dox dfm10_doy dfm10_doz;
    0 0 0 0 0 0 dfm11_dvx dfm11_dvy dfm11_dvz dfm11_dox dfm11_doy dfm11_doz;
    0 0 0 0 0 0 dfm12_dvx dfm12_dvy dfm12_dvz dfm12_dox dfm12_doy dfm12_doz];

% Bc=[0 0 0 0 0 0;
%     0 0 0 0 0 0;
%     0 0 0 0 0 0;
%     0 0 0 0 0 0;
%     0 0 0 0 0 0;
%     0 0 0 0 0 0;
%     dfm7_dda_l dfm7_dda_r dfm7_ddc dfm7_dml dfm7_dmr dfm7_dmt;
%     dfm8_dda_l dfm8_dda_r dfm8_ddc 0 0 0;
%     dfm9_dda_l dfm9_dda_r dfm9_ddc 0 0 0;
%     dfm10_dda_l dfm10_dda_r dfm10_ddc dfm10_ddml dfm10_ddmr dfm10_ddmt;
%     dfm11_dda_l dfm11_dda_r dfm11_ddc dfm11_ddml dfm11_ddmr dfm11_ddmt;
%     dfm12_dda_l dfm12_dda_r dfm12_ddc dfm12_ddml dfm12_ddmr dfm12_ddmt;];

Bc=[0 0 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0;
    dfm7_dda_l dfm7_dda_r dfm7_ddc dfm7_dml dfm7_dmr;
    dfm8_dda_l dfm8_dda_r dfm8_ddc 0 0;
    dfm9_dda_l dfm9_dda_r dfm9_ddc 0 0;
    dfm10_dda_l dfm10_dda_r dfm10_ddc dfm10_ddml dfm10_ddmr;
    dfm11_dda_l dfm11_dda_r dfm11_ddc dfm11_ddml dfm11_ddmr
    dfm12_dda_l dfm12_dda_r dfm12_ddc dfm12_ddml dfm12_ddmr;];

gc=fm-Ac*Xbar_k(1:sx)-Bc*Ubar_k(1:su);
Bc_aug=[Bc gc];



tmp = expm([Ac Bc_aug; zeros(su+1,sx+su+1)]*Ts);


tmp = expm([Ac Bc_aug; zeros(su+1,sx+su+1)]*Ts);

Ad = zeros(sx+1,sx+1);
Bd = zeros(sx+1,su+1);
gd = zeros(sx+1,1);
Ad(1:sx,1:sx) =tmp(1:sx,1:sx);
Bd(1:sx,1:su) =tmp(1:sx,sx+1:sx+su);
gd(1:sx) =tmp(1:sx,sx+su+1);

% following to avoid numerical errors
Ad(end,end)=1;
Bd(end,end)=Ts;


end




















