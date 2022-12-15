function u = Pi_test(x,dt,i)
persistent state_prev;
persistent I_term;
global de_att

if isempty(state_prev)
    state_prev = [x(10:12);x(7)];
end

if isempty(I_term)
    I_term = zeros(4,1);
end

Control_Mat=[0,0.1,0,0;0.5,-0.5,0,0;-0.5,-0.5,0,0;0,0,0.2,1;0,0,-0.2,1];
states_d=[0;0;0;10];
states = [x(10:12);x(7)];
states_e = states_d-states;
de_att=states_e;

% roll pitch yaw vel P term
P_param=[0.05;0.05;0.05;0.5];
P_term = P_param.*states_e;

D_param = [0.001;0.001;0;0.01];
D_term = D_param.*(states-state_prev)/dt;
state_prev = [x(10);x(11);x(12);x(7)];

I_param = [0.01;0.01;0.01;0.01];
I_term = I_term + I_param.*states_e;

% constrain the I_term

input = Control_Mat*(P_term+I_term+D_term);

if input(4)<0.05
    input(4)=0.05;
end

if input(5)<0.05
    input(5)=0.05;
end

u=zeros(5,1);
u(1)=input(2);
u(2)=input(3);
u(3)=input(1);
u(4)=input(4);
u(5)=input(5);

roll = x(4);
pitch = x(5);
yaw = x(6);
vel = x(7);
end



