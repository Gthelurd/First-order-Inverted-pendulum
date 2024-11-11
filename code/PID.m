%%% author:廖洽源 %%%
% https://zhuanlan.zhihu.com/p/54071212
%Modeling
%%Transfer Function
mCart = 1.42;  
mPend = 0.12;
b = 0.1;     % 阻尼系数
I = 0.014;     % 转动惯量
g = 9.8;
L = 0.188;
q = (mCart+mPend)*(I+mPend*L^2)-(mPend*L)^2;
s = tf('s');
P_cart = (((I+mPend*L^2)/q)*s^2 - (mPend*g*L/q))/(s^4 + (b*(I + mPend*L^2))*s^3/q - ((mCart + mPend)*mPend*g*L)*s^2/q - b*mPend*g*L*s/q);
P_pend = (mPend*L*s/q)/(s^3 + (b*(I + mPend*L^2))*s^2/q - ((mCart + mPend)*mPend*g*L)*s/q - b*mPend*g*L/q);
sys_tf = [P_cart ; P_pend]
inputs = {'u'};
outputs = {'x'; 'phi'};
set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

t = 0:0.05:5;
u = ones(size(t));
[y,t] = lsim(sys_tf,u,t);
figure ;
plot(t,y) ;
title('Open-Loop Step Response') ;
% axis([0 3 0 50]) ;
legend('x','phi');

% PID环节
Kp = 100;
Ki = 1;
Kd = 30;
C = pid(Kp,Ki,Kd);
T1 = feedback(P_pend,C);
t=0:0.01:4;
figure;
impulse(T1,t)
title('Close-loop impluse Response')
legend('phi');
figure;
T2 = feedback(1,P_pend*C)*P_cart;
t = 0:0.01:5;
impulse(T2, t);
title('Close-loop impluse Response')
legend('x');

