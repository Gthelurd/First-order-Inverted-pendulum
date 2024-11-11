% Modeling
%% Transfer Function
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
sys_tf = [P_cart ; P_pend];
inputs = {'u'};
outputs = {'x'; 'phi'};
set(sys_tf,'InputName',inputs);
set(sys_tf,'OutputName',outputs);
%% PID 控制器设计
% 角度控制器
Kp_phi = 100;
Ki_phi = 1;
Kd_phi = 30;
C_phi = pid(Kp_phi, Ki_phi, Kd_phi);
% 位移控制器
Kp_x = 10;
Ki_x = 0.1;
Kd_x = 0.3;
C_x = pid(Kp_x, Ki_x, Kd_x);
%% 闭环系统
% 角度闭环系统
T1 = feedback(P_pend, C_phi);
% 位移闭环系统
T2 = feedback(1, P_pend * C_phi) * P_cart;
T2 = feedback(T2, C_x);

%% 仿真
t = 0:0.01:5;
% 角度响应
figure;
subplot(2, 1, 1);
impulse(T1, t);
ylim([-0.05, 0.05]);
title('角度响应');
grid on;
% 位移响应
subplot(2, 1, 2);
impulse(T2, t);
xlim([0, 1.5]);
title('位移响应');
grid on;
set(gcf, 'Position', [100, 100, 800, 600]);