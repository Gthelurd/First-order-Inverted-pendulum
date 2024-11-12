%%% author:Mworks-同源软控%%%
% https://zhuanlan.zhihu.com/p/565861449
% 倒立摆系统参数
M = 1.42;     % 小车质量
m = 0.12;     % 摆球质量
l = 0.188;    % 摆杆长度
I = 0.0014;     % 摆球转动惯量
g = 9.8;      % 重力加速度9.8m/s^2
q = (M+m)*(I+m*l^2)-(m*l)^2; %0.0082
p = I*(M+m)+(M*m*l^2);       %0.0082
% 倒立摆系统状态空间矩阵定义
A = [0               1 0 0
    (M+m)*m*g*l/(I*(M+m)+(M*m*l^2))    0 0 0
    0                0 0 1
    -m^2*g*l^2 /(I*(M+m)+(M*m*l^2))          0 0 0];
B = [0 -m*l/(I*(M+m)+(M*m*l^2)) 0 (I+m*l^2)/(I*(M+m)+(M*m*l^2))]';
C = [0 0 1 0];
% C = [1 0 0 0
%     0 0 1 0];
D = 0;
% 倒立摆系统状态空间模型
G = ss(A, B, C, D)
% 计算理想主导极点位置并分配系统所有极点位置
Ahat = [A ,zeros(size(A,1),1);-C,0];
Bhat = [B;0];
r=rank([Ahat,Bhat]);
idealPoles = roots([1, 2 * 0.7 * 1, 1]);
AllPoles = [idealPoles(1), idealPoles(2), -10, -15, -20];
% AllPoles = [idealPoles(1), idealPoles(2), -10, -15];
% 极点配置，计算状态增益K
Khat = place(Ahat, Bhat, AllPoles);
% 状态增益值分解
K = real(Khat(1:4))
kl = -real(Khat(end))
% 闭环系统状态方程
AA = [A - B*K, B*kl; -C, 0];
BB = [0; 0; 0; 0; 1];
CC = [C, 0];
DD = 0;
Gclose = ss(AA, BB, CC, DD);
% 计算并绘制阶跃响应
[y, t, x] = step(Gclose, 10);
% 绘制阶跃响应图
% 摆角与角速度响应
subplot(2, 1, 1);
plot(t, x(:, 1), 'b-', 'LineWidth', 2);
hold on;
plot(t, x(:, 2), 'r--', 'LineWidth', 2);
grid on;
xlabel('time(s)', 'FontSize', 20);
ylabel('{\theta and \dot \theta}', 'FontSize', 20);
title('摆角与角速度响应', 'FontSize', 25);
legend('摆角', '角速度', 'Location', 'best');

% 位移与速度响应
subplot(2, 1, 2);
plot(t, x(:, 3), 'g-', 'LineWidth', 2);
hold on;
plot(t, x(:, 4), 'm--', 'LineWidth', 2);
grid on;
xlabel('time(s)', 'FontSize', 20);
ylabel('{x and \dot x}', 'FontSize', 20);
title('位移与速度响应', 'FontSize', 25);
legend('位移', '速度', 'Location', 'best');

% 调整图形整体布局
set(gcf, 'Position', [100, 100, 800, 600]);