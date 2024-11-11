% 定义状态矩阵 A 和控制矩阵 B 观测矩阵 C 和输入矩阵 D
A = [0, 1, 0, 0;
     41.63, 0, 0, 0;
     0, 0, 0, 1;
     -0.6099, 0,0 , 0];
B = [0;
     -2.7584;
     0;
     0.6898];
C = [0 0 1 0];
D = 0;
G =ss(A,B,C,D)
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
figure;
subplot(2, 1, 1);
plot(t, x(:, 1), t, x(:, 2));
grid on;
xlabel('time(s)', 'FontSize', 20);
ylabel('{\theta and \dot \theta}', 'FontSize', 20);
title('摆角与角速度响应', 'FontSize', 25);
legend('摆角', '角速度');
subplot(2, 1, 2);
plot(t, x(:, 3), t, x(:, 4));
grid on;
xlabel('time(s)', 'FontSize', 20);
ylabel('{x and \dot x}', 'FontSize', 20);
title('位移与速度响应', 'FontSize', 25);
legend('位移', '速度');
