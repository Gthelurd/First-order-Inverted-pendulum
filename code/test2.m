% 定义状态矩阵 A 和控制矩阵 B 观测矩阵 C 和输入矩阵 D
A = [0, 1, 0, 0;
     41.63, 0, 0, 0;
     0, 0, 0, 1;
     -0.6099, 0, 0, 0];
B = [0;
     -2.7584;
     0;
     0.6898];
C = [1, 0, 0, 0;
     0, 0, 1, 0];
D = 0;
G = ss(A, B, C, D);
% 定义权重矩阵 Q 和 R
Q = eye(4);  % 状态权重矩阵，这里使用单位矩阵
R = 1;       % 控制权重矩阵，这里使用 1
[K, S, E] = lqr(A, B, Q, R);
B
A
K
% 构建闭环系统
A_cl = A - B * K;
B_cl = B;
C_cl = C;
D_cl = D;
Gclose = ss(A_cl, B_cl, C_cl, D_cl);

% 计算并绘制阶跃响应
t = 0:0.01:10;
[y, t, x] = step(Gclose, t);

figure;
% 摆角与角速度响应
subplot(2, 1, 1);
plot(t, x(:, 1), 'b-', 'LineWidth', 2);
hold on;
plot(t, x(:, 2), 'r--', 'LineWidth', 2);
grid on;
xlabel('time(s)', 'FontSize', 20);
ylabel('{\theta and \dot \theta}', 'FontSize', 20);
title('摆角与角速度响应', 'FontSize', 25);
legend('摆角', '角速度');
% 位移与速度响应
subplot(2, 1, 2);
plot(t, x(:, 3), 'g-', 'LineWidth', 2);
hold on;
plot(t, x(:, 4), 'm--', 'LineWidth', 2);
grid on;
xlabel('time(s)', 'FontSize', 20);
ylabel('{x and \dot x}', 'FontSize', 20);
title('位移与速度响应', 'FontSize', 25);
legend('位移', '速度');
set(gcf, 'Position', [100, 100, 800, 600]);