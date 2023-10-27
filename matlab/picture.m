
clear
% 定义 l0 和 phi0 的范围
l0_range = linspace(0.14, 0.2, 100); % 0 到 10 之间取 100 个点
phi0_range = linspace(pi/6, pi*5/6, 100); % 0 到 2*pi 之间取 100 个点

% 生成 l0 和 phi0 的网格
[l0_grid, phi0_grid] = meshgrid(l0_range, phi0_range);

% 计算对应的函数值
z  = JointPos(l0_grid, phi0_grid);
z = z(101:200, 1:100);

% 将复数值置为 0
z(imag(z) ~= 0) = 0;

% 绘制三维图像
surf(l0_grid, phi0_grid, z);
xlabel('l0');
ylabel('phi0');
zlabel('f(l0, phi0)');
title('Graph of the Expression');
