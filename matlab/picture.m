
clear
% 定义 l0 和 phi0 的范围
l0_range = linspace(0.13, 0.24, 50); % 0 到 10 之间取 100 个点
phi0_range = linspace(0.61, 2.40, 50); % 0 到 2*pi 之间取 100 个点

% 生成 l0 和 phi0 的网格
[l0_grid, phi0_grid] = meshgrid(l0_range, phi0_range);

% 计算对应的函数值
z  = JointPos(l0_grid, phi0_grid);

% 将复数值置为 0
z(imag(z) ~= 0) = 0;
z1 = z(1:50, 1:50);
z2 = z(51:100, 1:50);

% 绘制三维图像
figure;% 创建一个新的图形窗口
surf(l0_grid, phi0_grid, z1);
xlabel('l0');
ylabel('phi0');
zlabel('关节设定角度');
title('关节1的设定角度和l0,phi0的关系');


figure;% 创建一个新的图形窗口
surf(l0_grid, phi0_grid, z2);
xlabel('l0');
ylabel('phi0');
zlabel('关节设定角度');
title('关节2的设定角度和l0,phi0的关系');
