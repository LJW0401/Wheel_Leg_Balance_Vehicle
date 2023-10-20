% 定义匿名函数
f = @(l0, phi0) (200*((19*l0*cos(phi0))/200 + 5202558289538397/1152921504606846976))/(19*(l0^2 + (19*cos(phi0)*l0)/200 + 5202558289538397/2305843009213693952)^(1/2));

% 定义 l0 和 phi0 的范围
l0_range = linspace(0, 0.4, 1000); % 0 到 10 之间取 100 个点
phi0_range = linspace(0, pi, 1000); % 0 到 2*pi 之间取 100 个点

% 生成 l0 和 phi0 的网格
[l0_grid, phi0_grid] = meshgrid(l0_range, phi0_range);

% 计算对应的函数值
z = f(l0_grid, phi0_grid);

% 将复数值置为 NaN
z(imag(z) ~= 0) = NaN;

% 绘制三维图像
surf(l0_grid, phi0_grid, real(z));
xlabel('l0');
ylabel('phi0');
zlabel('f(l0, phi0)');
title('Graph of the Expression');
