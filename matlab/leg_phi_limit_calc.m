% 本程序用于计算腿部角度限幅大小，保证关节解算有实数解

clear;
clc;
% 设定所需符号
syms l0;
l1=0.095;l2=0.19;l3=l2;l4=l1;l5=0.095; % 腿部杆长(单位：m)

%求解下限
min = acos(((l1+l2)^2-l0^2-(l5/2)^2)/(l0*l5));
%求解上限
max = acos(((l3+l4)^2-l0^2-(l5/2)^2)/(-l0*l5));

% 求得腿部姿态 [min; max] = PhiLimit(l0)
limit=[min; max];
matlabFunction(limit,'File','LegAngleLimit');


disp("角度限幅函数生成完毕");

l0_range = linspace(0.13, 0.24, 50);
z  = LegAngleLimit(l0_range);