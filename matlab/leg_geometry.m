% 本程序用于腿部结构的几何反向解算，通过腿长l0和腿部角度phi0反推出电机位置phi1,phi4(JointPos)
clear;

% 设定所需符号
syms phi1 phi2 phi3 phi4;
syms dphi1 dphi4;
l1=0.095;l2=0.19;l3=l2;l4=l1;l5=0.095; % 腿部杆长(单位：m)
syms xc yc xb yb xd yd; % c,b,d三点的xy坐标
syms l0 phi0;

%进行几何计算
Lca2 = l0^2+(l5/2)^2+l0*l5*cos(phi0);
Lce2 = l0^2+(l5/2)^2-l0*l5*cos(phi0);

phi11 = acos(((l5/2)^2+Lca2-l0^2)/(l5*sqrt(Lca2)));
phi12 = acos((l1^2+Lca2-l2^2)/(2*l1*sqrt(Lca2)));

phi41 = acos(((l5/2)^2+Lce2-l0^2)/(l5*sqrt(Lce2)));
phi42 = acos((l4^2+Lce2-l3^2)/(2*l4*sqrt(Lce2)));

phi1 = phi11 + phi12;
phi4 = phi41 + phi42;

% 求得关节角度 [phi1; phi4] = JointPos(l0, phi0)
pos=[phi1; phi4];
matlabFunction(pos,'File','JointPos');

disp("腿部反几何学解算生成完毕");