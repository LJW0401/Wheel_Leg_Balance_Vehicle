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

cos_phi11 = ((l5/2)^2+Lca2-l0^2)/(l5*sqrt(Lca2));
cos_phi12 = (l1^2+Lca2-l2^2)/(2*l1*sqrt(Lca2));
cos_phi41 = ((l5/2)^2+Lce2-l0^2)/(l5*sqrt(Lce2));
cos_phi42 = (l4^2+Lce2-l3^2)/(2*l4*sqrt(Lce2));

%if abs(cos_phi11)<=1 && abs(cos_phi12)<=1 && abs(cos_phi41)<=1 && abs(cos_phi42)<=1
    phi11 = acos(cos_phi11);
    phi12 = acos(cos_phi12);

    phi41 = acos(cos_phi41);
    phi42 = acos(cos_phi42);
%else
%    phi11 = 0;phi12 = 0;phi41 = 0;phi42 = 0;
%end

phi1 = phi11 + phi12;
phi4 = pi-(phi41 + phi42);

% 求得关节角度 [phi1; phi4] = JointPos(l0, phi0)
pos=[phi1; phi4];
matlabFunction(pos,'File','JointPos');

disp("腿部反几何学解算生成完毕");