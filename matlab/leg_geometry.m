% 本程序用于腿部结构的几何解算，用电机所能获取的phi1,phi4计算腿长l0和腿部角度phi0(LegPos)
% 并通过腿长l0和腿部角度phi0反推出电机位置phi1,phi4(JointPos)
clear;

% 设定所需符号
syms phi1 phi2 phi3 phi4;
syms dphi1 dphi4;
l1=0.095;l2=0.19;l3=l2;l4=l1;l5=0.095; % 腿部杆长(单位：m)
syms xc yc xb yb xd yd; % c,b,d三点的xy坐标

%进行几何计算
xb=l1*cos(phi1);
yb=l1*sin(phi1);
xd=l5+l4*cos(phi4);
yd=l4*sin(phi4);

A0=2*l2*(xd-xb);
B0=2*l2*(yd-yb);
C0=l2^2+(xd-xb)^2+(yd-yb)^2-l3^2;
phi2=2*atan((B0+sqrt(A0^2+B0^2-C0^2))/(A0+C0));

xc=xb+l2*cos(phi2);
yc=yb+l2*sin(phi2);

l0=sqrt((xc-l5/2)^2+yc^2);
phi0=atan2(yc,(xc-l5/2));

% 求得腿部姿态 [l0; phi0] = leg_pos(phi1, phi4)
pos=[l0; phi0];
matlabFunction(pos,'File','LegPos');


%进行几何计算
syms l0 phi0;
eq1 = l0==sqrt((xc-l5/2)^2+yc^2);
eq2 = phi0==atan2(yc,(xc-l5/2));

eq3 = xc==xb+l2*cos(phi2);
eq4 = yc==yb+l2*sin(phi2);

eq5 = A0==2*l2*(xd-xb);
eq6 = B0==2*l2*(yd-yb);
eq7 = C0==l2^2+(xd-xb)^2+(yd-yb)^2-l3^2;
eq8 = phi2==2*atan((B0+sqrt(A0^2+B0^2-C0^2))/(A0+C0));

eq8 = xb==l1*cos(phi1);
eq9 = yb==l1*sin(phi1);
eq10 = xd==l5+l4*cos(phi4);
eq11 = yd==l4*sin(phi4);

sol = solve(eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq7,eq9,eq10,eq11, phi1,phi4);
sol;




