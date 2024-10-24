% 本程序用于求解LQR反馈矩阵lqr_k(L0)
% 对于每一个腿长L0，求解一次系统状态空间方程，然后求得反馈矩阵K
% 对于不同的K，对L0进行拟合，得到lqr_k

clear;

L0s=0.13:0.01:0.24; % L0变化范围
Ks=zeros(2,6,length(L0s)); % 存放不同L0对应的K

for step=1:length(L0s)
    
    % 所需符号量
    syms theta theta1 theta2; % theta1=dTheta, theta2=ddTheta
    syms x x1 x2;
    syms phi phi1 phi2;
    syms T Tp N P Nm Pm Nf t;
    
    % 机器人结构参数(国际单位制)
    R=0.0425; L=L0s(step)/2; Lm=L0s(step)/2; l=0; mw=0.158; mp=0.130; M=2.250;
    Iw=0.001188791+0.001471158; 
    Ip=(0.000004161+0.00008696823)*2; 
    Im=0.01884243947;
    g=9.8;
    
    % 进行物理计算
    Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
    Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
    N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
    P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));
    
    equ1=x2-(T-N*R)/(Iw/R+mw*R);
    equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
    equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
    [x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);
    
    % 求得雅克比矩阵，然后得到状态空间方程
    Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
    Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
    A=vpa(subs(Ja,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
    B=vpa(subs(Jb,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
    
    % 离散化
    [G,H]=c2d(eval(A),eval(B),0.002);%最后一个参数是离散化的步长(s)
    
    % 定义权重矩阵Q, R
    Q=diag([1 10 100 20 1000 1]);
    R=diag([1 1]);

    % 求解反馈矩阵K
    Ks(:,:,step)=dlqr(G,H,Q,R);

end

% 对K的每个元素关于L0进行拟合
K=sym('K',[2 6]);
syms L0;
for x=1:2
    for y=1:6
        p=polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
        K(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end

% 输出到m函数
matlabFunction(K,'File','LQR_K');

% 代入L0=0.07打印矩阵K
vpa(subs(K,L0,0.07))

disp("LQR模型函数生成完毕");