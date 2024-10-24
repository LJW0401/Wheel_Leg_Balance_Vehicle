x_length = length(L0s);
X=L0s;
Y=zeros(x_length,1);
for i=1:length(L0s)
    Y(i)=Ks(1,3,i);
end
plot(X,Y);
hold on;

for i=1:length(L0s)
    L_i = L0s(i);
    K=LQR_K(L_i);
    Y(i)=K(1,3);
end

scatter(X,Y);