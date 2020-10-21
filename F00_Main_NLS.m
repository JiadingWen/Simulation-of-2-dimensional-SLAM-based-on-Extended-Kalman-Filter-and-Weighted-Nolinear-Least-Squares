%% 2D SLAM NLS

clc
clear
close all

%% 生成数据

% run F01_GenerateData.m

%% 加载数据

% 观测数据 控制数据 时间周期等
load M_NormalData  % 五列分别为:节点1 时间节点2 时间周期3 控制输入 4~5 观测数据1 6~7 观测数据2 8~9
load M_RealData 

%% 加载噪音

% 假设我们能获悉以下参数
mean_noise = 0; % 噪音的均值
StandardDeviation_noise = 0.1; % 噪音的标准差
variance_noise = StandardDeviation_noise^2; % 噪音的方差

%% 加载初值

% 设置Robot初始位置为原点
x_r_0 = 0;
y_r_0 = 0;
phi_r_0 = 0;

%% 设置NLS参数

% 最大循环次数
MaxIteration = 1000;

% 最大收敛误差
MaxError = 0.000000000000001;

%% 初始化

% 获取初始观测数据
r_f1_0 = NormalData(1,6);
theta_f1_0 = NormalData(1,7);
r_f2_0 = NormalData(1,8);
theta_f2_0 = NormalData(1,9);

% PZ
PZ = diag([0.0000001 0.0000001 0.0000001 variance_noise variance_noise variance_noise variance_noise]);

% 循环初始输入
X_NLS_00 = [0.000001; 0.000001; 0.000001;  ...
    1; 1; 1; 1;];
X0 = X_NLS_00;

% 初始化WNLS循环
for i = 1:MaxIteration

    % 
    x_r_0 = X0(1,1);
    y_r_0 = X0(2,1);
    phi_r_0 = X0(3,1);
    x_f1 = X0(4,1);
    y_f1 = X0(5,1);
    x_f2 = X0(6,1);
    y_f2 = X0(7,1);
    
    % A
    A = [1 0 0 0 0 0 0
        0 1 0 0 0 0 0
        0 0 1 0 0 0 0
        ((x_f1 - x_r_0)^2 + (y_f1 - y_r_0)^2)^(-0.5)*(x_f1 - x_r_0)*(-1)  ...
        ((x_f1 - x_r_0)^2 + (y_f1 - y_r_0)^2)^(-0.5)*(y_f1 - y_r_0)*(-1)  ...
        0 ((x_f1 - x_r_0)^2 + (y_f1 - y_r_0)^2)^(-0.5)*(x_f1 - x_r_0)  ...
        ((x_f1 - x_r_0)^2 + (y_f1 - y_r_0)^2)^(-0.5)*(y_f1 - y_r_0) 0 0
        
        (y_f1 - y_r_0)/((x_f1 - x_r_0)^2 + (y_f1 - y_r_0)^2)  ...
        -(x_f1 - x_r_0)/((x_f1 - x_r_0)^2 + (y_f1 - y_r_0)^2) -1  ...
        -(y_f1 - y_r_0)/((x_f1 - x_r_0)^2 + (y_f1 - y_r_0)^2)  ...
        (x_f1 - x_r_0)/((x_f1 - x_r_0)^2 + (y_f1 - y_r_0)^2) 0 0
        
        ((x_f2 - x_r_0)^2 + (y_f2 - y_r_0)^2)^(-0.5)*(x_f2 - x_r_0)*(-1)  ...
        ((x_f2 - x_r_0)^2 + (y_f2 - y_r_0)^2)^(-0.5)*(y_f2 - y_r_0)*(-1) 0 0 0  ...
        ((x_f2 - x_r_0)^2 + (y_f2 - y_r_0)^2)^(-0.5)*(x_f2 - x_r_0)  ...
        ((x_f2 - x_r_0)^2 + (y_f2 - y_r_0)^2)^(-0.5)*(y_f2 - y_r_0)
        
        (y_f2 - y_r_0)/((x_f2 - x_r_0)^2 + (y_f2 - y_r_0)^2)  ...
        -(x_f2 - x_r_0)/((x_f2 - x_r_0)^2 + (y_f2 - y_r_0)^2)  ...
        -1 0 0  ...
        -(y_f2 - y_r_0)/((x_f2 - x_r_0)^2 + (y_f2 - y_r_0)^2)  ...
        (x_f2 - x_r_0)/((x_f2 - x_r_0)^2 + (y_f2 - y_r_0)^2)];
    
    % Z
    Z = [0; 0; 0; r_f1_0; theta_f1_0; r_f2_0; theta_f2_0;];
    
    % F(X0)
    F_X0 = [x_r_0;
        y_r_0;
        phi_r_0;
        ((x_f1 - x_r_0)^2 + (y_f1 - y_r_0)^2)^(0.5);
        S_Wrap(atan2((y_f1 - y_r_0), (x_f1 - x_r_0)) - phi_r_0);
        ((x_f2 - x_r_0)^2 + (y_f2 - y_r_0)^2)^(0.5);
        S_Wrap(atan2((y_f2 - y_r_0), (x_f2 - x_r_0)) - phi_r_0);];
    
    % Z_F_X0
    Z_F_X0 = [Z(1,1) - F_X0(1,1); Z(2,1) - F_X0(2,1); S_Wrap(Z(3,1) - F_X0(3,1));  ...
        Z(4,1) - F_X0(4,1); S_Wrap(Z(5,1) - F_X0(5,1));  ...
        Z(6,1) - F_X0(6,1); S_Wrap(Z(7,1) - F_X0(7,1));];
    
    % b
    b = Z_F_X0 + A*X0;
    
    % X1
    X1 = pinv(A'*pinv(PZ)*A)*A'*pinv(PZ)*b;
    
    % P1
    P1 = pinv(A'*pinv(PZ)*A);
    
    % 判断收敛
    if ((X0 - X1)'*(X0 - X1)) < MaxError
        X_NLS_00 = X1;
        P_NLS_00 = P1;
        break 
    end

    % 为下一次循环做准备
    X0 = X1;
    
end

clear PZ

% 可视化
figure(1)
xlim([-10 17])
ylim([-10 20])
hold on
plot(X_NLS_00(1,1),X_NLS_00(2,1),'ro');
length = 0.5;
plot([X_NLS_00(1,1),X_NLS_00(1,1) + length*cos(X_NLS_00(3,1))],[X_NLS_00(2,1), X_NLS_00(2,1) + length*sin(X_NLS_00(3,1))],'r')
% real feature
plot(RealData(1,11),RealData(1,12),'b*');
plot(RealData(1,17),RealData(1,18),'b*');
hold off

%% NLS化

% 循环初始输入
X_NLS_KK = X_NLS_00;
P_NLS_KK = P_NLS_00;

% T
T = NormalData(i,3);

% 预分配内存
PZ = zeros(14,14);

for j = 1:(size(NormalData,1) - 1)
    % 
    x_r_k = X_NLS_KK(1,1);
    y_r_k = X_NLS_KK(2,1);
    phi_r_k = X_NLS_KK(3,1);
    x_f1 = X_NLS_KK(4,1);
    y_f1 = X_NLS_KK(5,1);
    x_f2 = X_NLS_KK(6,1);
    y_f2 = X_NLS_KK(7,1);
    
    % 
    vk = NormalData(j,4);
    wk = NormalData(j,5);
    
    % 
    r_f1_k1 = NormalData(j+1,6);
    theta_f1_k1 = NormalData(j+1,7);
    r_f2_k1 = NormalData(j+1,8);
    theta_f2_k1 = NormalData(j+1,9);
    
    % Z
    Z = [x_r_k y_r_k phi_r_k x_f1 y_f1 x_f2 y_f2  ...
        vk 0 wk  ...
        r_f1_k1 theta_f1_k1 r_f2_k1 theta_f2_k1]';
    
    % X0
    X0([1 2 3 7 8 9 10],1) = X_NLS_KK;
    X0([4 5 6],1) = [x_r_k + vk*T*cos(phi_r_k);
        y_r_k + vk*T*sin(phi_r_k);
        S_Wrap(phi_r_k + wk*T);];
    
    % PZ
    PZ([1 2 3 4 5 6 7],[1 2 3 4 5 6 7]) = P_NLS_KK;
    PZ([8 9 10],[8 9 10]) = diag([variance_noise 0.0000001 variance_noise]);
    PZ([11 12 13 14],[11 12 13 14]) = diag([variance_noise variance_noise variance_noise variance_noise]);
    
    for i = 1:MaxIteration
        % 
        x_r_k = X0(1,1);
        y_r_k = X0(2,1);
        phi_r_k = X0(3,1);
        x_r_k1 = X0(4,1);
        y_r_k1 = X0(5,1);
        phi_r_k1 = X0(6,1);
        x_f1 = X0(7,1);
        y_f1 = X0(8,1);
        x_f2 = X0(9,1);
        y_f2 = X0(10,1);
        
        % A
        A = [1 0 0  ...
            0 0 0  ...
            0 0 0 0;
            
            0 1 0  ...
            0 0 0  ...
            0 0 0 0;
            
            0 0 1  ...
            0 0 0  ...
            0 0 0 0;
            
            0 0 0  ...
            0 0 0  ...
            1 0 0 0;
            
            0 0 0  ...
            0 0 0  ...
            0 1 0 0;
            
            0 0 0  ...
            0 0 0  ...
            0 0 1 0;
            
            0 0 0  ...
            0 0 0  ...
            0 0 0 1;
            
            (1/T)*((x_r_k1 - x_r_k)^2 + (y_r_k1 - y_r_k)^2)^(-0.5)*(x_r_k1 - x_r_k)*(-1)  ...
            (1/T)*((x_r_k1 - x_r_k)^2 + (y_r_k1 - y_r_k)^2)^(-0.5)*(y_r_k1 - y_r_k)*(-1)  ...
            0  ...
            (1/T)*((x_r_k1 - x_r_k)^2 + (y_r_k1 - y_r_k)^2)^(-0.5)*(x_r_k1 - x_r_k)  ...
            (1/T)*((x_r_k1 - x_r_k)^2 + (y_r_k1 - y_r_k)^2)^(-0.5)*(y_r_k1 - y_r_k)  ...
            0  ...
            0 0 0 0;
            
            -sin(phi_r_k) cos(phi_r_k) (x_r_k1 - x_r_k)*cos(phi_r_k) + (y_r_k1 - y_r_k)*sin(phi_r_k)  ...
            sin(phi_r_k) -cos(phi_r_k) 0  ...
            0 0 0 0;
            
            0 0 -(1/T)  ...
            0 0 (1/T)  ...
            0 0 0 0;
            
            0 0 0  ...
            ((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(-0.5)*(x_f1 - x_r_k1)*(-1)  ...
            ((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(-0.5)*(y_f1 - y_r_k1)*(-1)  ...
            0  ...
            ((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(-0.5)*(x_f1 - x_r_k1)  ...
            ((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(-0.5)*(y_f1 - y_r_k1)  ...
            0 0;
            
            0 0 0  ...
            (y_f1 - y_r_k1)/((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)  ...
            -(x_f1 - x_r_k1)/((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)  ...
            -1  ...
            -(y_f1 - y_r_k1)/((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)  ...
            (x_f1 - x_r_k1)/((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)  ...
            0 0;
            
            0 0 0  ...
            ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(-0.5)*(x_f2 - x_r_k1)*(-1)  ...
            ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(-0.5)*(y_f2 - y_r_k1)*(-1)  ...
            0  ...
            0 0  ...
            ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(-0.5)*(x_f2 - x_r_k1)  ...
            ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(-0.5)*(y_f2 - y_r_k1);
            
            0 0 0  ...
            (y_f2 - y_r_k1)/((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)  ...
            -(x_f2 - x_r_k1)/((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)  ...
            -1  ...
            0 0  ...
            -(y_f2 - y_r_k1)/((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)  ...
            (x_f2 - x_r_k1)/((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2);
            ];
        
        % F(X0)
        F_X0 = [x_r_k;
            y_r_k;
            phi_r_k;
            x_f1;
            y_f1;
            x_f2;
            y_f2;
            
            (1/T)*((x_r_k1 - x_r_k)^2 + (y_r_k1 - y_r_k)^2)^(0.5);
            (x_r_k1 - x_r_k)*sin(phi_r_k) - (y_r_k1 - y_r_k)*cos(phi_r_k);
            (1/T)*(phi_r_k1 - phi_r_k);
            
            ((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(0.5);
            S_Wrap(atan2((y_f1 - y_r_k1), (x_f1 - x_r_k1)) - phi_r_k1);
            ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(0.5);
            S_Wrap(atan2((y_f2 - y_r_k1), (x_f2 - x_r_k1)) - phi_r_k1);];
        
        % Z - F(X0)
        Z_F_X0 = [
            Z(1,1) - F_X0(1,1); 
            Z(2,1) - F_X0(2,1); 
            S_Wrap(Z(3,1) - F_X0(3,1));
            Z(4,1) - F_X0(4,1); 
            Z(5,1) - F_X0(5,1); 
            Z(6,1) - F_X0(6,1); 
            Z(7,1) - F_X0(7,1); 
            
            Z(8,1) - F_X0(8,1); 
            Z(9,1) - F_X0(9,1);
            (1/T)*S_Wrap(T*(Z(10,1) - F_X0(10,1)));
            
            Z(11,1) - F_X0(11,1); 
            S_Wrap(Z(12,1) - F_X0(12,1));
            Z(13,1) - F_X0(13,1); 
            S_Wrap(Z(14,1) - F_X0(14,1));];
        
        % b
        b = Z_F_X0 + A*X0;
        
        % X1
        X1 = pinv(A'*pinv(PZ)*A)*A'*pinv(PZ)*b;
        
        % P1
        P1 = pinv(A'*pinv(PZ)*A);
        
        if ((X0 - X1)'*(X0 - X1)) < MaxError
            
            break
        end
        
        % 为下一次循环做好准备
        X0 = X1;
        
    end
    
    %% 为下一次循环做好准备 
    
    X_NLS_KK = X1([4 5 6 7 8 9 10],1);
    P_NLS_KK = P1([4 5 6 7 8 9 10],[4 5 6 7 8 9 10]);
    
    %% 可视化
    
    % real feature
    plot(RealData(1,11),RealData(1,12),'b*');
    hold on
    xlim([-10 17])
    ylim([-10 20])
    plot(RealData(1,17),RealData(1,18),'b*');
    
    % estimated state
    plot(X_NLS_KK(1,1),X_NLS_KK(2,1),'ro');
    length = 0.5;
    plot([X_NLS_KK(1,1),X_NLS_KK(1,1) + length*cos(X_NLS_KK(3,1))],[X_NLS_KK(2,1), X_NLS_KK(2,1) + length*sin(X_NLS_KK(3,1))],'r')

    % real state
    load M_RealData
    plot(RealData(j+1,4),RealData(j+1,5),'bo');
    length = 0.5;
    plot([RealData(j+1,4),RealData(j+1,4) + length*cos(RealData(j+1,6))],[RealData(j+1,5), RealData(j+1,5) + length*sin(RealData(j+1,6))],'b')
    hold off
    pause(0.5)
    
    %% disp final error

    % 
    disp(j+1)
    
    % state error
    disp([RealData(j+1,4) - X_NLS_KK(1,1),RealData(j+1,5) - X_NLS_KK(2,1), (RealData(j+1,6) - X_NLS_KK(3,1))*180/pi])

    % feature error
    disp([RealData(1,11) - X_NLS_KK(4,1), RealData(1,12) - X_NLS_KK(5,1)])
    disp([RealData(1,17) - X_NLS_KK(6,1), RealData(1,18) - X_NLS_KK(7,1)])
    
end

%% 可视化

% estimated feature
hold on
xlim([-10 17])
ylim([-10 20])
plot(X_NLS_KK(4,1),X_NLS_KK(5,1),'r*');
plot(X_NLS_KK(6,1),X_NLS_KK(7,1),'r*');





















