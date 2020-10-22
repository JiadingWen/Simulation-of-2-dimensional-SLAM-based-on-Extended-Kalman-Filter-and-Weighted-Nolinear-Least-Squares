%% 2D SLAM EKF

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
x_r_0 = 5;
y_r_0 = 0;
phi_r_0 = 0;

%% 初始化

% 获取观测数据
r_f1_0 = NormalData(1,6);
theta_f1_0 = NormalData(1,7);
r_f2_0 = NormalData(1,8);
theta_f2_0 = NormalData(1,9);

% 
X_EKF_00 = [x_r_0
        y_r_0
        phi_r_0
        x_r_0 + cos(theta_f1_0 + phi_r_0)*r_f1_0
        y_r_0 + sin(theta_f1_0 + phi_r_0)*r_f1_0
        x_r_0 + cos(theta_f2_0 + phi_r_0)*r_f2_0
        y_r_0 + sin(theta_f2_0 + phi_r_0)*r_f2_0]; 

% 
P = diag([0.0000001 0.0000001 0.0000001 variance_noise variance_noise variance_noise variance_noise]);

% Jacobian
J_F0_X = [1 0 0 0 0 0 0
         0 1 0 0 0 0 0
         0 0 1 0 0 0 0
         1 0 -sin(theta_f1_0 + phi_r_0)*r_f1_0    cos(theta_f1_0 + phi_r_0)    -sin(theta_f1_0 + phi_r_0)*r_f1_0   0 0
         0 1  cos(theta_f1_0 + phi_r_0)*r_f1_0    sin(theta_f1_0 + phi_r_0)     cos(theta_f1_0 + phi_r_0)*r_f1_0   0 0
         1 0 -sin(theta_f2_0 + phi_r_0)*r_f2_0  0 0     cos(theta_f2_0 + phi_r_0)    -sin(theta_f2_0 + phi_r_0)*r_f2_0
         0 1  cos(theta_f2_0 + phi_r_0)*r_f2_0  0 0     sin(theta_f2_0 + phi_r_0)     cos(theta_f2_0 + phi_r_0)*r_f2_0];

% 
P_EKF_00 = J_F0_X*P*J_F0_X';

% 可视化
figure(1)
xlim([-5 17])
ylim([-5 20])
hold on
plot(X_EKF_00(1,1),X_EKF_00(2,1),'ro');
length = 0.5;
plot([X_EKF_00(1,1),X_EKF_00(1,1) + length*cos(X_EKF_00(3,1))],[X_EKF_00(2,1), X_EKF_00(2,1) + length*sin(X_EKF_00(3,1))],'r')
% real feature
plot(RealData(1,11),RealData(1,12),'b*');
plot(RealData(1,17),RealData(1,18),'b*');
hold off

%% EKF化

% 循环初始输入
X_EKF_KK = X_EKF_00;
P_EKF_KK = P_EKF_00;

for i = 1:(size(NormalData,1) - 1)
    %% Prediction using process model
    
    % X_EKF_KK
    x_r_k = X_EKF_KK(1,1);
    y_r_k = X_EKF_KK(2,1);
    phi_r_k = X_EKF_KK(3,1);
    x_f1 = X_EKF_KK(4,1);
    y_f1 = X_EKF_KK(5,1);
    x_f2 = X_EKF_KK(6,1);
    y_f2 = X_EKF_KK(7,1);
    
    % P_EKF_KK
    
    % Uk
    vk = NormalData(i,4);
    wk = NormalData(i,5);
    Q = diag([variance_noise variance_noise]);
    
    % T
    T = NormalData(i,3);
    
    % J_F_X_EKF_KK
    J_F_X_EKF_KK = [1 0 vk*T*-sin(phi_r_k) 0 0 0 0
        0 1 vk*T*cos(phi_r_k) 0 0 0 0
        0 0 1 0 0 0 0
        0 0 0 1 0 0 0
        0 0 0 0 1 0 0
        0 0 0 0 0 1 0
        0 0 0 0 0 0 1];
    
    % J_F_Uk
    J_F_Uk = [T*cos(phi_r_k) 0
        T*sin(phi_r_k) 0
        0 T
        0 0
        0 0
        0 0
        0 0];
    
    % X_EKF_K1K
    x_r_k1 = x_r_k + vk*T*cos(phi_r_k);
    y_r_k1 = y_r_k + vk*T*sin(phi_r_k);
    phi_r_k1 = S_Wrap(phi_r_k + wk*T);
    %x_f1 = x_f1;
    %y_f1 = y_f1;
    %x_f2 = x_f2;
    %y_f2 = y_f2;
    X_EKF_K1K = [x_r_k1;y_r_k1;phi_r_k1;x_f1;y_f1;x_f2;y_f2;];
    
    % P_EKF_K1K
    P_EKF_K1K = J_F_X_EKF_KK*P_EKF_KK*J_F_X_EKF_KK' + J_F_Uk*Q*J_F_Uk';
    
    %% Update using observations
    
    % J_H_X_EKF_K1K
    J_H_X_EKF_K1K = [((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(-0.5)*(x_f1 - x_r_k1)*(-1)  ...
        ((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(-0.5)*(y_f1 - y_r_k1)*(-1)  ...
        0 ((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(-0.5)*(x_f1 - x_r_k1)  ...
        ((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(-0.5)*(y_f1 - y_r_k1) 0 0
        
        (y_f1 - y_r_k1)/((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)  ...
        -(x_f1 - x_r_k1)/((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2) -1  ...
        -(y_f1 - y_r_k1)/((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)  ...
        (x_f1 - x_r_k1)/((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2) 0 0
        
        ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(-0.5)*(x_f2 - x_r_k1)*(-1)  ...
        ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(-0.5)*(y_f2 - y_r_k1)*(-1) 0 0 0  ...
        ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(-0.5)*(x_f2 - x_r_k1)  ...
        ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(-0.5)*(y_f2 - y_r_k1)
        
        (y_f2 - y_r_k1)/((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)  ...
        -(x_f2 - x_r_k1)/((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)  ...
        -1 0 0  ...
        -(y_f2 - y_r_k1)/((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)  ...
        (x_f2 - x_r_k1)/((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)
        ];
    
    % R
    R = diag([variance_noise variance_noise variance_noise variance_noise]);
    
    % S
    S = J_H_X_EKF_K1K*P_EKF_K1K*J_H_X_EKF_K1K' + R;
    
    % K
    %K = P_EKF_K1K*J_H_X_EKF_K1K'*inv(S);
    K = P_EKF_K1K*J_H_X_EKF_K1K'*pinv(S);
    
    % X_EKF_K1K1
    
    % H(X_EKF_K1K)
    r_f1_k1 = ((x_f1 - x_r_k1)^2 + (y_f1 - y_r_k1)^2)^(0.5); 
    theta_f1_k1 = S_Wrap(atan2((y_f1 - y_r_k1), (x_f1 - x_r_k1)) - phi_r_k1); 
    r_f2_k1 = ((x_f2 - x_r_k1)^2 + (y_f2 - y_r_k1)^2)^(0.5);
    theta_f2_k1 = S_Wrap(atan2((y_f2 - y_r_k1), (x_f2 - x_r_k1)) - phi_r_k1); 
    H_X_EKF_K1K = [r_f1_k1;theta_f1_k1;r_f2_k1;theta_f2_k1];
    % Zk1
    Zk1 = [NormalData(i+1,6);NormalData(i+1,7);NormalData(i+1,8);NormalData(i+1,9);];
    % Zk1 - H_X_EKF_K1K
    Zk1_H_X_EKF_K1K = [NormalData(i+1,6) - r_f1_k1; S_Wrap(NormalData(i+1,7) - theta_f1_k1);  ...
        NormalData(i+1,8) - r_f2_k1; S_Wrap(NormalData(i+1,9) - theta_f2_k1)];
    % X_EKF_K1K1
    X_EKF_K1K1 = X_EKF_K1K + K*Zk1_H_X_EKF_K1K;
    x_r_k1 = X_EKF_K1K1(1,1);
    y_r_k1 = X_EKF_K1K1(2,1);
    X_EKF_K1K1(3,1) = S_Wrap(X_EKF_K1K1(3,1));
    phi_r_k1 = X_EKF_K1K1(3,1);
    x_f1 = X_EKF_K1K1(4,1);
    y_f1 = X_EKF_K1K1(5,1);
    x_f2 = X_EKF_K1K1(6,1);
    y_f2 = X_EKF_K1K1(7,1);
    
    % P_EKF_K1K1
    P_EKF_K1K1 = P_EKF_K1K - K * S * K';
    
    %% 为下次循环做好准备
    X_EKF_KK = X_EKF_K1K1;
    P_EKF_KK = P_EKF_K1K1;
    
    
    %% 可视化

    % real feature
    % hold on
    plot(RealData(1,11),RealData(1,12),'b*');
    hold on
    xlim([-5 17])
    ylim([-5 20])
    plot(RealData(1,17),RealData(1,18),'b*');
    
    % estimated feature
    plot(X_EKF_KK(4,1),X_EKF_KK(5,1),'r*');
    plot(X_EKF_KK(6,1),X_EKF_KK(7,1),'r*'); 
    
    % estimated state
    plot(X_EKF_KK(1,1),X_EKF_KK(2,1),'ro');
    length = 0.5;
    plot([X_EKF_KK(1,1),X_EKF_KK(1,1) + length*cos(X_EKF_KK(3,1))],[X_EKF_KK(2,1), X_EKF_KK(2,1) + length*sin(X_EKF_KK(3,1))],'r')

    % real state
    figure(1)
    plot(RealData(i+1,4),RealData(i+1,5),'bo');
    length = 0.5;
    plot([RealData(i+1,4),RealData(i+1,4) + length*cos(RealData(i+1,6))],[RealData(i+1,5), RealData(i+1,5) + length*sin(RealData(i+1,6))],'b')
    hold off
    pause(0.2)
    
    %% disp final error

    % 
    disp(i+1)
    
    % state error
    disp([RealData(i+1,4) - X_EKF_KK(1,1),RealData(i+1,5) - X_EKF_KK(2,1), (RealData(i+1,6) - X_EKF_KK(3,1))*180/pi])

    % feature error
    disp([RealData(1,11) - X_EKF_KK(4,1), RealData(1,12) - X_EKF_KK(5,1)])
    disp([RealData(1,17) - X_EKF_KK(6,1), RealData(1,18) - X_EKF_KK(7,1)])

end






