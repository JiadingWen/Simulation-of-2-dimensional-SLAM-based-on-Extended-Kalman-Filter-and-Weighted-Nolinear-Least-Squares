%% 程序说明

% 生成包含所有信息的RealData (真实state 真实feature 等等)
% 生成包含人可以获得的一般数据 NormalData (控制输入 观测数据 等等)

%% 程序头

% clc
% clear
% close all

%% 设置噪音参数

mean_noise = 0; % 噪音的均值
StandardDeviation_noise = 0.1; % 噪音的标准差
variance_noise = StandardDeviation_noise^2; % 噪音的方差

%% 生成 RealData

% 预分配内存
max_DataSize = 200;                            % 设置真实数据的行数, i.e.节点数量
RealData = zeros(max_DataSize,22);           % 十一列分别为:节点 时间节点 时间周期 
                                             % RobotState 4~6 控制输入   7~8 实际控制噪音 9~10
                                             % feature1 11~12 观测噪音1 13~14 观测数据1 15~16 
                                             % feature2 17~18 观测噪音2 19~20 观测数据2 21~22

% 保存随机生成器的状态
s = rng;

% 初始化节点1
RealData(1,1) = 0;                                                          % 节点 从0开始
RealData(1,2) = 0;                                                          % 时间 从0s开始
RealData(1,3) = 1;                                                          % 时间周期 周期固定为1s

RealData(1,4) = 0;                                                          % RobotState 从(0,0,0)开始
RealData(1,5) = 0;
RealData(1,6) = 0;                                                          % 以上分别为 x y phi

RealData(1,7) = 10;                                                         % 控制输入 linear 固定为10 即 每秒前进10m
RealData(1,8) = pi/2;                                                       % 控制输入 angular 固定为 90 
RealData(1,9) = normrnd(mean_noise,StandardDeviation_noise,1);              % 控制噪音 linear 服从高斯分布 0 mean 0.1^2 variance 
RealData(1,10) = normrnd(mean_noise,StandardDeviation_noise,1);             % 控制噪音 angular 服从高斯分布 0 mean 0.1^2 variance 

RealData(1,11) = 5;                                                         % feature1 特征所在的位置 固定为(5,5)
RealData(1,12) = 5;

RealData(1,13) = normrnd(mean_noise,StandardDeviation_noise,1);              % 观测噪音1 range 服从高斯分布 0 mean 0.1^2 variance 
RealData(1,14) = normrnd(mean_noise,StandardDeviation_noise,1);              % 观测噪音1 theta 服从高斯分布 0 mean 0.1^2 variance
RealData(1,15) = ((RealData(1,11) - RealData(1,4))^2 + (RealData(1,12) - RealData(1,5))^2)^0.5 + RealData(1,13);                  % 观测数据1 range 详见观测模型 
RealData(1,16) = S_Wrap(atan2((RealData(1,12) - RealData(1,5)),(RealData(1,11) - RealData(1,4))) - RealData(1,6) + RealData(1,14));        % 观测数据1 theta 详见观测模型 

RealData(1,17) = 15;                                                         % feature2 特征所在的位置 固定为(15,5)
RealData(1,18) = 5;

RealData(1,19) = normrnd(mean_noise,StandardDeviation_noise,1);              % 观测噪音2 range 服从高斯分布 0 mean 0.1^2 variance 
RealData(1,20) = normrnd(mean_noise,StandardDeviation_noise,1);              % 观测噪音2 theta 服从高斯分布 0 mean 0.1^2 variance
RealData(1,21) = ((RealData(1,17) - RealData(1,4))^2 + (RealData(1,18) - RealData(1,5))^2)^0.5 + RealData(1,19);                  % 观测数据2 range 详见观测模型 
RealData(1,22) = S_Wrap(atan2((RealData(1,18) - RealData(1,5)),(RealData(1,17) - RealData(1,4))) - RealData(1,6) + RealData(1,20));        % 观测数据2 theta 详见观测模型 

%% 节点++
for i = 2:max_DataSize
    
    RealData(i,1) = i-1; 
    RealData(i,2) = RealData(i-1,2) + RealData(i-1,3);
    RealData(i,3) = RealData(i-1,3);
    
    RealData(i,4) = RealData(i-1,4) + (RealData(i-1,7)+ RealData(i-1,9))*RealData(i-1,3)*cos(RealData(i-1,6));    % 控制模型 详见控制模型
    RealData(i,5) = RealData(i-1,5) + (RealData(i-1,7)+ RealData(i-1,9))*RealData(i-1,3)*sin(RealData(i-1,6));
    RealData(i,6) = S_Wrap(RealData(i-1,6) + (RealData(i-1,8)+ RealData(i-1,10))*RealData(i-1,3));
    
    RealData(i,7) = 10;                                                         % 控制输入 linear 固定为10 即 每秒前进10m
    RealData(i,8) = pi/2;                                                       % 控制输入 angular 固定为 90 
    RealData(i,9) = normrnd(mean_noise,StandardDeviation_noise,1);              % 控制噪音 linear 服从高斯分布 0 mean 0.1^2 variance 
    RealData(i,10) = normrnd(mean_noise,StandardDeviation_noise,1);             % 控制噪音 angular 服从高斯分布 0 mean 0.1^2 variance 

    RealData(i,11) = 5;                                                         % feature1 特征所在的位置 固定为(5,5)
    RealData(i,12) = 5;

    RealData(i,13) = normrnd(mean_noise,StandardDeviation_noise,1);              % 观测噪音1 range 服从高斯分布 0 mean 0.1^2 variance 
    RealData(i,14) = normrnd(mean_noise,StandardDeviation_noise,1);              % 观测噪音1 theta 服从高斯分布 0 mean 0.1^2 variance
    RealData(i,15) = ((RealData(i,11) - RealData(i,4))^2 + (RealData(i,12) - RealData(i,5))^2)^0.5 + RealData(i,13);                  % 观测数据1 range 详见观测模型 
    RealData(i,16) = S_Wrap(atan2((RealData(i,12) - RealData(i,5)),(RealData(i,11) - RealData(i,4))) - RealData(i,6) + RealData(i,14));        % 观测数据1 theta 详见观测模型 

    RealData(i,17) = 15;                                                         % feature2 特征所在的位置 固定为(15,5)
    RealData(i,18) = 5;

    RealData(i,19) = normrnd(mean_noise,StandardDeviation_noise,1);              % 观测噪音2 range 服从高斯分布 0 mean 0.1^2 variance 
    RealData(i,20) = normrnd(mean_noise,StandardDeviation_noise,1);              % 观测噪音2 theta 服从高斯分布 0 mean 0.1^2 variance
    RealData(i,21) = ((RealData(i,17) - RealData(i,4))^2 + (RealData(i,18) - RealData(i,5))^2)^0.5 + RealData(i,19);                  % 观测数据2 range 详见观测模型 
    RealData(i,22) = S_Wrap(atan2((RealData(i,18) - RealData(i,5)),(RealData(i,17) - RealData(i,4))) - RealData(i,6) + RealData(i,20));        % 观测数据2 theta 详见观测模型 
    
end

% 储存 RealDta
save M_RealData RealData

%% 生成 NormalData

% 预分配内存
NormalData = zeros(max_DataSize,9); % 五列分别为:节点 时间节点 时间周期 控制输入 4~5 观测数据1 6~7 观测数据2 8~9

% 从 RealData 中提取 NormalData
NormalData(:,1) = RealData(:,1); % 节点
NormalData(:,2) = RealData(:,2); % 时间节点
NormalData(:,3) = RealData(:,3); % 时间周期

NormalData(:,[4 5]) = RealData(:,[7 8]); % 控制输入

NormalData(:,[6 7]) = RealData(:,[15 16]); % 观测数据1
NormalData(:,[8 9]) = RealData(:,[21 22]); % 观测数据2

% 储存 NormalData
save M_NormalData NormalData






