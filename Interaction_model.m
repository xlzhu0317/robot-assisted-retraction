close all;
clear;
clc;
warning('off');
iiwapath = 'D:\KUKA\KST-Kuka-Sunrise-Toolbox-master\KST-Kuka-Sunrise-Toolbox-master\Matlab_client';
addpath(genpath(iiwapath));

ip = '172.31.1.147';
arg1 = KST.LBR14R820;
arg2 = KST.Medien_Flansch_elektrisch;
Tef_flange = eye(4);
%Tef_flange(3,4) = 30/1000;                                         
iiwa = KST(ip, arg1, arg2, Tef_flange); 

flag = iiwa.net_establishConnection();
if flag == 0
    return;
end
disp('机器人已连接！');
pause(1);



try
    targetForce = [0; 0; 40];
    MassofTool = 0.5;                                                     % 末端工具重量
    Gravity = [0; 0; 9.81];
    CenofTool=[0; 0; 0];                                                  % 重心相对法兰的位置

    runTime = 20*60;                                                         % 运行时间，单位s，即12分钟
    fs = 50;                                                              % 单位Hz 这是指采样信号的频率
    timeInt  = 1 / fs;                                                    % 时间间隔，即1ms发送一次信号
    timeVec  = 0:timeInt:runTime;
    totalLoop = length(timeVec);
    t = 0;                                                                % 时间计数判断

    % Control Loop
    ALL_EEFForce = [];                                                    % 记录测量的末端力
    ALL_EEFCartPose =[];                                                  % 记录测量的末端位置
    ALL_EEFadmitCartPose = [];                                            % 记录导纳控制计算的末端位置
    ALL_TimeInt = [];                                                     % 记录运行时间

    % Initialize force control variables
    PosErrorLast = [0; 0; 0];                                             % 初始的位置偏差
    VelErrorLast = [0; 0; 0];                                             % 初始的速度偏差
    AccErrorLast = [0; 0; 0];                                             % 初始的加速度偏差

    % Initialize addmittance parameter
    K_cartesian = diag([500, 500, 500]);                                     % 弹簧系数
    B_cartesian = diag([50, 50, 50]);                                  % 阻尼系数
    M_cartesian = diag([0.9, 0.9, 0.9]);                                  % 质量系数的逆

    fprintf('Cartesian position acting at end effector:\n')
    init_eef_cart = cell2mat(iiwa.getEEFPos());                           % 读取末端的位姿1*6
    disp(init_eef_cart);

    RotaM = iiwa.getEEFOrientationR();                                    % 读取末端的旋转矩阵
    %G = RotaM * MassofTool * Gravity;                                  % 工具重力分解到法兰

    init_pos = [init_eef_cart(1), init_eef_cart(2), init_eef_cart(3)];
    init_ori = [init_eef_cart(4), init_eef_cart(5), init_eef_cart(6)]; 

    fprintf('Cartesian position acting at end effector:\n')
    ALL_EEFCartPose = [ALL_EEFCartPose init_pos'];                       % 记录初始末端位置
    disp(ALL_EEFCartPose);
    
    EEFTarget = zeros(3, totalLoop);
    EEFTarget(1,1) = init_pos(1);                                                 % X轴目标位置
    EEFTarget(2,1) = init_pos(2);                                                 % Y轴目标位置
    EEFTarget(3,1) = init_pos(3);                                                 % Z轴目标位置

    iiwa.realTime_startDirectServoCartesian();
   
        controlSignal = init_eef_cart;       

        inputcontrolSignal = num2cell(controlSignal);
        init_eef_force = cell2mat(iiwa.sendEEfPositionGetEEF_Force_rel_EEF(inputcontrolSignal));
        Delta_F = targetForce - init_eef_force';

        tic;                                                                % 计时器

        while(t <= runTime)

            t0 = toc;


            AccError(3) = M_cartesian(3,3) * (Delta_F(3) - B_cartesian(3,3) * VelErrorLast(3) - K_cartesian(3,3) * PosErrorLast(3));  % 导纳控制求出的加速度

            VelError(3) = VelErrorLast(3) + AccError(3) * (t0 - t);          % 求出的速度

            PosError(3) = PosErrorLast(3) + VelError(3) * (t0 - t);          % 求出的位置

            PosError_Base = RotaM * [0; 0; PosError(3)];

            PosTargetNew(1) = EEFTarget(1) - PosError_Base(1);                                     % X轴方向目标位置不变
            PosTargetNew(2) = EEFTarget(2) - PosError_Base(2);                                     % X轴方向目标位置不变
            PosTargetNew(3) = EEFTarget(3) - PosError_Base(3);                       % Z轴方向导纳控制器更新后的目标位置
            PosTargetNew = [PosTargetNew(1); PosTargetNew(2); PosTargetNew(3)];
            ALL_EEFadmitCartPose = [ALL_EEFadmitCartPose PosTargetNew];        % 记录末端轨迹

            controlSignal = [PosTargetNew(1), PosTargetNew(2), PosTargetNew(3), init_eef_cart(4), init_eef_cart(5), init_eef_cart(6)];       % 锁定末端旋转, 1*6的矩阵

            inputcontrolSignal = num2cell(controlSignal);                       % 下面的API输入是6个元胞

            readcurrentpose = cell2mat(iiwa.sendEEfPositionGetActualEEFpos(inputcontrolSignal));

            EEFCartNow = [readcurrentpose(1); readcurrentpose(2); readcurrentpose(3)];
            ALL_EEFCartPose = [ALL_EEFCartPose EEFCartNow];                    % 记录实际的末端轨迹

            eef_force = cell2mat(iiwa.sendEEfPositionGetEEF_Force_rel_EEF(inputcontrolSignal));
            ALL_EEFForce = [ALL_EEFForce eef_force'];                           % 记录实时末端力变化

            Delta_F = targetForce - eef_force';                                 % 计算实时末端力
            % ALL_EEFForceBias = [ALL_EEFForceBias Delta_F];                      % 记录实时末端力偏差变化

            EEFTarget = PosTargetNew;                                           % 将PosTargetNew赋值给下一循环的EEFTarget
            PosErrorLast = PosError;                                            % 更新位置误差
            VelErrorLast = VelError;                                            % 更新速度误差
            AccErrorLast = AccError;                                            % 更新加速度误差

            t = toc;

            fprintf('时间t:\n')
            disp(t);
            
            ALL_TimeInt = [ALL_TimeInt t];


        end

    iiwa.realTime_stopDirectServoCartesian();                                    % 停止伺服控制

catch exception
    disp(['Error: ', getReport(exception)]);
end

iiwa.net_turnOffServer( );
warning('on');
rmpath(genpath(iiwapath));


outputFileName = sprintf('%02d%02d%02d%02d.xlsx', month(datetime), day(datetime), hour(datetime), minute(datetime));

ALL_EEFForce = ALL_EEFForce';
ALL_EEFCartPose = ALL_EEFCartPose';
ALL_EEFadmitCartPose = ALL_EEFadmitCartPose';
ALL_TimeInt = ALL_TimeInt';

writematrix(ALL_EEFForce, outputFileName, 'Sheet', 'EEFForce');
writematrix(ALL_EEFCartPose, outputFileName, 'Sheet', 'EEFCartPose');
writematrix(ALL_EEFadmitCartPose, outputFileName, 'Sheet', 'EEFadmitCartPose');
writematrix(ALL_TimeInt, outputFileName, 'Sheet', 'TimeInt');

disp(['数据已保存到文件：', outputFileName]);