% 使用ATI传感器，速度环,加入迭代控制和零空间优化
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
%Tef_flange(3,4) = 30/1000;                                          % 修改第3行第4列的值
iiwa = KST(ip, arg1, arg2, Tef_flange); 

flag = iiwa.net_establishConnection();
if flag == 0
    return;
end
disp('机器人已连接！');
pause(1);



try
    targetForce = [0; 0; 20];

    runTime = 2*60;                                                      % 运行时间，单位s，即90分钟
    Delta_T = 0.001;                                                      % 时间计数判断
    K_H = [1; 1; 1];
    Q = diag([0.5, 0.5, 0.5]);                                                          % 损失函数权重
    P = diag([0.15, 0.15, 0.15]); 
    K_w = diag([0.05, 0.5, 0.05, 1.2, 0.1, 1, 0.1]);
    t=0;
    

    % Control Loop
    ALL_EEFForce = [];                                                    % 记录测量的末端力
    ALL_EEFadmitCartVel = [];                                             % 记录导纳控制计算的末端速度
    ALL_TimeInt = [];                                                     % 记录运行时间
    ALL_Joint = [];                                                       % 记录角度
    ALL_B = [];
    ALL_JointVel = [];
    ALL_LossF = [];
    ALL_ObjW = [];

    % Initialize force control variables
    VelErrorLast = [0; 0; 0];                                             % 初始的速度偏差

    % Initialize addmittance parameter
    B_cartesianLast = diag([200, 200, 300]);                                     % 阻尼系数
    M_cartesian = diag([1.25, 1.25, 1.25]);                                  % 质量系数的逆

    EEFTarget = [0; 0; 0];

    RotaM = iiwa.getEEFOrientationR();                                    % 读取末端的旋转矩阵

    init_eef_force = ATISensor();
    Delta_F_Last = init_eef_force - targetForce;

    JointInit = cell2mat(iiwa.getJointsPos());

    Joint = JointInit;
    JointLast = JointInit;

    iiwa.realTime_startVelControlJoints();
   

        tic;                                                                % 计时器

        while(t <= runTime)

            t0 = toc;

            [~, J] = iiwa.gen_DirectKinematics(Joint);
            Jvel = J(1:3,:);

            AccError(3) = M_cartesian(3,3) * (Delta_F_Last(3) - B_cartesianLast(3,3) * VelErrorLast(3));  % 导纳控制求出的加速度
            VelError(3) = VelErrorLast(3) + AccError(3) * Delta_T;          % 求出的速度
            
            VelError_Base = RotaM * [0; 0; VelError(3)];
         

            % EEFVel = EEFTarget - VelError_Base;                                     % X轴方向目标位置不变
            EEFVel = RotaM * [0; 0; VelError(3)];
            ALL_EEFadmitCartVel = [ALL_EEFadmitCartVel EEFVel];

            N = eye(7) - J' * inv(J*J') * J;  

            Delta_q = Joint - JointInit;
            Delta_q_Last = JointLast - JointInit;

            W_q = - Delta_q * K_w * Delta_q' + Delta_q_Last * K_w * Delta_q_Last';

            JonitVel = pinv(Jvel) * EEFVel + N * W_q * pinv(Joint - JointLast);

            CurrentJoint = iiwa.sendJointsVelocitiesGetActualJpos(num2cell(JonitVel));

            eef_force = ATISensor();
            ALL_EEFForce = [ALL_EEFForce eef_force];                           % 记录实时末端力变化

            Delta_F = eef_force - targetForce;                                 % 计算实时末端力

            t = toc;

            %Delta_T = t - t0;
            
            % Loss Function

            %D_Delta_F = (Delta_F - Delta_F_Last) / Delta_T;

            %LossF = Delta_F' * Q * Delta_F + Delta_F_Last' * P * Delta_F_Last;
            LossF = Delta_F' * Q * Delta_F - Delta_F_Last' * Q * Delta_F_Last;

            B_cartesian = B_cartesianLast - K_H' * LossF *inv(B_cartesianLast); 

            B_cartesianLast = B_cartesian; 
            
            JointLast = Joint; 
            Joint = cell2mat(CurrentJoint);                                    % 将当前关节角度赋值给下一循环
            ALL_Joint = [ALL_Joint Joint'];
            ALL_JointVel = [ALL_JointVel JonitVel];
            ALL_LossF = [ALL_LossF LossF];
            ALL_ObjW = [ALL_ObjW W_q];

            B_flattened = B_cartesianLast(:);
            ALL_B = [ALL_B B_flattened];


            VelErrorLast = VelError;                                           % 更新速度误差
            Delta_F_Last = Delta_F;

            

            fprintf('时间t:\n')
            disp(t);
            
            ALL_TimeInt = [ALL_TimeInt t];


        end

    iiwa.realTime_stopVelControlJoints();                                    % 停止伺服控制

catch exception
    disp(['Error: ', getReport(exception)]);
end

iiwa.net_turnOffServer( );
warning('on');
rmpath(genpath(iiwapath));



figure(1)
subplot(3,1,1)
plot(ALL_TimeInt(:), ALL_EEFForce(1,:),'b','Linewidth',2);
title('末端x方向受力','Fontsize',10);
grid on

subplot(3,1,2)
plot(ALL_TimeInt(:), ALL_EEFForce(2,:),'g','Linewidth',2);
title('末端y方向受力','Fontsize',10);
grid on

subplot(3,1,3)
plot(ALL_TimeInt(:), ALL_EEFForce(3,:),'r','Linewidth',2);
title('末端z方向受力','Fontsize',10);
grid on


outputFileName = sprintf('%02d%02d%02d%02d.xlsx', month(datetime), day(datetime), hour(datetime), minute(datetime));

ALL_EEFForce = ALL_EEFForce';
ALL_EEFadmitCartPose = ALL_EEFadmitCartVel';
ALL_TimeInt = ALL_TimeInt';
ALL_Joint = ALL_Joint';
ALL_JointVel = ALL_JointVel';
ALL_LossF = ALL_LossF';
ALL_ObjW = ALL_ObjW';
ALL_B = ALL_B';

writematrix(ALL_EEFForce, outputFileName, 'Sheet', 'EEFForce');
writematrix(ALL_EEFadmitCartPose, outputFileName, 'Sheet', 'EEFadmitCartVel');
writematrix(ALL_TimeInt, outputFileName, 'Sheet', 'TimeInt');
writematrix(ALL_Joint, outputFileName, 'Sheet', 'Joint');
writematrix(ALL_JointVel, outputFileName, 'Sheet', 'JointVel');
writematrix(ALL_LossF, outputFileName, 'Sheet', 'LossF');
writematrix(ALL_ObjW, outputFileName, 'Sheet', 'W_q');
writematrix(ALL_B, outputFileName, 'Sheet', 'B');

disp(['数据已保存到文件：', outputFileName]);




