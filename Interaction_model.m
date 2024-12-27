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
iiwa = KST(ip, arg1, arg2, Tef_flange); 

flag = iiwa.net_establishConnection();
if flag == 0
    return;
end
disp('机器人已连接！');
pause(1);



try
    targetForce = [0; 0; 40];
    MassofTool = 0.5;                                                     
    Gravity = [0; 0; 9.81];
    CenofTool=[0; 0; 0];                                                  

    runTime = 20*60;                                                         
    fs = 50;                                                              
    timeInt  = 1 / fs;                                                    
    timeVec  = 0:timeInt:runTime;
    totalLoop = length(timeVec);
    t = 0;                                                                

    % Control Loop
    ALL_EEFForce = [];                                                    
    ALL_EEFCartPose =[];                                                  
    ALL_EEFadmitCartPose = [];                                            
    ALL_TimeInt = [];                                                     

    % Initialize force control variables
    PosErrorLast = [0; 0; 0];                                             
    VelErrorLast = [0; 0; 0];                                             
    AccErrorLast = [0; 0; 0];                                             

    % Initialize addmittance parameter
    K_cartesian = diag([500, 500, 500]);                                     
    B_cartesian = diag([50, 50, 50]);                                  
    M_cartesian = diag([0.9, 0.9, 0.9]);                                  

    fprintf('Cartesian position acting at end effector:\n')
    init_eef_cart = cell2mat(iiwa.getEEFPos());                           
    disp(init_eef_cart);

    RotaM = iiwa.getEEFOrientationR();                                    
    %G = RotaM * MassofTool * Gravity;                                  

    init_pos = [init_eef_cart(1), init_eef_cart(2), init_eef_cart(3)];
    init_ori = [init_eef_cart(4), init_eef_cart(5), init_eef_cart(6)]; 

    fprintf('Cartesian position acting at end effector:\n')
    ALL_EEFCartPose = [ALL_EEFCartPose init_pos'];                       
    disp(ALL_EEFCartPose);
    
    EEFTarget = zeros(3, totalLoop);
    EEFTarget(1,1) = init_pos(1);                                                 
    EEFTarget(2,1) = init_pos(2);                                                 
    EEFTarget(3,1) = init_pos(3);                                                 

    iiwa.realTime_startDirectServoCartesian();
   
        controlSignal = init_eef_cart;       

        inputcontrolSignal = num2cell(controlSignal);
        init_eef_force = cell2mat(iiwa.sendEEfPositionGetEEF_Force_rel_EEF(inputcontrolSignal));
        Delta_F = targetForce - init_eef_force';

        tic;                                                                

        while(t <= runTime)

            t0 = toc;


            AccError(3) = M_cartesian(3,3) * (Delta_F(3) - B_cartesian(3,3) * VelErrorLast(3) - K_cartesian(3,3) * PosErrorLast(3));  

            VelError(3) = VelErrorLast(3) + AccError(3) * (t0 - t);          

            PosError(3) = PosErrorLast(3) + VelError(3) * (t0 - t);          

            PosError_Base = RotaM * [0; 0; PosError(3)];

            PosTargetNew(1) = EEFTarget(1) - PosError_Base(1);                                     
            PosTargetNew(2) = EEFTarget(2) - PosError_Base(2);                                     
            PosTargetNew(3) = EEFTarget(3) - PosError_Base(3);                       
            PosTargetNew = [PosTargetNew(1); PosTargetNew(2); PosTargetNew(3)];
            ALL_EEFadmitCartPose = [ALL_EEFadmitCartPose PosTargetNew];        

            controlSignal = [PosTargetNew(1), PosTargetNew(2), PosTargetNew(3), init_eef_cart(4), init_eef_cart(5), init_eef_cart(6)];       

            inputcontrolSignal = num2cell(controlSignal);                       

            readcurrentpose = cell2mat(iiwa.sendEEfPositionGetActualEEFpos(inputcontrolSignal));

            EEFCartNow = [readcurrentpose(1); readcurrentpose(2); readcurrentpose(3)];
            ALL_EEFCartPose = [ALL_EEFCartPose EEFCartNow];                    

            eef_force = cell2mat(iiwa.sendEEfPositionGetEEF_Force_rel_EEF(inputcontrolSignal));
            ALL_EEFForce = [ALL_EEFForce eef_force'];                           

            Delta_F = targetForce - eef_force';                                                    

            EEFTarget = PosTargetNew;                                           
            PosErrorLast = PosError;                                            
            VelErrorLast = VelError;                                            
            AccErrorLast = AccError;                                            

            t = toc;

            fprintf('时间t:\n')
            disp(t);
            
            ALL_TimeInt = [ALL_TimeInt t];


        end

    iiwa.realTime_stopDirectServoCartesian();                                    

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
