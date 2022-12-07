addpath(genpath('D:\Innopolis_University\Semester2\Advanced Robotics\Project\Task2\Code\utils'))
% Geometric Calibration
clear all;
close all;
clc;

% set initial data

%define lengthes of  robot links in meter 
d_0=.675;  %base length
d_1=.350;  %link 1 length-203.2
d_2=1.150; %link 2 length
d_3=1.200; %link 3 length (between in j3 and SW)
d_4=-.041;  %shift in z between j3 and j4

% % ideal parameters / measured parameters
idealParams = [0 0 0 0]';%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% ideal base and tools transformation matrices
Tbase = [1 0 0 0; 0 1 0 0; 0 0 1 0.675; 0 0 0 1];
Ttool1 = R_T(Rz(pi/2))*Tx(.183746)*Ty(-.001818)*Tz(-.173871);
Ttool2 =  R_T(Rz(pi/2))*Tx(.249225)*Ty(-.003035)*Tz(.046036);
Ttool3 =  R_T(Rz(pi/2))*Tx(.017719)*Ty(.093845)*Tz(.220954);

qNum = 20; %no of configurations
% noise
sigma = 50*1e-3;

% read joint angles for each config from text file
A =fopen("configurations.txt",'r');
sizeA = [6 20];
fromatspec = "%f";
q= fscanf(A,fromatspec,sizeA);
q=deg2rad(q');

% read real 3 measurments points for each config from text file
A =fopen("points.txt",'r');
sizeA = [3 60];
fromatspec = "%f";
T= fscanf(A,fromatspec,sizeA);
count=1;
M1=zeros(4,4,qNum);
M2=zeros(4,4,qNum);
M3=zeros(4,4,qNum);

for i =1:3:size(T,2)
    R1 = RobotModelFK_Kuka(q(count,:),idealParams,sigma,Tbase,Ttool1);
    R2 = RobotModelFK_Kuka(q(count,:),idealParams,sigma,Tbase,Ttool2);
    R3 = RobotModelFK_Kuka(q(count,:),idealParams,sigma,Tbase,Ttool3);
    M1(:,:,count) = [R1(1:3,1:3) T(:,i)*10^-3;0 0 0 1];
    M2(:,:,count) = [R2(1:3,1:3) T(:,i+1)*10^-3;0 0 0 1];
    M3(:,:,count) = [R3(1:3,1:3) T(:,i+2)*10^-3;0 0 0 1];
    count= count+1;
end
%convert from mm to meter
[p,tb,tt1,tt2,tt3]=FindAllParams_Kuka(q,sigma, idealParams, Tbase,Ttool1,Ttool2,Ttool3, M1, M2, M3);

% % trajectories


traj2q = [-21.22 9.5 0 -54.58 41.61 33.03 -36.67 0;
          37.26 25.81 0 -27.26 -51.7 50.49 38.85 0;
          4.92 16.69 0 -118.71 6.89 -45.61 -4.83 0;
          -21.22 9.5 0 -54.58 41.61 33.03 -36.67 0].*pi/180;
      
traj1q = [90 0 0 90 -90 90 ;
          90 0 0 0 -90 90 ;
          90 45 0 -90 -90 90 ;
          90 0 0 90 -90 90 ].*pi/180;
      
traj3q = [0.7854 1.9721 0.0096 0 -0.0096 -1.8223 ;
        0.7854 -1.9271 0.0096 0 -0.0096 1.8223 ;
        0 1.8281 0 0 0 -1.6314 ;
        0.7854 1.9721 0.0096 0 -0.0096 -1.8223 ];

% show trajectories with and without calibration
tr1=Traj2Points_Kuka(traj1q,idealParams +(rand*2-1)/100 , Tbase);
tr2=Traj2Points_Kuka(traj1q,idealParams, Tbase);
tr3=Traj2Points_Kuka(traj1q,idealParams+p, tb);
%tr4=Traj2Points_Kuka(traj1q,idealParams+p_op, tb_op);
figure()
hold on
grid on
plot3(tr1(1,:), tr1(2,:), tr1(3,:), 'Color','blue')
plot3(tr2(1,:), tr2(2,:), tr2(3,:), 'Color', 'black')
plot3(tr3(1,:), tr3(2,:), tr3(3,:), 'Color', 'red')
legend('target trajectory','without calibration', 'with calibration')
view(3)
% tr1=Traj2Points_Kuka(traj2q,realParams, TbaseR);
% tr2=Traj2Points_Kuka(traj2q,idealParams, Tbase);
% tr3=Traj2Points_Kuka(traj2q,idealParams+p, tb);
% figure()
% hold on
% grid on
% plot3(tr1(1,:), tr1(2,:), tr1(3,:))
% plot3(tr2(1,:), tr2(2,:), tr2(3,:))
% plot3(tr3(1,:), tr3(2,:), tr3(3,:))
% legend('target trajectory','without calibration','with calibration')
% view(3)
% tr1=Traj2Points_Kuka(traj3q,realParams, TbaseR);
% tr2=Traj2Points_Kuka(traj3q,idealParams, Tbase);
% tr3=Traj2Points_Kuka(traj3q,idealParams+p, tb);
% figure()
% hold on
% grid on
% plot3(tr1(1,:), tr1(2,:), tr1(3,:))
% plot3(tr2(1,:), tr2(2,:), tr2(3,:))
% plot3(tr3(1,:), tr3(2,:), tr3(3,:))
% legend('target trajectory','without calibration','with calibration')
% view(3)

% q = RandomConfig_Kuka(qNum, lowLimit, upLimit);
% for i=1:size(q,1)
%         Mr(:,:,i) = RobotModelFK_Kuka(q(i,:),realParams,0,TbaseR,eye(4));
%         Mc(:,:,i) = RobotModelFK_Kuka(q(i,:),idealParams,0,Tbase,eye(4));
%         Mdist(:,:,i) = Mr(:,:,i) - Mc(:,:,i);
%         n(i)=norm(Mdist(1:3,4,i));       
% end
% disp('without calibration')
% acc = mean(n)
% for i=1:size(q,1)
%       %  Mr(:,:,i) = RobotModelFK_Kuka(q(i,:),realParams,0,TbaseR,eye(4));
%         Mc(:,:,i) = RobotModelFK_Kuka(q(i,:),idealParams + p ,0,tb,eye(4));
%         Mdist(:,:,i) = Mr(:,:,i) - Mc(:,:,i);
%         n(i)=norm(Mdist(1:3,4,i));       
% end
% disp('with calibration')
% acc = mean(n)
% for i=1:size(q,1)
%        % Mr(:,:,i) = RobotModelFK_Kuka(q(i,:),realParams,0,TbaseR,eye(4));
%         Mc(:,:,i) = RobotModelFK_Kuka(q(i,:),idealParams + p_op ,0,tb_op,eye(4));
%         Mdist(:,:,i) = Mr(:,:,i) - Mc(:,:,i);
%         n(i)=norm(Mdist(1:3,4,i));       
% end
% disp('with optimal calibration')
% acc = mean(n)
