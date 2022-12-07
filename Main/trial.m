addpath(genpath('D:\Innopolis_University\Semester2\Advanced Robotics\Project\Task2\Code\utils'))

%define lengthes of  robot links in meter 
d_1=0.350;  %link 1 length
d_2=-1.150; %link 2 length
d_3=1.200; %link 3 length (between in j3 and SW)
d_4=-0.041;  %shift in z between j3 and j4

%q = [deg2rad(47.71	)	deg2rad(-21.96	)	deg2rad(43.02	)	deg2rad(227.67	)	deg2rad(35.93	)	deg2rad(-231.38)];
q = [deg2rad(0)	deg2rad(-90)	deg2rad(90)	deg2rad(0)	deg2rad(0)	deg2rad(0)];			
Tbase = [1 0 0 0; 0 1 0 0; 0 0 1 0.675; 0 0 0 1];

params= [0 d_1 0 0 d_2 0 0 0 d_3 d_4 0 0 0 0 0 0 0 0];

Ttool1 = R_T(Rz(pi/2))*Tx(.183746)*Ty(-.001818)*Tz(-.173871);
Ttool2 =  R_T(Rz(pi/2))*Tx(.249225)*Ty(-.003035)*Tz(.046036);
Ttool3 =  R_T(Rz(pi/2))*Tx(.017719)*Ty(.093845)*Tz(.220954);

T1 = RobotModelFK_Kuka(q,params,0,Tbase,Ttool1)
% qNum=20;
% A =fopen("points.txt",'r');
% sizeA = [3 60];
% fromatspec = "%f";
% T= fscanf(A,fromatspec,sizeA);
% count=1;
% M1=zeros(4,4,qNum);
% M2=zeros(4,4,qNum);
% M3=zeros(4,4,qNum);
% for i =1:3:size(T,2)
%     M1(:,:,count) = [eye(3) T(:,i);0 0 0 1];
%     M2(:,:,count) = [eye(3) T(:,i+1);0 0 0 1];
%     M3(:,:,count) = [eye(3) T(:,i+2);0 0 0 1];
%     count= count+1;
% end

