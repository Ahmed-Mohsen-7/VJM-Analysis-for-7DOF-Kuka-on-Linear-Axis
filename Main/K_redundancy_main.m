addpath(genpath('D:\Innopolis_University\Semester2\Advanced Robotics\Project\Task2\Code\utils'))
clear
clc
q_0 = [0,0,0,0,0,0,0]; %currect robot angles

%define lengthes of  robot links in meter 
d_0=0.675;  %base length
d_1=0.350;  %link 1 length
d_2=1.150; %link 2 length
d_3=1.200; %link 3 length (between in j3 and SW)
d_4=-0.041;  %shift in z between j3 and j4
d_5=0.240;  %link 4 length (between in SW and ee)

link_lengths=[d_0;
              d_1;
              d_2;
              d_3;
              d_4;
              d_5];
 

%% Visualizing the home position
[T, ~, ~, ~, ~, ~, ~] = FK(q_0, link_lengths);

%figure

phi_x = atan2(T(3,1),T(3,2));
phi_z = atan2(T(1,3),-T(2,3));
phi_y = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));

cur_pos = [T(1:3,4);phi_x;phi_y;phi_z];

count = 1;

hold on

color_list = {'blue', 'red', 'black'};


%% Redundancy Resolution
[T, T1, T2, T3, T4, T5, T6] = FK([1,deg2rad(75),deg2rad(35),deg2rad(5),0,deg2rad(10),0], link_lengths);
p_global = [T(1:3,4); 0; 0; 0];


pause(3)
tic
while norm(p_global(1:3) - cur_pos(1:3)) > 1e-02
        [q, q_dot] = PseudoInverse(q_0, link_lengths, p_global, 0);
        %[q, q_dot] = Damped_LS(q_0, link_lengths, p_global);
        %[q, q_dot] = Null_Space(q_0, link_lengths, p_global, 0);
        [q, q_dot] = TaskAugmentation(q_0, link_lengths, p_global);
        norm(p_global(1:3) - cur_pos(1:3))
        [T, T1, T2, T3, T4, T5, T6,T7] = FK(q, link_lengths);
        Visualize_Robot(T, T1, T2, T3, T4, T5, T6,T7, color_list{3}, 0,p_global)
        dim=[0.5, 0.2, 0.1, 0.1]; %From here you can location the position of text
        str=sprintf('Position Error: %.2f',norm(p_global(1:3) - cur_pos(1:3))); %if No floting varibale number, use %d
        annotation('textbox',dim,'String',str,'FitBoxToText','on');
        pause(0.001);
        phi_x = atan2(T(3,1),T(3,2));
        phi_z = atan2(T(1,3),-T(2,3));
        phi_y = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));
        cur_pos = [T(1:3,4);phi_x;phi_y;phi_z];
        clf
        view(3)
        q_0 = q;
        count = count + 1
end
toc
Visualize_Robot(T, T1, T2, T3, T4, T5, T6,T7, 'blue', 0)

