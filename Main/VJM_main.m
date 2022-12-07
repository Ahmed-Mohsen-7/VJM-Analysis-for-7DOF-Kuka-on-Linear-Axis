clear;
clc;

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
               
ee_x=.700;
ee_y=.100;
ee_z=.200;

p_ee=[ee_x;        %random position of end effector
      ee_y;
      ee_z];

q_0 = [0,0,0,0,0,0,0]; %intial agnels

d = 0.15;       % Cylinder diameter
K_act = 1000000;   % actuator stiffness
E = 7.0000e+10; % Young's modulus
G = 2.5500e+10; % shear modulus
%for cylinder
S = pi*d^2/4;
Iy = pi*d^4/64;
Iz = pi*d^4/64;         

%calculate stiffness matrix for all links
K_l1=K_link(E, G, link_lengths(1), S, Iy, Iz);
K_l2=K_link(E, G, link_lengths(2), S, Iy, Iz);
K_l3=K_link(E, G, link_lengths(3), S, Iy, Iz);
K_l4=K_link(E, G, link_lengths(4), S, Iy, Iz);
K_l5=K_link(E, G, link_lengths(5), S, Iy, Iz);
K_l6=K_link(E, G, link_lengths(6), S, Iy, Iz);
K_links= [K_l1;K_l2;K_l3;K_l4;K_l5;K_l6];

%range of position in meters
step=0.1;
positions=1:step:1.7;

%each position is square matrix of all possible arrangments, all rows are
%identical
[x,y,z]=meshgrid(positions',positions',positions');
res_VJM=zeros(size(x,1),size(y,1),size(z,1)); %matrix to store deflections for x,y,z,3 alphas 
data_VJM=zeros(size(x,1),size(y,1),size(y,1),6);
%forces applied to ee
F=[50;50;50;0;0;0];
count=1;
figure
grid on;
title('End effector deflections via VJM ');
xlabel('x-axis (m)'); 
ylabel('y-axis (m)');
zlabel('z-axis (m)'); 
for i=1:size(x,1)
    for j=1:size(y,2)
       for k =1:size(z,3)  %850:900
            p_ee=[x(i,j,k);y(i,j,k);y(i,j,k);0;0;0]; %
            Kc_vjm=VJM(link_lengths,p_ee,K_links,K_act,q_0);
            if rank(Kc_vjm)== 6
                dx_vjm=Kc_vjm\F;
                res_VJM(i,j,k)=norm(dx_vjm(1:3));
                count=count + 1
                data_VJM(i,j,k,:) = dx_vjm;
            end
%             scatter3(x(i,j,k), y(i,j,k), z(i,j,k), 40, res_VJM(i,j,k));
%             hold on;
        end
    end
end
scatter3(reshape(x,[],1), reshape(y ,[],1), reshape(z ,[],1), 40, reshape(res_VJM ,[],1))
title('End effector deflections using VJM ');
xlabel('x-axis (m)'); 
ylabel('y-axis (m)');
zlabel('z-axis (m)'); 
colorbar;

titles=[" x component"," y component"," z component"," alpha_x component"," alpha_y component"," alpha_z component"];
for i=1:6
    max_val=max(max(max(data_VJM(:,:,:,i))));
    min_val=min(min(min(data_VJM(:,:,:,i))));
    if i<4
       fprintf("Max deflection for%s: %.4f [mm]; Min deflection: %.4f [mm]\n",titles(i),max_val*10^3,min_val*10^3)
    else
       fprintf("Max deflection for%s: %.4f [deg]; Min deflection: %.4f [deg]\n",titles(i),rad2deg(max_val),rad2deg(min_val))
    end
end
