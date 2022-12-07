
function [Kc,K_t]=VJM(link_lengths,goal,K_links,K_act,q_init)
%{
this function take link_lengths (constants)
goal is position of end effector
k_links contains k in local coordinates for all links
k_act is actuation matrix
q_0 is intial robot angles
%}
d_0=link_lengths(1);  %base length
d_1=link_lengths(2);  %link 1 length
d_2=link_lengths(3);  %link 2 length
d_3=link_lengths(4);  %link 3 length (between in j3 and SW)
d_4=link_lengths(5);  %shift in z between j3 and j4
d_5=link_lengths(6);  %link 4 length (between SW and ee)

Kc=zeros(6);
q=IK(goal,q_init,link_lengths);

q_0=q(1); %first joint angle (P)
q_1=q(2); %second joint angle (R)
q_2=q(3); %third joint angle (R)
q_3=q(4); %fourth joint angle (R)
q_4=q(5); %fifth joint angle (R)
q_5=q(6); %sixth joint angle (R)
q_6=q(7); %seventh joint angle (R)


%Forward Kinematics

Tlink1 =   Tz(d_0)  ;
Tlink2 =   Tx(d_1) ;
Tlink3 =   Tx(d_2) ;
Tlink4  =  Tz(d_4) ;
Tlink5 =   Tx(d_3) ;
Tlink6 =   Tx(d_5);


T0=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;

T0(1:3,4)=[0;0;0];  %position equals zero 
T0=T0';


%>>>> J_theta for 7 joint angles
dH_q1=Ty(q_0) *dTy* Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6 * T0;
J_q1=Jnum(dH_q1);

dH_q2=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*dRz(0)*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 *Tlink5* R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6 *T0;
J_q2=Jnum(dH_q2);

dH_q3=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*dRy(0)*Tlink3 * R_T(Ry(q_3)) * Tlink4 *Tlink5* R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6*T0;
J_q3=Jnum(dH_q3);

dH_q4=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) *dRy(0)* Tlink4 *Tlink5* R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6*T0;
J_q4=Jnum(dH_q4);

dH_q5=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*dRy(0)*Tlink3 * R_T(Ry(q_3)) * Tlink4 *Tlink5* R_T(Rx(q_4)) *dRx(0)*R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6*T0;
J_q5=Jnum(dH_q5);

dH_q6=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*dRy(0)*Tlink3 * R_T(Ry(q_3)) * Tlink4 *Tlink5* R_T(Rx(q_4)) *R_T(Ry(q_5)) *dRy(0)* R_T(Rx(q_6)) * Tlink6 * T0;
J_q6=Jnum(dH_q6);

dH_q7=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*dRy(0)*Tlink3 * R_T(Ry(q_3)) * Tlink4 *Tlink5* R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) *dRx(0)* Tlink6 * T0;
J_q7=Jnum(dH_q7);


%T3D for virtual joint of first link >>>> J_theta
dH_1=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_1=Jnum(dH_1);
dH_2=Ty(q_0) * Tlink1 *dTy* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_2=Jnum(dH_2);
dH_3=Ty(q_0) * Tlink1 *dTz* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_3=Jnum(dH_3);
dH_4=Ty(q_0) * Tlink1 *dRx(0)* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_4=Jnum(dH_4);
dH_5=Ty(q_0) * Tlink1 *dRy(0)* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_5=Jnum(dH_5);
dH_6=Ty(q_0) * Tlink1 *dRz(0)* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_6=Jnum(dH_6);

%T3D for virtual joint of second link >>>> J_theta
dH_7=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *dTx* R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_7=Jnum(dH_7);
dH_8=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *dTy* R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_8=Jnum(dH_8);
dH_9=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *dTz* R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_9=Jnum(dH_9);
dH_10=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *dRx(0)* R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_10=Jnum(dH_10);
dH_11=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *dRy(0)* R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_11=Jnum(dH_11);
dH_12=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *dRz(0)* R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_12=Jnum(dH_12);

%T3D for virtual joint of third link >>>> J_theta
dH_13=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2* R_T(Ry(q_2))*Tlink3 *dTx* R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_13=Jnum(dH_13);
dH_14=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 *dTy* R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_14=Jnum(dH_14);
dH_15=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 *dTz* R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_15=Jnum(dH_15);
dH_16=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 *dRx(0)* R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_16=Jnum(dH_16);
dH_17=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 *dRy(0)* R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_17=Jnum(dH_17);
dH_18=Ty(q_0) * Tlink1 *dTx* R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 *dRz(0)* R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_18=Jnum(dH_18);

%T3D for virtual joint of fourth link >>>> J_theta
dH_19=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 *dTx* Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_19=Jnum(dH_19);
dH_20=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 *dTy* Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_20=Jnum(dH_20);
dH_21=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 *dTz* Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_21=Jnum(dH_21);
dH_22=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 *dRx(0)* Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_22=Jnum(dH_22);
dH_23=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 *dRy(0)* Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_23=Jnum(dH_23);
dH_24=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 *dRz(0)* Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_24=Jnum(dH_24);


%T3D for virtual joint of fifth link >>>> J_theta
dH_25=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 *dTx* R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_25=Jnum(dH_25);
dH_26=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 *dTy* R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_26=Jnum(dH_26);
dH_27=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 *dTz* R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_27=Jnum(dH_27);
dH_28=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5*dRx(0) * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_28=Jnum(dH_28);
dH_29=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 *dRy(0)* R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_29=Jnum(dH_29);
dH_30=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 *dRz(0)* R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6;
J_30=Jnum(dH_30);

%T3D for virtual joint of sixth link >>>> J_theta
dH_31=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6*dTx;
J_31=Jnum(dH_31);
dH_32=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6*dTy;
J_32=Jnum(dH_32);
dH_33=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6*dTz;
J_33=Jnum(dH_33);
dH_34=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6*dRx(0);
J_34=Jnum(dH_34);
dH_35=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6*dRy(0);
J_35=Jnum(dH_35);
dH_36=Ty(q_0) * Tlink1 * R_T(Rz(q_1))*Tlink2 *R_T(Ry(q_2))*Tlink3 * R_T(Ry(q_3)) * Tlink4 * Tlink5 * R_T(Rx(q_4)) *R_T(Ry(q_5)) * R_T(Rx(q_6)) * Tlink6*dRz(0);
J_36=Jnum(dH_36);


J_theta=[J_q1,J_q2,J_q3,J_q4,J_q5,J_q6,J_q7,...
         J_1,J_2,J_3,J_4,J_5,J_6,...
         J_7,J_8,J_9,J_10,J_11,J_12,...
         J_13,J_14,J_15,J_16,J_17,J_18,...
         J_19,J_20,J_21,J_22,J_23,J_24,...
         J_25,J_26,J_27,J_28,J_29,J_30,...
         J_31,J_32,J_33,J_34,J_35,J_36];
%k_c_0
K_l1=K_links(1:6,:);
K_l2=K_links(7:12,:);
K_l3=K_links(13:18,:);
K_l4=K_links(19:24,:);
K_l5=K_links(25:30,:);
K_l6=K_links(31:36,:);
K_t=[diag([K_act,K_act,K_act,K_act,K_act,K_act,K_act]), zeros(7,36);
     zeros(36,7),blkdiag(K_l1,K_l2,K_l3,K_l4,K_l5,K_l6)];       

%Reduced force-deflection relation
Kci=inv(J_theta*inv(K_t)*J_theta');
Kc=Kc+Kci(1:6,1:6);
end

function [H]=dTx()
H=zeros(4);
H(1,4)=1;
end

function [H]=dTy()
H=zeros(4);
H(2,4)=1;
end

function [H]=dTz()
H=zeros(4);
H(3,4)=1;
end

function [H]=dRx(alpha)
H=zeros(4);
H(1:3,1:3)=Rx(pi/2+alpha);
H(1,1)=0;
end

function [H]=dRy(alpha)
H=zeros(4);
H(1:3,1:3)=Ry(pi/2+alpha);
H(2,2)=0;
end

function [H]=dRz(alpha)
H=zeros(4);
H(1:3,1:3)=Rz(pi/2+alpha);
H(3,3)=0;
end

function [J]=Jnum(dH)
J=[dH(1,4); dH(2,4); dH(3,4); dH(3,2); dH(1,3); dH(2,1)];
end