function [J, J1, J2, J3, J4, J5, J6, J7] =  Jacobian(q, link_lengths)
%% Extracting the link lengths
d_0 = link_lengths(1);
d_1 = link_lengths(2);
d_2 = link_lengths(3);
d_3 = link_lengths(4);
d_4 = link_lengths(5);
d_5 = link_lengths(6);

q_0 = q(1);
q_1 = q(2);
q_2 = q(3);
q_3 = q(4);
q_4 = q(5);
q_5 = q(6);
q_6 = q(7);

%% Getting the forward kinematics
T =  FK(q, link_lengths);
T(1:3, 4) = 0;

%% Getting the jacobians
Td = dTy*Tz(d_0)*R_T(Rz(q_1))*Tx(d_1)*R_T(Ry(q_2))*Tx(d_2)*R_T(Ry(q_3))*Tx(d_3)*Tz(d_4)*R_T(Rx(q_4))*R_T(Ry(q_5))*R_T(Rx(q_6))*Tx(d_5) / T;
 
J1 = Jnum(Td);

Td = Ty(q_0)*Tz(d_0)*dRz(q_1)*Tx(d_1)*R_T(Ry(q_2))*Tx(d_2)*R_T(Ry(q_3))*Tx(d_3)*Tz(d_4)*R_T(Rx(q_4))*R_T(Ry(q_5))*R_T(Rx(q_6))*Tx(d_5) / T;

J2 = Jnum(Td);

Td = Ty(q_0)*Tz(d_0)*R_T(Rz(q_1))*Tx(d_1)*dRy(q_2)*Tx(d_2)*R_T(Ry(q_3))*Tx(d_3)*Tz(d_4)*R_T(Rx(q_4))*R_T(Ry(q_5))*R_T(Rx(q_6))*Tx(d_5) / T;
 
J3 = Jnum(Td);

Td = Ty(q_0)*Tz(d_0)*R_T(Rz(q_1))*Tx(d_1)*R_T(Ry(q_2))*Tx(d_2)*dRy(q_3)*Tx(d_3)*Tz(d_4)*R_T(Rx(q_4))*R_T(Ry(q_5))*R_T(Rx(q_6))*Tx(d_5) / T;
 
J4 = Jnum(Td);

Td = Ty(q_0)*Tz(d_0)*R_T(Rz(q_1))*Tx(d_1)*R_T(Ry(q_2))*Tx(d_2)*R_T(Ry(q_3))*Tx(d_3)*Tz(d_4)*dRx(q_4)*R_T(Ry(q_5))*R_T(Rx(q_6))*Tx(d_5) / T;
 
J5 = Jnum(Td);

Td = Ty(q_0)*Tz(d_0)*R_T(Rz(q_1))*Tx(d_1)*R_T(Ry(q_2))*Tx(d_2)*R_T(Ry(q_3))*Tx(d_3)*Tz(d_4)*R_T(Rx(q_4))*dRy(q_5)*R_T(Rx(q_6))*Tx(d_5) / T;
 
J6 = Jnum(Td);

Td = Ty(q_0)*Tz(d_0)*R_T(Rz(q_1))*Tx(d_1)*R_T(Ry(q_2))*Tx(d_2)*R_T(Ry(q_3))*Tx(d_3)*Tz(d_4)*R_T(Rx(q_4))*R_T(Ry(q_5))*dRx(q_6)*Tx(d_5) / T;
 
J7 = Jnum(Td);

J = [J1, J2, J3, J4, J5, J6, J7];
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

function [T]=dRx(q)
Sq = sin(q);    Cq = cos(q);

T = [   0   0     0     0; ...
        0  -Sq   -Cq    0; ...
        0   Cq   -Sq    0; ...
        0   0     0     0];
end

function [T]=dRy(q)
Sq = sin(q);    Cq = cos(q);

T = [  -Sq   0    Cq    0; ...
        0    0    0     0; ...
       -Cq   0   -Sq    0; ...
        0    0    0     0];
end

function [T]=dRz(q)
Sq = sin(q);    Cq = cos(q);

T = [  -Sq  -Cq   0    0; ...
        Cq  -Sq   0    0; ...
        0    0    0    0; ...
        0    0    0    0];
end
% 
% function [H]=dRy(alpha)
% H=zeros(4);
% H(1:3,1:3)=Ry(pi/2+alpha);
% H(2,2)=0;
% end
% 
% function [H]=dRz(alpha)
% H=zeros(4);
% H(1:3,1:3)=Rz(pi/2+alpha);
% H(3,3)=0;
% end

function [J]=Jnum(dH)
J=[dH(1,4); dH(2,4); dH(3,4); dH(3,2); dH(1,3); dH(2,1)];
end



