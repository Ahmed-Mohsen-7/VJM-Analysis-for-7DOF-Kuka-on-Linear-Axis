function out = Jparams_Kuka(q,params,Tbase,Ttool)

Htx = [0 0 0 1;
       0 0 0 0; 
       0 0 0 0;
       0 0 0 0];

Hty = [0 0 0 0;
       0 0 0 1;
       0 0 0 0; 
       0 0 0 0];

Htz = [0 0 0 0;
       0 0 0 0;
       0 0 0 1;
       0 0 0 0];
   
Hrx = [0 0 0 0;
       0 0 -1 0;
       0 1 0 0;
       0 0 0 0];

Hry = [0 0 1 0;
       0 0 0 0;
      -1 0 0 0;
       0 0 0 0];

Hrz = [0 -1 0 0;
       1 0 0 0;
       0 0 0 0; 
       0 0 0 0];
   
   
% Td = Tbase*R_T(Rz(q(1)))*Htx *Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J1 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*Hty*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J2 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*Hrx*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J3 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Hrx*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool; 
% J4 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*Hty*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J5 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*Hry*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool; 
% J6 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*Hrz*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J7 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Hrx*Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J8 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Hty*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J9 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))*Htz* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J10 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))*Hrz* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J11 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))*R_T(Ry(q(4)))*Hry*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J12 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*Htx*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J13 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*Htz*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J14 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*Hrz*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
% J15 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*Hrx*R_T(Ry(q(6)))*Ttool;
% J16 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*Htz*R_T(Ry(q(6)))*Ttool;
% J17 = [Td(1,4), Td(2,4), Td(3,4)]';
% 
% Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
%      Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*Hrz*R_T(Ry(q(6)))*Ttool;
% J18 = [Td(1,4), Td(2,4), Td(3,4)]';


d_1=.350;  %link 1 length-203.2
d_2=1.150; %link 2 length
d_3=1.200; %link 3 length (between in j3 and SW)
d_4=-.041;  %shift in z between j3 and j4 
d5 = 0.215;  %link 5 length (between in SW and ee)

Td = Tbase*R_T(Rz(q(1)))*Ty(d_1)*R_T(Rx(q(2)))*Hrx*Ty(d_2)*R_T(Rx(q(3)))*...
     Ty(d_3)*Tz(d_4)*R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ty(d5)*Ttool; 
J1 = [Td(1,4), Td(2,4), Td(3,4)]';

Td= Tbase*R_T(Rz(q(1)))*Ty(d_1)*R_T(Rx(q(2)))*Ty(d_2)*R_T(Rx(q(3)))*Hrx*...
     Ty(d_3)*Tz(d_4)*R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ty(d5)*Ttool;
J2 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Ty(d_1)*R_T(Rx(q(2)))*Ty(d_2)*R_T(Rx(q(3)))*...
     Ty(d_3)*Tz(d_4)*R_T(Ry(q(4)))*Hry*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ty(d5)*Ttool; 
J3 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Ty(d_1)*R_T(Rx(q(2)))*Ty(d_2)*R_T(Rx(q(3)))*...
     Ty(d_3)*Tz(d_4)*R_T(Ry(q(4)))*R_T(Rx(q(5)))*Hrx*R_T(Ry(q(6)))*Ty(d5)*Ttool;  
J4 = [Td(1,4), Td(2,4), Td(3,4)]';

% out = [J1 J2 J3 J4 J5 J6 J7 J8 J9 J10 J11 J12 J13 J14 J15 J16 J17 J18];
out = [J1 J2 J3 J4];
end