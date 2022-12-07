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
   
   
Td = Tbase*R_T(Rz(q(1)))*Hty *Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))* R_T(Rx(q(4)))*R_T(Ry(q(5)))*R_T(Rx(q(6)))*Ttool;
J1 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*Htx*R_T(Ry(q(2)))*Tx(params(5))*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))* R_T(Rx(q(4)))*R_T(Ry(q(5)))*R_T(Rx(q(6)))*Ttool;
J2 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*Hrx*R_T(Ry(q(2)))*Tx(params(5))*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))* R_T(Rx(q(4)))*R_T(Ry(q(5)))*R_T(Rx(q(6)))*Ttool;
J3 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Hry*Tx(params(5))*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))* R_T(Rx(q(4)))*R_T(Ry(q(5)))*R_T(Rx(q(6)))*Ttool; 
J4 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*Htx*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))* R_T(Rx(q(4)))*R_T(Ry(q(5)))*R_T(Rx(q(6)))*Ttool; 
J5 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*Hrx*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))* R_T(Rx(q(4)))*R_T(Ry(q(5)))*R_T(Rx(q(6)))*Ttool;  
J6 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*Hrz*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))* R_T(Rx(q(4)))*R_T(Ry(q(5)))*R_T(Rx(q(6)))*Ttool;
J7 = [Td(1,4), Td(2,4), Td(3,4)]';

Td =  Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*R_T(Ry(q(3)))*Hry*...
     Tx(params(9))*Tz(params(10))* R_T(Rx(q(4)))*R_T(Ry(q(5)))*R_T(Rx(q(6)))*Ttool;
J8 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*R_T(Ry(q(3)))*...
     Tx(params(9))*Htx*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
J9 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))*Htz*R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
J10 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))*Hrz*R_T(Ry(q(4)))*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
J11 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))*R_T(Ry(q(4)))*Hrx*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
J12 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Tx(params(2))*R_T(Ry(q(2)))*Tx(params(5))*R_T(Ry(q(3)))*...
     Tx(params(9))*Tz(params(10))*R_T(Ry(q(4)))*Hty*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
J13 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
     Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*Htz*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
J14 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
     Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*Hrz*R_T(Rx(q(5)))*R_T(Ry(q(6)))*Ttool;
J15 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
     Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*Hry*R_T(Ry(q(6)))*Ttool;
J16 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
     Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*Htz*R_T(Ry(q(6)))*Ttool;
J17 = [Td(1,4), Td(2,4), Td(3,4)]';

Td = Tbase*R_T(Rz(q(1)))*Ty(params(2))*R_T(Rx(q(2)))*Ty(params(5))*R_T(Rx(q(3)))*...
     Ty(params(9))*Tz(params(10))* R_T(Ry(q(4)))*R_T(Rx(q(5)))*Hrz*R_T(Ry(q(6)))*Ttool;
J18 = [Td(1,4), Td(2,4), Td(3,4)]';

out = [J1 J2 J3 J4 J5 J6 J7 J8 J9 J10 J11 J12 J13 J14 J15 J16 J17 J18];
end