function [T,T1, T2, T3, T4, T5, T6, T7] =  FK(q, link_lengths)
%FK Summary of this function goes here
%% Extracting the link lengths
d_0=link_lengths(1);  %base length
d_1=link_lengths(2);  %link 1 length
d_2=link_lengths(3);  %link 2 length
d_3=link_lengths(4);  %link 3 length (between in j3 and SW)
d_4=link_lengths(5);  %shift in z between j3 and j4
d_5=link_lengths(6);  %link 4 length (between SW and ee)

q_0 = q(1);
q_1 = q(2);
q_2 = q(3);
q_3 = q(4);
q_4 = q(5);
q_5 = q(6);
q_6 = q(7);

%% Transformations
T1 = Ty(q_0);

T2 = T1 * Tz(d_0) * R_T(Rz(q_1)) ;

T3 = T2 * Tx(d_1) * R_T(Ry(q_2));

T4 = T3 * Tx(d_2) * R_T(Ry(q_3));

T5 = T4 *  Tz(d_4) *Tx(d_3) * R_T(Rx(q_4));

T6 = T5 * R_T(Ry(q_5));

T7 = T6 * R_T(Rx(q_6));

T = T7  * Tx(d_5);




end

