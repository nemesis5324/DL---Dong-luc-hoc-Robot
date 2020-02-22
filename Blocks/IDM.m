function [invJ,Qp] = IDM(Q,V)
L1=0.2;
L2=0.1442;
q1=Q(1);
q2=Q(2);
c1=cos(q1);
s1=sin(q1);
c12=cos(q1+q2);
s12=sin(q1+q2);
J=[-L2*s12-L1*s1,-L2*s12;
    L2*c12+L1*c1,L2*c12];
Qp=J^-1*V;
invJ=J^-1;