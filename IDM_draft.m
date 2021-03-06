% function gamma = IDM(Q, QP, QPP)
%% IDM
% Calculate A(Q)
clear;clc;
syms Q1 Q2 QP1 QP2 QPP1 QPP2
syms L1 M1 MX1 MY1 MZ1 XX1 XY1 XZ1 YY1 YZ1 ZZ1
syms L2 M2 MX2 MY2 MZ2 XX2 XY2 XZ2 YY2 YZ2 ZZ2
Q=[Q1;Q2];
QP=[QP1;QP2];
QPP=[QPP1;QPP2];
V0=[0;0;0];
V1=[0;0;0];
W0=[0;0;0];
V2=[L1*sin(Q2)*QP1
    L1*cos(Q2)*QP1
        0  ];
W1=[0;0;QP1];
W2=[0;0;QP1+QP2];
J11=[XX1 XY1 XZ1
     XY1 YY1 YZ1
     XZ1 YZ1 ZZ1];
J22=[XX2 XY2 XZ2
     XY2 YY2 YZ2
     XZ2 YZ2 ZZ2];
MS2=[MX2;MY2;MZ2];
E1=1/2*transpose(W1)*J11*W1;
E2=1/2*(transpose(W2)*J22*W2 + M2*V2.'*V2 + 2*MS2.'*cross(V2,W2));
E=E1+E2;
A11=diff(diff(E,QP1),QP1);
A11=simplify(A11);

A12=diff(diff(E,QP1),QP2);
A12=simplify(A12);

A21=diff(diff(E,QP2),QP1);
A21=simplify(A21);

A22=diff(diff(E,QP2),QP2);
A22=simplify(A22);

A=[A11 A12
   A21 A22];
C=zeros(2,2);
C=sym(C);
% Calculate C
for i=1:2
    for j=1:2
        for k=1:2
            temp=1/2*(diff(A(i,j),Q(k))...
                +diff(A(i,k),Q(j))...
                -diff(A(j,k),Q(i)));
            C(i,j)=C(i,j)+temp*QP(k);
        end
    end
end
gamma = A*QPP + C*QP;
