function gamma = IDM(Q,Qp,Qdp)
gamma=zeros(2,1);
Q2=Q(2);
Qp1=Qp(1);
Qp2=Qp(2);
Qdp1=Qdp(1);
Qdp2=Qdp(2);
M1 = 1.4899
L1=0.2;M2=0.464;ZZ1=0.0045;ZZ2=0.00156;
MY2=0;MX2=0.025686;
gamma(1) = Qdp1*(ZZ1 + ZZ2 + L1^2*M2 + 2*L1*MX2*cos(Q2) - 2*L1*MY2*sin(Q2)) + Qdp2*(ZZ2 + L1*MX2*cos(Q2) - L1*MY2*sin(Q2)) - L1*Qp2*(Qp1 + Qp2)*(MY2*cos(Q2) + MX2*sin(Q2)) - L1*Qp1*Qp2*(MY2*cos(Q2) + MX2*sin(Q2));
gamma(2) =  Qdp2*ZZ2 + Qdp1*(ZZ2 + L1*MX2*cos(Q2) - L1*MY2*sin(Q2)) + (L1*Qp1*(MY2*cos(Q2) + MX2*sin(Q2))*(Qp1 - Qp2))/2;
 end
