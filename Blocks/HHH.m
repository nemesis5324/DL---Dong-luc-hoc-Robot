function QDP  = HHH(QP,Q,PDP,invJ)
L1=0.2;
L2=0.1442;
q1=Q(1);
q2=Q(2);
c1=cos(q1);
s1=sin(q1);
c12=cos(q1+q2);
s12=sin(q1+q2);
Jp=[-L2*c12*(QP(1)+QP(2))-L1*c1*QP(1), -L2*c12*(QP(1)+QP(2))
    -L2*s12*(QP(1)+QP(2))-L1*s1*QP(1), -L2*s12*(QP(1)+QP(2))];
QDP=invJ*(PDP-Jp*QP);