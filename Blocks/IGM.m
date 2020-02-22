function Q = IGM(P,Qt)
L1=0.2;
L2=0.1442;
x=P(1);
y=P(2);
q2=-acos((x^2+y^2-L1^2-L2^2)/2/L1/L2);
q1=cmp(x,y,L1,L2,q2);
k=1;
while ((q1<0)&&(Qt(1)>(2*k-1)*pi/2))
    q1=q1+2*k*pi;
    k=k+1;
end
k=1;
while ((q1>=0)&&(Qt(1)>(2*k+1)*pi/2)&&(q1<=pi))
    q1=q1+2*k*pi;
    k=k+1;
end
Q=[q1;q2];
function q1=cmp(x,y,L1,L2,q2)
L1x=L1+L2*cos(q2);
L1y=L2*sin(q2);
q=[L1x -L1y;L1y L1x]\[x;y];
q1=atan2(q(2),q(1));

