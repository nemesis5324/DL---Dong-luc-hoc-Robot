function [Q,Qd,Qdp] = fcn(time,Ps,tf)
n=length(tf);
[k,tk_1] = find(time,tf,n);
if(k==1)
    t=min(time/tf(1),1);
else
    t = min((time-tk_1)/tf(k),1);
end
r=10*t^3-15*t^4+6*t^5;
rd=30*t^2-60*t^3+30*t^4;
r2d=60*t-180*t^2+120*t^3;
D=Ps(:,k+1)-Ps(:,k);
Q=Ps(:,k)+r*D;
Qd=rd*D;
Qdp=r2d*D;
function [k,tk_1]=find(time,tf,n)
t=0;
k=n;
for i=1:n
    tk=t+tf(i);
    if(t<=time)&&(time<tk)
        k=i;
        break;
    end
    t=tk;
    if(i==n)
        t=tk-tf(n);
    end
end
tk_1=t;
