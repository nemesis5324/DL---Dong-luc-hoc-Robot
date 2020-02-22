function T0n = SymDGM(sigma,a,d,t,r,q)
global T01 T12 T23 T34 T45 T56
% This function calculates direct geometric model
% T0n - ma tran bien doi thuan nhat cua khau cuoi
% (end-effector) trong he toa do R0
% sigma(j) = 1 if joint j is prismatic
% sigma(j) = 0 if joint j is revolute
% a - the angle between Z(j-1) and Zj about Xj
% d - the distance between Z(j-1) and Zj along Xj
% t - the angle between X(j-1) and Xj about Zj
% r - the distance between X(j-1) and Xj along Zj
% q - joint variables

% Convert all input vector to symbolic
a = sym(a);
d = sym(d);
t = sym(t);
r = sym(r);
n = length(q);  % number of joint variables
T0n = eye(4);
for j=1:n
    if(sigma(j)==1)
        r(j)=q(j);
    else
        t(j)=q(j);
    end
    Ct = cos(t(j));
    St = sin(t(j));
    Ca = cos(a(j));
    Sa = sin(a(j));
%     Ma tran (j-i)Tj:
% Calculate T01, T12,...T56
    Tj = [   Ct      -St      0       d(j)
          Ca*St    Ca*Ct    -Sa   -r(j)*Sa
          Sa*St    Sa*Ct     Ca    r(j)*Ca
              0        0      0         1];
    switch j
        case 1
            T01 = Tj;
        case 2
            T12 = Tj;
        case 3
            T23 = Tj;
        case 4
            T34 = Tj;
        case 5
            T45 = Tj;
        case 6
            T56 = Tj;
    end
%     disp(['T',num2str(j-1),num2str(j),'=']);
%     disp(Tj);
    T0n = T0n*Tj;
    T0n = simplify(T0n);
end

