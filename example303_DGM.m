%% Example 3.3, Symbolic direct geometric model 
% of the Staubli RX-90 robot 
% Using function T0n = SymDGM(sigma,a,d,t,r,q)
clear;clc;
global T01 T12 T23 T34 T45 T56
syms t1 t2 t3 t4 t5 t6 d3 r4
sigma = [0 0 0 0 0 0];
a = [0 pi/2 0 -pi/2 pi/2 -pi/2];
d = [0 0 d3 0 0 0];
t = [t1 t2 t3 t4 t5 t6];
q=t;
r = [0 0 0 r4 0 0];
T0n = SymDGM(sigma,a,d,t,r,q);
Sx = T0n(1,1);

syms S1 S2 S3 S4 S5 S6 S23
syms C1 C2 C3 C4 C5 C6 C23
S = [S1 S2 S3 S4 S5 S6];
C = [C1 C2 C3 C4 C5 C6];
for i=1:6
    T01=subs(T01,sin(t(i)),S(i));
    T01=subs(T01,cos(t(i)),C(i));
    
    T12=subs(T12,sin(t(i)),S(i));
    T12=subs(T12,cos(t(i)),C(i));
    
    T23=subs(T23,sin(t(i)),S(i));
    T23=subs(T23,cos(t(i)),C(i));
    
    T34=subs(T34,sin(t(i)),S(i));
    T34=subs(T34,cos(t(i)),C(i));
    
    T45=subs(T45,sin(t(i)),S(i));
    T45=subs(T45,cos(t(i)),C(i));
    
    T56=subs(T56,sin(t(i)),S(i));
    T56=subs(T56,cos(t(i)),C(i));
    
    T0n=subs(T0n,sin(t(i)),S(i));
    T0n=subs(T0n,cos(t(i)),C(i));
end
T0n = subs(T0n,sin(t2+t3),S23);
T0n = subs(T0n,cos(t2+t3),C23);

%% Optimization of the computation of DGM


