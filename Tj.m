%%
clear;clc;
syms a d t r
ca = cos(a);
sa = sin(a);
ct = cos(t);
st = sin(t);

rot_x_a = [1  0  0  0
           0 ca -sa 0
           0 sa  ca 0
           0  0   0 1];
trans_x_d = [1 0 0 d
             0 1 0 0
             0 0 1 0
             0 0 0 1];
rot_z_t = [ ct -st  0  0
            st  ct  0  0
             0   0  1  0
             0   0  0  1];
trans_z_r = [1 0 0 0
             0 1 0 0
             0 0 1 r
             0 0 0 1];
rot_x_a*trans_x_d*rot_z_t*trans_z_r