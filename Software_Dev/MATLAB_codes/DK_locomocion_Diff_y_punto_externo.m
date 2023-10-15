clear all
clc

syms V theta_p theta d_B V_der V_izq R
syms VB_x VB_y

       
V = (V_der+V_izq)/2;
theta_p = (V_der-V_izq)/(2*R);

mth_o_c = [cos(theta_p), -sin(theta_p), 0, V*cos(theta);
           sin(theta_p), cos(theta_p),  0, V*sin(theta);
                    0,            0,  1,            0;
                    0,            0,  0,            1];
                
mth_c_B = [1, 0, 0,   0;
           0, 1, 0,   theta_p*d_B;
           0, 0, 1,   0;
           0, 0, 0,   1];


mth_o_b = mth_o_c * mth_c_B;

equ1 = VB_x == mth_o_b(1,4)
equ2 = VB_y == mth_o_b(2,4)

sys_equs = [equ1,equ2];

IK_barra = solve(sys_equs,[V_der,V_izq])

IK_barra.V_der
IK_barra.V_izq


