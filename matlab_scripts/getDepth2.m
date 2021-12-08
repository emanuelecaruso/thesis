clear
clc

syms f d1 d2 w h u1 v1 u2 v2 r00 r01 r02 r10 r11 r12 r20 r21 r22 t0 t0 t1 t2
syms slope1 slope2 vh

k=[f  0 -w/2 ;
   0 -f -h/2;
   0  0 -1       ];

kinv = inv(k);

T= [r00 r01 r02 t0;
    r10 r11 r12 t1;
    r20 r12 r22 t2;
    0 0 0 1];
      
% T= [1 0 0 t0; 0 1 0 t1; 0 0 1 t2; 0 0 0 1];

p_proj1 = [u1*d1;v1*d1;d1]; %uv and depth on cam1 (d is known)
p_proj2 = [u2*d2;v2*d2;d2]; %uv and depth on cam2 (d2 is unknown)

a=[kinv*p_proj2;1]; %3D point in cam2 frame + 1 for homogeneous transf
b=[kinv*p_proj1;1]; %3D point in cam1 frame + 1 for homogeneous transf

mlt=T*b; %T transform cam1 to cam2 (cam1 expressed in cam2)


% d2 function as d1
disp("d2")
expr1 = mlt(3) == a(3);
slv_d2 = solve(expr1,d2);
slv_d2 = collect(slv_d2,[u1,v1,d1])
CN = coeffs(slv_d2,[u1,v1,d1]);
A=CN(4);
B=CN(3);
C=CN(2);
D=CN(1);

A=simplify(A,'Steps',1000);
B=simplify(B,'Steps',1000);
C=simplify(C,'Steps',1000);
D=simplify(D,'Steps',1000);

disp("A")
disp(A);
disp("B")
disp(B);
disp("C")
disp(C);
disp("D")
disp(D);
