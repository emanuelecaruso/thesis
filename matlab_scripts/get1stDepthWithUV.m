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
pretty(simplify(slv_d2,'Steps',1000));


% % uv coords that depends on d
% disp("u2")
% expr2 = mlt(1) == a(1);
% slv_u2 = solve(expr2,u2);
% slv_u2 = subs(slv_u2,d2,slv_d2);
% slv_u2 = subs(slv_u2,v1,u1*slope1+vh);
% slv_u2 = collect(slv_u2,u1);
% [N,D] = numden(slv_u2);
% CN = coeffs(N,u1);
% CD = coeffs(D,u1);
% A=CN(2);
% B=CN(1);
% C=CD(2);
% D=CD(1);
% A=simplify(A,'Steps',1000);
% B=simplify(B,'Steps',1000);
% C=simplify(C,'Steps',1000);
% D=simplify(D,'Steps',1000);
% disp("A")
% pretty(A);
% disp("B")
% pretty(B);
% disp("C")
% pretty(C);
% disp("D")
% pretty(D);



disp("v2")
expr3 = mlt(2) == a(2);
slv_u3 = solve(expr3,v2);
slv_u3 = subs(slv_u3,d2,slv_d2);
slv_u3 = subs(slv_u3,u1,v1/slope1+vh);
slv_u3 = collect(slv_u3,v1)
[N,D] = numden(slv_u3);
CN = coeffs(N,v1);
CD = coeffs(D,v1);
A=CN(2);
B=CN(1);
C=CD(2);
D=CD(1);
A=simplify(A,'Steps',1000);
B=simplify(B,'Steps',1000);
C=simplify(C,'Steps',1000);
D=simplify(D,'Steps',1000);
disp("A")
pretty(A);
disp("B")
pretty(B);
disp("C")
pretty(C);
disp("D")
pretty(D);
