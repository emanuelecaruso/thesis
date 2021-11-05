clear
clc

syms f d d2 width height u1 v1 u2 v2 r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3
syms slope1 slope2 vh

k=[f  0 -width/2 ;
   0 -f -height/2;
   0  0 -1       ];

kinv = inv(k);
T= [r11 r12 r13 t1; r21 r22 r23 t2; r31 r32 r33 t3; 0 0 0 1]; %transform from cam2 to cam1
% T= [1 0 0 t1; 0 1 0 t2; 0 0 1 t3; 0 0 0 1]; %transform from cam2 to cam1

p_proj1 = [u1*d;v1*d;d]; %uv and depth on cam1 (d is known)
p_proj2 = [u2*d2;v2*d2;d2]; %uv and depth on cam2 (d2 is unknown)

a=[kinv*p_proj2;1]; %3D point in cam1 frame + 1 for homogeneous transf
b=[kinv*p_proj1;1]; %3D point in cam2 frame + 1 for homogeneous transf

mlt=T*b; %3D point from cam2 to cam1 (has to be the same as a)

% d2 function as d
disp("d2")
expr1 = mlt(3) == a(3);
slv_d2 = solve(expr1,d2);
pretty(simplify(slv_d2,'Steps',1000));


% uv coords that depends on d
disp("u2")
expr2 = mlt(1) == a(1);
slv_u2 = solve(expr2,u2);
slv_u2 = simplify(slv_u2,'Steps',1000);
slv_u2 = subs(slv_u2,d2,slv_d2);
slv_u2 = simplify(slv_u2,'Steps',1000);
slv_u2 = subs(slv_u2,v1,u1*slope1+vh);
slv_u2 = simplify(slv_u2,'Steps',1000);
slv_u2 = collect(slv_u2,u1);
% slv_u2 = simplify(slv_u2,'Steps',1000);
pretty(slv_u2);


% disp("v2")
% expr3 = mlt(2) == a(2);
% slv_v2 = solve(expr3,v2);
% slv_v2 = simplify(slv_v2,'Steps',1000);
% slv_v2 = subs(slv_v2,d2,slv_d2);
% slv_v2 = simplify(slv_v2,'Steps',1000);
% pretty(slv_v2);

