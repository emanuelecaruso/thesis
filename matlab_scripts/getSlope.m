clear
clc

syms f d2 w h u1 v1 u2 v2 r00 r01 r02 r10 r11 r12 r20 r21 r22 t0 t0 t1 t2 dfix
syms slope1 slope2 vh

k=[f  0 -w/2 ;
   0 -f -h/2;
   0  0 -1       ];

kinv = inv(k);

R=[r00 r01 r02;
   r10 r11 r12;
   r20 r12 r22];

Rinv = inv(R);

t=[t0; t1; t2];

T=[R t; 0 0 0 1];

Tinv = [Rinv -Rinv*t];
      

d1=1;
p_proj1 = [u1*d1;v1*d1;d1]; %uv and depth on cam1 (d1 is known)

p_proj2 = [u2*d2;v2*d2;d2]; %uv and depth on cam2 (d2 is unknown)

a=[kinv*p_proj2;1]; %3D point in cam2 frame + 1 for homogeneous transf
b=[kinv*p_proj1;1]; %3D point in cam1 frame + 1 for homogeneous transf

mlt=T*b; %T (cam1 expressed in cam2)

cam1_in_cam2= k*t;
cam1_in_cam2=cam1_in_cam2/(cam1_in_cam2(3));
cam1_in_cam2=cam1_in_cam2(1:2);
cam1_in_cam2=simplify(cam1_in_cam2,'Steps',1000);

% cam2_in_cam1= Tinv*[0;0;0;1];
% cam2_in_cam1=cam2_in_cam1/(cam2_in_cam1(3));
% cam2_in_cam1=cam2_in_cam1(1:2);
% cam2_in_cam1=simplify(cam2_in_cam1,'Steps',1000);

% d2 function as d1
% disp("d2")
expr1 = mlt(3) == a(3)
slv_d2 = solve(expr1,d2);
% pretty(simplify(slv_d2,'Steps',1000));

% disp("u2")
expr2 = mlt(1) == a(1);
slv_u2 = solve(expr2,u2);
slv_u2 = simplify(slv_u2,'Steps',1000);
% pretty(slv_u2)

% disp("v2")
expr3 = mlt(2) == a(2);
slv_v2 = solve(expr3,v2);
slv_v2 = simplify(slv_v2,'Steps',1000);
% pretty(slv_v2)

disp("slope")
slope=(slv_v2-cam1_in_cam2(2))/(slv_u2-cam1_in_cam2(1));
slope=simplify(slope,'Steps',1000);
slope = subs(slope,d2,slv_d2);
slope=simplify(slope,'Steps',1000);
slope = collect(slope,[u1,v1])
[N,D] = numden(slope);

CN = coeffs(N,[u1,v1]);
CD = coeffs(D,[u1,v1]);

% A*u1+B*v1*C
% -----------
% D*u1+E*v1+F

A=CN(3);    %u1
B=CN(2);    %v1
C=CN(1);    %note term

D=CD(3);    %u1
E=CD(2);    %v1
F=CD(1);    %note term


A=simplify(A,'Steps',1000);
B=simplify(B,'Steps',1000);
C=simplify(C,'Steps',1000);
D=simplify(D,'Steps',1000);
E=simplify(E,'Steps',1000);
F=simplify(F,'Steps',1000);

disp("A")
disp(A);
disp("B")
disp(B);
disp("C")
disp(C);

disp("D")
disp(D);
disp("E")
disp(E);
disp("F")
disp(F);

%% verify


% A*u1+B*v1*C
% -----------
% D*u1+E*v1+F




