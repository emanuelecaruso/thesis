clear
clc

syms f d1 d2 w h u1 v1 u2 v2 r00 r01 r02 r10 r11 r12 r20 r21 r22 t0 t0 t1 t2
syms slope1 slope2 vh

k=[f  0 w/2 ;
   0  f h/2;
   0  0 1       ];

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

%%

% u2=f(u1,v1,d1) (get coord as u2)
disp("u2")
expr2 = mlt(1) == a(1);
slv_u2 = solve(expr2,u2);
slv_u2 = subs(slv_u2,d2,slv_d2);
slv_u2 = collect(slv_u2,[u1,v1,d1])

[N,D] = numden(slv_u2);
CN = coeffs(N,[u1,v1,d1]);
CD = coeffs(D,[u1,v1,d1]);

% A*u1*d1 + B*v1*d1 + C*d1 + D
% ----------------------------
% E*u1*d1 + F*v1*d1 + G*d1 + H

A=CN(4);
B=CN(3);
C=CN(2);
D=CN(1);

E=CD(4);
F=CD(3);
G=CD(2);
H=CD(1);

A=simplify(A,'Steps',1000);
B=simplify(B,'Steps',1000);
C=simplify(C,'Steps',1000);
D=simplify(D,'Steps',1000);
E=simplify(E,'Steps',1000);
F=simplify(F,'Steps',1000);
G=simplify(G,'Steps',1000);
H=simplify(H,'Steps',1000);

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
disp("G")
disp(G);
disp("H")
disp(H);

%%

% v2=f(u1,v1,d1) (get coord as v2)
disp("v2")
expr3 = mlt(2) == a(2);
slv_v2 = solve(expr3,v2);
slv_v2 = subs(slv_v2,d2,slv_d2);
slv_v2 = collect(slv_v2,[u1,v1,d1]);

[N,D] = numden(slv_v2);
CN = coeffs(N,[u1,v1,d1]);
CD = coeffs(D,[u1,v1,d1]);

% A*u1*d1 + B*v1*d1 + C*d1 + D
% ----------------------------
% E*u1*d1 + F*v1*d1 + G*d1 + H

A=CN(4);
B=CN(3);
C=CN(2);
D=CN(1);

E=CD(4);
F=CD(3);
G=CD(2);
H=CD(1);

A=simplify(A,'Steps',1000);
B=simplify(B,'Steps',1000);
C=simplify(C,'Steps',1000);
D=simplify(D,'Steps',1000);
E=simplify(E,'Steps',1000);
F=simplify(F,'Steps',1000);
G=simplify(G,'Steps',1000);
H=simplify(H,'Steps',1000);

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
disp("G")
disp(G);
disp("H")
disp(H);
