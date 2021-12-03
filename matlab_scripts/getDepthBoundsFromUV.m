clc
clear

syms u2 d1 u1 v1 A B C D E F G H real  %u21 d11 u1 v1 known

% A*u1*d1 + B*v1*d1 + C*d1 + D
% ----------------------------
% E*u1*d1 + F*v1*d1 + G*d1 + H

expr = u2 == ((A*u1*d1 + B*v1*d1 + C*d1 + D)/(E*u1*d1 + F*v1*d1 + G*d1 + H));
slv = solve(expr,d1);
slv =simplify(slv,'Steps',1000);
slv = collect(slv,[u2,u1,v1])

[N,D] = numden(slv);
CN = coeffs(N,u2);
CD = coeffs(D,u2);

% A*u2 + B
% ----------------------------
% C*u2 + D

A=CN(2);
B=CN(1);
C=CD(2);
D=CD(1);

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