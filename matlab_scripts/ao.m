clear
clc

syms da db dc p1 p2 p3

p=[p1;p2;p3];


Rx=[ 1      0       0;
     0 cos(da) -sin(da);
     0 sin(da)  cos(da)];
 
Ry= [cos(db)  0  sin(db);
     0       1       0;
     -sin(db) 0  cos(db)];

Rz= [ cos(dc) -sin(dc) 0;
      sin(dc)  cos(dc) 0;
      0       0      1];

R = Rx*Ry*Rz
% A=R*p;
% A=jacobian(A,[a,b,c]);
% A=subs(A,[a,b,c],[0,0,0])
  


%%
clc
clear

syms s x y z p1 p2 p3

p=[p1;p2;p3];

R=[ 1-2*(y^2)-2*(z^2), 2*x*y-2*s*z, 2*x*z+2*s*y ;
        2*x*y+2*s*z, 1-2*(x^2)-2*(z^2), 2*(y*z)-2*(s*x);
        2*x*z-2*s*y, 2*y*z+2*s*x, 1-2*(x^2)-2*(y^2) ];
    
A=R*p;
A=jacobian(A,[x,y,z]);
A=subs(A,[x,y,z],[0,0,0])