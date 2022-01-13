clear
clc

syms a b c p1 p2 p3

p=[p1;p2;p3];


Rx=[ 1      0       0;
     0 cos(a) -sin(a);
     0 sin(a)  cos(a)];
 
Ry= [cos(b)  0  sin(b);
     0       1       0;
     -sin(b) 0  cos(b)];

Rz= [ cos(c) -sin(c) 0;
      sin(c)  cos(c) 0;
      0       0      1];

R = Rx*Ry*Rz;
A=R*p;
A=jacobian(A,[a,b,c]);
A=subs(A,[a,b,c],[0,0,0])
  


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