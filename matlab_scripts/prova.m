clear
clc

syms t0 t1 t2

a =1.230034589767456
b= -2.4980255375339766e-07
c=-1.7942723035812378

R= [                         cos(b)*cos(c),                       -cos(b)*sin(c),         sin(b);
      cos(a)*sin(c) + cos(c)*sin(a)*sin(b), cos(a)*cos(c) - sin(a)*sin(b)*sin(c), -cos(b)*sin(a);
      sin(a)*sin(c) - cos(a)*cos(c)*sin(b), cos(c)*sin(a) + cos(a)*sin(b)*sin(c),  cos(a)*cos(b)];
 
a =1.230034589767456
b= -2.4980255375339766e-07
c=-1.7942723035812378
R
% pretty(simplify(subs(R,[a,b,c],[1.230034589767456, -2.4980255375339766e-07, -1.7942723035812378]))

% invR=inv(R);
% invR=simplify(invR,'Steps',1000)
% 
% t=[t0;t1;t2];
% invt = -invR*t;
% invt=simplify(invt,'Steps',1000)
% 
% T=[invR , invt; 0,0,0,1];
% 
% syms p0 p1 p2
% 
% p=[p0;p1;p2;1];
% 
% f=T*p;
% 
% J=jacobian(f,[t0 t1 t2 a b c]);
% J=subs(J,[t0 t1 t2 a b c],[0 0 0 0 0 0]);
% J=simplify(J,'Steps',1000)