clear
clc

% JM is jacobian of the warping function wrt to cam_m pose parameters

syms u_A v_A real % uv seen in frame A (knownk)
syms d_A real % depth belief in frame A (known)
syms f w h real % intrindic cam parameters (known)

% B_T_W
syms r00 r01 r02 r10 r11 r12 r20 r21 r22
syms t0 t1 t2
B_T_W=  [r00 r01 r02 t0;
         r10 r11 r12 t1;
         r20 r12 r22 t2;
         0 0 0 1];
 
% points on which tangent space is fixed
%  camera wrt to world
syms r00_0 r01_0 r02_0 r10_0 r11_0 r12_0 r20_0 r21_0 r22_0
syms t0_0 t1_0 t2_0
X0= [r00_0 r01_0 r02_0 t0_0;
     r10_0 r11_0 r12_0 t1_0;
     r20_0 r12_0 r22_0 t2_0;
     0 0 0 1];
 
 
syms x1 x2 x3 x4 x5 x6 real % current guess (known)

syms dx1 dx2 dx3 dx4 dx5 dx6 real % perturbation (to be estimated)

%%

K=[f  0 w/2 ;
   0  f h/2 ;
   0  0 1   ];

Kinv = inv(K);


p_A = Kinv * [ u_A*d_A; v_A*d_A; d_A ]; % 3d point expressed in frame A
p_A_hom = [p_A;1];
p_A_hom = simplify(p_A_hom,'Steps',1000);

x = [x1, x2, x3, x4, x5, x6];
X=v2T(x);  % current guess

dt= [dx1, dx2, dx3, dx4, dx5, dx6];
DX=v2T(dt);  % perturbation

% jacobian are evaluated on x=0 (first estimate jacobians)
%p_B_hom=B_T_W*DX*X*X0*p_A_hom; NO
p_B_hom=B_T_W*DX*X0*p_A_hom;
p_B_hom = simplify(p_B_hom,'Steps',1000);

p_B=p_B_hom(1:3);

uv_B_ = K*p_B;
uv_B_ = simplify(uv_B_,'Steps',1000)

uv_B=uv_B_/uv_B_(3);
uv_B=uv_B(1:2);

uv_B=simplify(uv_B,'Steps',1000)

J = jacobian(uv_B,[dx1,dx2,dx3,dx4,dx5,dx6]);
JM = subs(J,[dx1,dx2,dx3,dx4,dx5,dx6],[0,0,0,0,0,0]);
JM = simplify(JM,'Steps',1000)
save("JM.mat","JM")

%%
clc
clear



function T = v2T(v)
    t1_0=v(1);
    t2_0=v(2);
    t3=v(3);
    a1=v(4);
    a2=v(5);
    a3=v(6);
    c1=cos(a1);
    c2=cos(a2);
    c3=cos(a3);
    s1=sin(a1);
    s2=sin(a2);
    s3=sin(a3);
    
    R=[ c1*c3+s1*s2*s3, c3*s1*s2-c1*s3, c2*s1;
        c2*s3,  c2*c3, -s2;
        c1*s2*s3-c3*s1, c1*c3*s2+s1*s3, c1*c2];
    t=[t1_0;t2_0;t3];
    T=[R,t;0,0,0,1];
    
end
