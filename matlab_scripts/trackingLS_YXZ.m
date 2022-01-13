clear
clc

% using euler angles

syms u_A v_A real % uv seen in frame A (knownk)
syms d_A real % depth belief in frame A (known)
syms f w h real % intrindic cam parameters (known)
syms dt1 dt2 dt3 da1 da2 da3 real % perturbation (to be estimated)
syms t1 t2 t3 a1 a2 a3 real % current guess (known)
syms grad_u grad_v %intensity gradients of the image (known)

K=[f  0 w/2 ;
   0  f h/2 ;
   0  0 1   ];

Kinv = inv(K);


p_A = Kinv * [ u_A*d_A; v_A*d_A; d_A ]; % 3d point expressed in frame A
p_A_hom = [p_A;1];
p_A_hom = simplify(p_A_hom,'Steps',1000);

v_guess = [t1, t2, t3, a1, a2, a3];
B_T_A=v2T(v_guess);  % current guess

v_d= [dt1, dt2, dt3, da1, da2, da3];
DX=v2T(v_d);  % perturbation

p_B_hom=DX*B_T_A*p_A_hom;
p_B_hom = simplify(p_B_hom,'Steps',1000);

p_B=p_B_hom(1:3);

uv_B_ = K*p_B;
uv_B_ = simplify(uv_B_,'Steps',1000)

uv_B=uv_B_/uv_B_(3);
uv_B=uv_B(1:2);

uv_B=simplify(uv_B,'Steps',1000)

J = jacobian(uv_B,[dt1,dt2,dt3,da1,da2,da3]);
J_YXZ = subs(J,[dt1,dt2,dt3,da1,da2,da3],[0,0,0,0,0,0]);
J_YXZ = simplify(J_YXZ,'Steps',1000)

%%
clc
clear



function T = v2T(v)
    t1=v(1);
    t2=v(2);
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
    t=[t1;t2;t3];
    T=[R,t;0,0,0,1];
    
end
