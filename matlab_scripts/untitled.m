A=randi([0,9],2,2)
B=randi([0,9],2,100);
C=randi([0,9],100,2);
D=eye(100);

S=A-B*D*C;
Sinv=inv(S);

D*C*S*B*D