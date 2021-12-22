clear
clc

syms u2 v2 u1 v1 d1 d2 SlopeDiff real 
syms t2 r22 r12 r20 w h f
syms Asr Bsr Csr Dsr Esr Fsr real %get slope2 from u1 v1
syms Asm Bsm Csm Dsm Esm Fsm real %get slope1 from u2 v2
syms Aum Bum Cum Dum Eum Fum Gum Hum real %get v2 from u1 v1
syms Avm Bvm Cvm Dvm Evm Fvm Gvm Hvm real %get u2 from u1 v1
syms Aur Bur Cur Dur Eur Fur Gur Hur real %get v1 from u2 v2
syms Avr Bvr Cvr Dvr Evr Fvr Gvr Hvr real %get u1 from u2 v2

eq1 = SlopeDiff == ((Asr*u1+Bsr*v1+Csr)/(Dsr*u1+Esr*v1+Fsr))-((Asm*u2+Bsm*v2+Csm)/(Dsm*u2+Esm*v2+Fsm));
eq2 = u2 == ((Aum*u1+Bum*v1+Cum+Dum)/(Eum*u1+Fum*v1+Cum+Dum));
eq3 = v2 == ((Avm*u1+Bvm*v1+Cvm+Dvm)/(Evm*u1+Fvm*v1+Cvm+Dvm));
eq4 = u1 == ((Aur*u2*d2+Bur*v2*d2+Cur*d2+Dur)/(Eur*u2*d2+Fur*v2*d2+Cur*d2+Dur));
eq5 = v1 == ((Avr*u2*d2+Bvr*v2*d2+Cvr*d2+Dvr)/(Evr*u2*d2+Fvr*v2*d2+Cvr*d2+Dvr));
eq6 = t2 - r22 + r12*(h/(2*f) - v1/f) + r20*(u1/f - w/(2*f)) == -d2;

% eq2 = u2 == ((Aum*((Aur*u2+Bur*v2+Cur+Dur)/(Eur*u2+Fur*v2+Cur+Dur))+Bum*((Avr*u2+Bvr*v2+Cvr+Dvr)/(Evr*u2+Fvr*v2+Cvr+Dvr))+Cum+Dum)/(Eum*((Aur*u2+Bur*v2+Cur+Dur)/(Eur*u2+Fur*v2+Cur+Dur))+Fum*((Avr*u2+Bvr*v2+Cvr+Dvr)/(Evr*u2+Fvr*v2+Cvr+Dvr))+Cum+Dum));
% eq3 = v2 == ((Avm*((Aur*u2+Bur*v2+Cur+Dur)/(Eur*u2+Fur*v2+Cur+Dur))+Bvm*((Avr*u2+Bvr*v2+Cvr+Dvr)/(Evr*u2+Fvr*v2+Cvr+Dvr))+Cvm+Dvm)/(Evm*((Aur*u2+Bur*v2+Cur+Dur)/(Eur*u2+Fur*v2+Cur+Dur))+Fvm*((Avr*u2+Bvr*v2+Cvr+Dvr)/(Evr*u2+Fvr*v2+Cvr+Dvr))+Cvm+Dvm));

% slv = solve(eq1,SlopeDiff)
slv = solve([eq1,eq2,eq3,eq4,eq5,eq6],[SlopeDiff,u1,v1,u2,v2,d2])
% slv = solve([eq1,eq2,eq3],[SlopeDiff,u2,v2])
% slv = simplify(slv,'Steps',1000);
% pretty(slv[1])

