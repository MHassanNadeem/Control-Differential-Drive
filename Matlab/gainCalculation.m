clc
syms x y z zeta omega v1 v2 s alpha

alpha = 2;

[solx,soly,solz]=solve(x+z==(2+alpha)*zeta*omega,x*z+y*v1+v2^2==(alpha*2)*zeta^2*omega^2+omega^2,x*y*v1+z*v2^2==(alpha)*zeta*omega^3);
display('Kx =');
solx(1)
display('Ky =');
soly(1)
display('Kth =');
solz(1)
% 
% DesPoly = ((s + alpha*zeta*wn)*(s^2 + 2*zeta*wn*s + wn^2));
% DesCoeff = coeffs(DesPoly,s);
% 
% 
% characEqn = s^3+(x+z)*s^2+(x*z+y*v1+v2^2)*s+(x*y*v1+z*v2^2);
% CharCoeff = coeffs(characEqn,s);
% 
% 
% [solx,soly,solz] = solve(CharCoeff(3)-DesCoeff(3)==0, CharCoeff(2)-DesCoeff(2)==0,CharCoeff(1)==DesCoeff(1))