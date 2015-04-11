%% clear all
clear all
clc
%% variables
s = tf('s');
ts = 2;  %4.6/sig
Mp = .05; % -pi*zeta/sqrt(1-zeta^2)
alpha = 2; % change the third  pole 

sig = 4.6/ts;
zeta = -log(Mp)/sqrt(pi^2+(log(Mp))^2);
wn = sig/zeta;

% lambda = sym('lambda');
% eqn = ((lambda + alpha*zeta*wn)*(lambda^2 + 2*zeta*wn*lambda + wn^2));

sys = 1/((s + alpha*zeta*wn)*(s^2 + 2*zeta*wn*s + wn^2));

%% response
step(sys);
display('Damping Ratios and Natural Frequency');
damp(sys)

display('Step Info');
stepinfo(sys)

%%
% %% get co-efficients and compare
% 
% C1 = coeffs(eqn,lambda);
% 
% s = sym('s');
% x = sym('x');
% y = sym('y');
% z = sym('z');
% ur1 = sym('ur1');
% ur2 = sym('ur2');
% characEqn = s^3+(x+z)*s^2+(x*z+y*ur1+ur2^2)*s+(x*y*ur1+z*ur2^2);
% 
% C2 = coeffs(characEqn,s);
% 
% 
% eqn1 = C2(1)-C1(1);
% eqn2 = C2(2)-C1(2);
% eqn3 = C2(3)-C1(3);
% 
% 
% [solx,soly,solz] = solve(eqn1 == 0, eqn2 == 0,eqn3 == 0)
% 
