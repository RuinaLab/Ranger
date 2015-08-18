function [m,c,k] = fitSecondOrderSys(q,u)
% [m,c,k] = fitSecondOrderSys(q,u)
%
% Given the coefficients of the sine curves for the input and the position
% of a system, this function computes the system parameters: m,c,k,q0
%
% m*ddq + c*dq + k*(q-q0) = u(t)
%
% q = struct with the position function
% u = struct with the input function
%

T = q.fit.period;
nData = 100;
t = linspace(0,T,nData)';

ddx = q.ddFun(t);
dx = q.dFun(t);
x = q.fun(t);
f = u.fun(t);

A = [ddx,dx,x,ones(size(x))];
B = f;
X = A\B;

m = X(1);
c = X(2);
k = X(3);

end