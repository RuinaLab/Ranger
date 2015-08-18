function [M,C,K,X0] = fitSecondOrderSystem(ddX,dX,X,U)
% [M,C,K,X0] = fitSecondOrderSystem(ddX,dX,X,U)
%
% Computes a least squares fit between the data:
%
% M*ddX + C*dX + K*(X-X0) = U;
%
%
% [ddX, dX, X, 1]*[M;C;K;-K*X0] = U
%
% A*z = b;
%
% z = A\b;



ONE = ones(size(X));

A = [ddX', dX', X', ONE'];
b = U';

Z = A\b;

M = Z(1);
C = Z(2);
K = Z(3);
X0 = -Z(4)/K;

end