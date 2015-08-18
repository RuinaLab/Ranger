function [C,K,X0] = fitSystem(ddX,dX,X,U,M)

% Fits a second order system, assuming that mass is known.
%
% % M*ddX + C*dX + K*(X-X0) = U;

ONE = ones(size(X));

A = [dX', X', ONE'];
b = U' - ddX'*M;
z = A\b;

C = z(1);
K = z(2);
X0 = -z(3)/K;

end