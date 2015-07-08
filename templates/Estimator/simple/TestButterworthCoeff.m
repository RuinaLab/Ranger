% This script runs a check on my algorithm for computing the 
% coefficients of the butterworth filter.

fc = 0.9;  %Normalized cut-off frequency

[B,A] = butter(2,fc);

q = sqrt(2);
c = tan(0.5*pi*(1-fc));

b0 = 1/(1 + q*c + c*c);
b1 = 2*b0;
b2 = b0;
b = [b0,b1,b2];

a0 = 1;
a1 = -2*(c*c-1.0)*b0;
a2 = (1-q*c+c*c)*b0;
a = [a0,a1,a2];

%%%% The following should be true:
%A == a
%B == b
%
%
%
% y0*a0 + y1*a1 + y2*a2 == z0*b0 + z1*b1 + z2*b2
%
% y0 = (z0*b0 + z1*b1 + z2*b2 - y1*a1 - y2*a2)/a0;
%