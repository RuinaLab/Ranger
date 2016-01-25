function g = Smooth_Saturation(x,Bounds,alpha)

%FUNCTION:
%   This function produces a smooth saturation of an input x.
%
%INPUTS:
%   x = input vector of states to be saturated
%   Bounds = a (1x2) row vector of the saturation bounds
%   alpha = the smoothing parameter. small alpha corresponds to minimal
%       smoothing. Alpha is precisely the maximum x distance from the
%       bounds where the smoothing is present. If alpha is negative then it
%       will be set to zero smoothing. If alpha is too big, then it will be
%       capped based on the Bounds.
%OUTPUTS:
%   g = the smoothed and saturated version of x
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Ensure valid input
Bounds = sort(Bounds);   %Make sure that the bounds are in ascending order

if alpha < 0   %alpha cannot be negative
    alpha = 0;
elseif alpha > 0.5*diff(Bounds)   %Alpha cannot be too big!
    alpha = 0.5*diff(Bounds);
end


%Initialize the output
    g = zeros(size(x)); 

%Lower Parameters:
    x1 = Bounds(1)-alpha;   %Smoothing over this range
    x2 = Bounds(1)+alpha;    %Smoothing over this range

    y1 = Bounds(1);   %This point is on the saturated side
    y2 = Bounds(1)+alpha;    %This point is on the linear side

    dy1 = 0;   %Saturated = zero derivative
    dy2 = 1;  %y=x in the unsaturated region;

    
%Upper Parameters:
    x3 = Bounds(2)-alpha;    %Smoothing over this range
    x4 = Bounds(2)+alpha;    %Smoothing over this range

    y3 = Bounds(2)-alpha; %This point is on the linear side
    y4 = Bounds(2);      %This point is on the saturated side

    dy3 = 1;   %y=x in the unsaturated region
    dy4 = 0;  %Saturated = zero derivative


%Check bounds between points
    
    C1 = x<x1;          %Low Saturated
    C2 = x1<=x & x<x2;  %Low Smooth
    C3 = x2<=x & x<x3;  %y=x , no saturation
    C4 = x3<=x & x<x4;  %Upp smooth
    C5 = x4 <= x;       %Upp Saturated
    
%Check if there is anything that needs to be lower smoothed
if sum(C2) > 0
    X = [x1;x2];
    Y = [y1; y2];
    dY = [dy1;dy2];
    Coeff_Low = SplineCoeff(X,Y,dY);
    g(C2) = polyval(Coeff_Low,x(C2));
end

%Check if there is anything that needs to be upper smoothed
if sum(C4) > 0   
    X = [x3;x4];
    Y = [y3; y4];
    dY = [dy3;dy4];
    Coeff_Upp = SplineCoeff(X,Y,dY);
    g(C4) = polyval(Coeff_Upp,x(C4));
end   

%Old Fashioned saturation function
    g(C1) = Bounds(1);
    g(C3) = x(C3);
    g(C5) = Bounds(2);
    
end


function Coeff = SplineCoeff(X,Y,dY)

%FUNCTION:
%   
%
%INPUTS:
%
%   X = (2x1) column vector of the two points of interest
%   Y = (2x1) column vector of the function value at the two points of
%       interest
%   dY = (2x1) column vector of the slope of the function at the two points
%       of interest
%
%OUTUTS:
%   Coeff = a (4x1) column vector of coefficients such that:
%       y(x) = polyval(Coeff',x)
%

Const = [Y;dY];
x1 = X(1);
x2 = X(2);

M = [...
        x1^3    x1^2    x1      1;
        x2^3    x2^2    x2      1;
        3*x1^2  2*x1    1       0;
        3*x2^2  2*x2    1       0];
    
Coeff = M\Const;   %Solve the linear system

end