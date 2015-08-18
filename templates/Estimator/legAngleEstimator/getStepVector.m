function [stepLength, legAngle] = getStepVector(x,y,slope)

stepLength = sqrt(x*x + y*y);

angleRaw = atan(y/x);

end