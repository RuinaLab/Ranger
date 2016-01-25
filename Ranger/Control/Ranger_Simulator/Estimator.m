function Xhat = Estimator(t,Y,P)

%FUNCTION:
%   The estimator reads in the current time, state estimate, and parameters
%   and uses that to create a state estimate Xhat.
%
%INPUTS:
%   t = [1 x N] vector of times
%   Y = Measurement - See Sensor_Model for definitions
%   P = The parameter struct generated in Set_Parameters.m
%
%OUTPUTS:
%   Xhat = State estimate. [17 x N] see format below:
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  From README.txt:
%
%     X(1)  = q1 = Stance Foot Abs. Angle             
%     X(2)  = u1 = Stance Foot Abs. Rate 
%     X(3)  = q2 = Stance Foot Joint Angle
%     X(4)  = u2 = Stance Foot Joint Rate
%     X(5)  = q6 = Stance Foot Motor Angle
%     X(6)  = u6 = Stance Foot Motor Rate
%     X(7)  = q3 = Hip Angle
%     X(8)  = u3 = Hip Rate
%     X(9)  = q4 = Swing Foot Joint Angle
%     X(10) = u4 = Swing Foot Joint Rate
%     X(11) = q5 = Swing Foot Motor Angle
%     X(12) = u5 = Swing Foot Motor Rate
%     X(13) = q1 = Integral of System Energy
%     X(14) = x_h = Hip X Coordinate       
%     X(15) = dx_h = Hip X Velocity
%     X(16) = y_h = Hip Y Coordinate
%     X(17) = dy_h = Hip Y Velocity
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOVELY 

persistent Xhat_Old;    %Store the previous state estimate;

switch P.Model.Noise_Model
    case 0  %Use perfect state feedback
        Xhat = Y;
    otherwise
        
        %
        %
        %  CODE HERE
        %
        %
end


end