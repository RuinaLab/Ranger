function Y = Initial_State(P)

%FUNCTION:
%   This function takes the parameter struct and generates the initial
%   state of the robot for the simulation. This function should be called
%   from Set_Parameters.m
%
%INPUTS: 
%   P = The parameter struct generated in Set_Parameters.m
%
%OUTPUTS:
%   Y = The initial state of the robot - format is the same as in Pranav's
%   Code:
%   
%     Y(1)  = q1 = Stance Foot Abs. Angle             
%     Y(2)  = u1 = Stance Foot Abs. Rate 
%     Y(3)  = q2 = Stance Foot Joint Angle
%     Y(4)  = u2 = Stance Foot Joint Rate
%     Y(5)  = q6 = Stance Foot Motor Angle
%     Y(6)  = u6 = Stance Foot Motor Rate
%     Y(7)  = q3 = Hip Angle
%     Y(8)  = u3 = Hip Rate
%     Y(9)  = q4 = Swing Foot Joint Angle
%     Y(10) = u4 = Swing Foot Joint Rate
%     Y(11) = q5 = Swing Foot Motor Angle
%     Y(12) = u5 = Swing Foot Motor Rate
%     Y(13) = q1 = Integral of System Energy
%     Y(14) = x_h = Hip X Coordinate       
%     Y(15) = dx_h = Hip X Velocity
%     Y(16) = y_h = Hip Y Coordinate
%     Y(17) = dy_h = Hip Y Velocity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% Dummy state (DS) taken from Pranav's solution
Y = [   3.1416;
       -12.2162;
        0.2462;
        12.3709;
        0.2462;
        0;
        0.4924;
        0.3094;
       -0.2462;
        8.8608;
       -0.0672;
        8.3202;
        0;
        0;
        0.9554;
        1.0210;
       -0.0362];
   
end