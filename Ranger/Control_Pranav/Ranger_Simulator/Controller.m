function U = Controller(t,Xhat,P)

%FUNCTION:
%   The controller takes the current state estimate, time, and robot
%   parameters and uses that to compute the
%
%INPUTS:
%   t = [1 x N]  time vector
%   Xhat = [17 x N]  state estimate vector
%   P = The parameter struct generated in Set_Parameters.m
%
%OUTPUTS:
%   U = [3 x N] actuator command current
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


switch P.Ctl.Controller_Type
    case 0   %No control (Passive Dynamics)
        Nt = length(t);
        U = zeros(3,Nt);
    otherwise
        %
        %
        % CODE HERE
        %
        %
end


end