function [Value, IsTerminal, Direction] = Events_DS(t,X,P)
%
%
%
%DUMB WAY TO DO THIS BECAUSE IT RE-COMPUTES ENTIRE DYNAMICS FUNCTION!
%
%
%
%Double stance is over when the reaction force on the rear foot goes to
%zero from above.

[~, Rxn] = Dynamics_DS(t,X,P);  %Compute the reaction force on the rear leg

%The event in question is the bead passing the end of the rod:
Value = Rxn(2);   %Check the reaction what will be the swing leg
IsTerminal = true;  %Stop the integration when Value == 0
Direction = -1;  %Only consider the case when the foot is leaving the ground

end