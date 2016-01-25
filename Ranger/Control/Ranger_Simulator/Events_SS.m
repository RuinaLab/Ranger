function [Value, IsTerminal, Direction] = Events_SS(~,X,P)

%Double stance is over when the reaction force on the rear foot goes to
%zero from above.

Heel = Compute_Height(X,P.robot);  %Compute the height of the heel above the ground

%The event in question is the bead passing the end of the rod:
Value = Heel; 
IsTerminal = true;  %Stop the integration when Value == 0
Direction = -1;  %Only consider the case when the foot is approaching the ground

end