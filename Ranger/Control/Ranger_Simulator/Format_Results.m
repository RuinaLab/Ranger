function PlotResults = Format_Results(Results)



%Format things in a way that is easy to plot
    BigState = [OUT_Animate.X_DS, OUT_Animate.X_SS, OUT_Animate.X_HS];
    K.Time =   [OUT_Animate.T_DS, OUT_Animate.T_SS, OUT_Animate.T_HS];

    K.SwitchTime = Results.AnimationStruct.T_SS(1);
    K.SwitchIdx = length(Results.AnimationStruct.T_DS+1);
    
    %Joints = BigState([1,3,5,7,9,11],:);
    K.StanceFootAbsAngle = BigState(1,:); % Stance Foot Absolute Angle
    K.StanceFootJointAngle = BigState(3,:); % Stance Foot Joint Angle
    K.StanceFootMotorAngle = BigState(5,:); % Stance Foot Motor Angle
    K.HipAngle = BigState(7,:); % Hip Angle
    K.SwingFootJointAngle = BigState(9,:); % Swing Foot Joint Angle
    K.SwingFootMotorAngle = BigState(11,:); % Swing Foot Motor Angle

    %Joint_Rates = BigState([2,4,6,8,10,12],:);
    K.StanceFootAbsRate = BigState(2,:); % Stance Foot Absolute Rate
    K.StanceFootJointRate = BigState(4,:); % Stance Foot Joint Rate
    K.StanceFootMotorRate = BigState(6,:); % Stance Foot Motor Rate
    K.HipRate = BigState(8,:); % Hip Rate
    K.SwingFootJointRate = BigState(10,:); % Swing Foot Joint Rate
    K.SwingFootMotorRate = BigState(12,:); % Swing Foot Motor Rate

    %Hip_Position = BigState([14,16],:);
    K.HipPosX = BigState(14,:); % Hip x position
    K.HipPosY = BigState(16,:); % Hig y position

    %Hip_Velocity = BigState([15,17],:);
    K.HipVelX = BigState(15,:); % Hip x velocity
    K.HipVelY = BigState(17,:); % Hig y velocity

    K.Energy = BigState(13,:);  %Total Energy
    
    
%Now compute the information about the actuators:

        K.HipMotorCurrent = [OUT_Animate.U_DS(1,:), OUT_Animate.U_SS(1,:), OUT_Animate.U_HS(1)];
        K.AnkleCurrent1 = [OUT_Animate.U_DS(2,:), OUT_Animate.U_SS(2,:), OUT_Animate.U_HS(2)];
        K.AnkleCurrent2 = [OUT_Animate.U_DS(2,:),...
            zeros(size(OUT_Animate.U_SS(2,:))),...  %HACK - assume not current to the ankle motor during swing 
            zeros(size(OUT_Animate.U_HS(2)))];      %HACK - assume not current to the ankle motor during swing 
        
    %Now plot the reaction forces:
        K.StanceFootVertRxn = [OUT_Animate.Rxn_DS(1,:), OUT_Animate.Rxn_SS(1,:), OUT_Animate.Rxn_HS(1)];
        K.SwingFootVertRxn = [OUT_Animate.Rxn_DS(2,:),...
            zeros(size(OUT_Animate.Rxn_SS(1,:))),... 
            zeros(size(OUT_Animate.Rxn_HS(1)))];
 
    %Collect everything together into a nice struct
        Results.Plot = K;        
        

end