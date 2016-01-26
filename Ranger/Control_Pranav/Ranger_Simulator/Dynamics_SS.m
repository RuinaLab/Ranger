function dX = Dynamics_SS(t,X,P)

%FUNCTION:
%   This function takes the current time (t), current state (X), and
%   parameter struct (P) and returns the time derivative of the state
%INPUTS: 
%   t = [1 x N] vector of times
%   X = [17 x N] state as defined in README.txt 
%   P = The parameter struct generated in Set_Parameters.m
%
%OUTPUTS:
%   dX = [17 x N] state derivative

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Figure out what the motors should do

%First take the truth state and 'measure' it to find the raw sensor values
Y = Sensor_Model(t,X,P);

%Estimate the state based on the measurements
Xhat = Estimator(t,Y,P);

%Figure out the command current
U_Cmd = Controller(t,Xhat,P);

%Plug that command current into the motor model
U = Motor_Model(U_Cmd,P);


%% Convert things to Pranav's 'convention'

% Actuator Conversion
% First, convert the actuators into a form that is readable by the MEX file
% Do this by creating a dummy time of 0 and then the simplest linear
% equation that cna be solved at this time (current = 1*actuator + 0*0)

%    h0 = coefs[0]; h1 = coefs[1];
%    a0 = coefs[2]; a1 = coefs[3];
%   Ih = h0 + h1*t;
%   Ia = a0 + a1*t;

hip = U(1,:);   %Extract the hip current
ankle = U(2,:);  %extract the ankle current
%Ignore the third actuator command... stupid hack by Pranav

%Pack up things into a way that can go to Pranav's stuff
Parameters = Extract_Parameters(P.robot);

dX = zeros(size(X));   %initialize the output of the function
for i=1:length(hip)

    time = 0;
    Coeff_Hip = [hip(i),0];
    Coeff_Ankle = [ankle(i) 0];
    %CAN'T DEAL WITH THE SWING FOOT ACTUATOR COMMANDS IN SINGLE STANCE
    %  --  HACK BY PRANAV TO MAKE THINGS SIMPLE (OR NOT)
    
    Params = [Parameters, Coeff_Hip, Coeff_Ankle];

    % MEX Function
    % Use the true state in the dynamics function
    dX_tmp = singleMEX(time,X(:,i),Params);   %Call Pranav's MEx File
    dX(:,i) = dX_tmp;   %Store the current timestep;
    
end


end