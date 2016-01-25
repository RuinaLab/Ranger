function Results = Run_Simulation(P)

%FUNCTION:
%   This function takes the struct of input parameters and uses it to run a
%   simulation. A structure Results is returned contain the results of the
%   simulation
%
%   For now, the simulation code will run entirely using ode45, with the
%   low level dynamics computed via the mex interface
%
%INPUTS: 
%   P = The parameter struct generated in Set_Parameters.m
%
%OUTPUTS:
%   Results = A struct of results.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set parameters for the simulation


%Function Handles used by ode45
    DS_Dyn_Func = @(t,X)Dynamics_DS(t,X,P);
    SS_Dyn_Func = @(t,X)Dynamics_SS(t,X,P); 

%Options -- Event Handles and Tolerances
    ErrTol = P.Model.Integration_Tolerance;
    Options_DS = odeset('Events',@(t,x)Events_DS(t,x,P),...
                        'RelTol',ErrTol,...
                        'AbsTol',ErrTol); 
    Options_SS = odeset('Events',@(t,x)Events_SS(t,x,P),...
                        'RelTol',ErrTol,...
                        'AbsTol',ErrTol); 
                    
%Create the initial state and time;
Time = 0;
State = P.Initial_State;
Mode = [];  %Store the system's dynamical mode here (DS,SS,HS)

%% Run the simulation

Current_Time = Time(end);    %Checked by the while loop for exit conditions
End_Time = P.Sim.Simulation_Duration;
DS_Mode = strcmp(P.Sim.Initial_Mode,'DS');   %1=DS, 0=SS

while Current_Time < End_Time  %While simulation is incomplete
    
    Tspan = [Current_Time,End_Time];
    IC = State(:,end);
    
    if DS_Mode  %Then currently in double stance
        [Tout, Yout] = ode45(DS_Dyn_Func,Tspan,IC,Options_DS);  %Toe-off causes early exit
        ModeOut = 1*ones(size(Tout));  %DS = 1
    else  %Then currently in single stance
        [Tout, Yout] = ode45(SS_Dyn_Func,Tspan,IC,Options_SS);  %Heel-strike cause early exit
        ModeOut = 2*ones(size(Tout));   %SS = 2
        
        %Heel Strike!
            Yminus = Yout(end,:)';
            Yplus = heelstrikeMEX(Yminus,P.robot);  %Do the heelstrike calculation
            %Append the heel strike states to the end of the ode45 output
            Tout = [Tout;(Tout(end)+eps)]; %#ok<AGROW>
            Yout = [Yout;Yplus']; %#ok<AGROW>
            ModeOut = [ModeOut;3];  %#ok<AGROW> %HS = 3
    end
        
    %Inefficient memory allocation for now - fix later
    Time = [Time,Tout']; %#ok<AGROW>
    State = [State,Yout']; %#ok<AGROW>
    Mode = [Mode, ModeOut']; %#ok<AGROW>
    DS_Mode = ~ DS_Mode;  %Do the other thing on the next step
    Current_Time = Time(end);  %Update the current time 
    
end


%% Store things to return:

Results.Parameters = P;    %Here are the parameters that were used...
Results.Time = Time;    %Time vector for output
Results.State = State;   %State vector for output
Results.Mode = Mode;  %DS = 1, SS = 2, HS = 3   dynamic mode for state

end