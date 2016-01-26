function P = Set_Parameters()

%This function sets all of the parameters that are used in the entire
%project.

%% Robot parameters -- These get passed through to low level dynamics
    [Motor, Parm] = Set_Robot();
    P.robot.motor = Motor; %Store the robot motor parameters
    P.robot.parm = Parm;  %Store the robot physical parameters
    P.robot.motor.Sat_Smooth_Parameter = 0.1;  %How much to smooth the saturation curve
    
%% Estimator
    %P.Est.blahblah.....
    
    
%% Model Accuracy Parameters
    P.Model.Noise_Model = 0;   
    %   0 = use perfect state
    %   1 = use noise model 1...
    P.Model.Integration_Tolerance = 1e-3;

%% Controller parameters
    P.Ctl.Controller_Type = 0;
    %   0 = no control (passive robot)
    %   1 = magic!
    
%% Plotting, and Animation
    P.Sim.Simulation_Duration = 4;   %(s) How many seconds should the simulation run?
    P.Sim.Initial_Mode = 'SS'; %Mode: 'SS' or 'DS'
    
    
%% Computer Generated Parameters:
    P.Initial_State = Initial_State(P);
    
    

    


end