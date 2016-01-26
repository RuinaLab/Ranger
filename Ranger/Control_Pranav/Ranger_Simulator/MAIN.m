%MAIN.m
clc; clear;


%%%%%%%%%%%%%
%
% Make sure that ODe45 doesn't choke on noise model... use fixed step?
%
%%%%%%%%%%%%%



% % %These lines compile the dynamics functions
% % mex doubleMEX.c;
% % mex singleMEX.c;
% % mex heelstrikeMEX.c

P = Set_Parameters();

Results = Run_Simulation(P);

%Plotting and animation
PLOT_Results(Results);
Animate_Ranger(Results);

%Save the Results:
Save_Results(Results);  

