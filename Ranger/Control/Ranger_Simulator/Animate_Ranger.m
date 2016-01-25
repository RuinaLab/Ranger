function Animate_Ranger(Results,N)

%FUNCTION:
%   This function takes in the results of a simulation and a parameter
%   struct and uses this to run a simulation.
%
%INPUTS:    
%   (Nss = number of integration steps in single stance, Nds = double stance)
%   OUT_Animate is a struct with 5 fields: X_SS: [17xNss double], T_SS:
%       [1xNss double], X_HS: [17x1 double], X_DS: [17xNds double], 
%       T_DS: [1xNds double]. 17 is the number of states used by Pranav's
%       dynamics. See README.txt for more details.
%   P = a struct of input parameters. See Set_Parameters.m for more
%       details.

%N is the figure to start plotting on
if nargin == 1
    N = 1;
end


figure(N); 

P = Results.Parameters;

%Now break out the variables by name:
q1 = Results.Plot.StanceFootAbsAngle';   %Stance foot absolute angle
q2 = Results.Plot.StanceFootJointAngle';   %Stance foot joint angle
q3 = Results.Plot.HipAngle';   %Hip angle
q4 = Results.Plot.SwingFootJointAngle';   %Swing Foot joint angle
Hip_X = Results.Plot.HipPosX';   % X position of the hip
Hip_X = Hip_X-min(Hip_X);   %Ensure that the hip starts at zero
Hip_Y = Results.Plot.HipPosY';   % Y position of the hip

%Define robot parameters:
l = P.robot.parm.l;    %Leg length

%Define the extents of the axis to use for plotting:
N_Steps = P.plot.N_Walking_Steps;
if P.plot.camera_tracking == true   %Should the camera be fixed to the world or the robot's hip?
    Extents = [-l/2, l/2,0,1.25*l];   %Camera is attached to the robot
else
    Extents = [min(Hip_X)-l/2,max(Hip_X)*N_Steps+l/2,0,1.25*l];   %Camera zooms out to see whole walk
end

T_var_All = Results.Plot.Time;


%% %%%%%%%%%%%%%%%%%%%%%%    IN A LOOP    %%%%%%%%%%%%%%%%%%%%%%%%%

%Misc. Initializations for the loop
tic;    %Start a timer
Loop_Time = 0;    %store how long has the simulation been running
i=1;   %Which data index to plot
Max_i = length(T_var_All);
Reset_Count = 0;   %Keep track of how many steps have been taken
Hip_Offset = 0;   %How far to offset the hip after each step
T_end = T_var_All(end);   %Ending time of one step

while Loop_Time < T_end*N_Steps;  %Loop while the CPU time is less than the end of the simulation's time
    %The next few lines pull the time step that is closest to the real time
    Loop_Time = toc/P.plot.slow_motion_factor;   %Get the current time  (Taking slow motion into accunt if desired)
    while (i<Max_i) && (Loop_Time > (T_var_All(i)+Reset_Count*T_end)) %While we are not at the end of the data and the CPU time is ahead of the simulation time
        i=i+1;   %Go to the next time frame
        if (i>=Max_i)  &&  (Reset_Count < N_Steps)  %Have we reached the end of the data, and desire to play another step?
            %Then we should take another step
            Hip_Offset = Hip_Offset + Hip_X(end);   %Shift all of the data
            i=1;      %Go back to the beginning index
            Reset_Count = Reset_Count + 1;
        end
    end
    
%Positions of useful points   (equations from Pranav)

if P.plot.camera_tracking == true
    Hip = [0;Hip_Y(i)];   %Current position of the hip
else
    Hip = [Hip_X(i)+Hip_Offset;Hip_Y(i)];   %Current position of the hip
end

Ank_1 = Hip + l*Rot(q1(i)+q2(i)-pi)*[0; -1];     %Define the stance ankle position
Ank_2 = Hip + l*Rot(q1(i)+q2(i)-q3(i)-pi)*[0; -1];    %Define the swing ankle position

%Points that outline the foot
[F1_X, F1_Y] = Foot_Points(P,q1(i),Ank_1);   %Stance foot
[F2_X, F2_Y] = Foot_Points(P,q1(i)+q2(i)-q3(i)-q4(i),Ank_2);   %Swing foot

%Get the leg Points:
Leg_1 = [Hip, Ank_1];
Leg_2 = [Hip, Ank_2];
L1_X = Leg_1(1,:);
L1_Y = Leg_1(2,:);
L2_X = Leg_2(1,:);
L2_Y = Leg_2(2,:);

%Plot various things:
clf; hold on;
plot(F1_X,F1_Y,'k-','LineWidth',2);    %Plot the stance foot
plot(L1_X,L1_Y,'k-','LineWidth',2);    %Plot the swing foot
plot(F2_X,F2_Y,'k-','LineWidth',2);    %Plot the stance leg
plot(L2_X,L2_Y,'k-','LineWidth',2);    %Plot the swing leg
plot([Extents(1),Extents(2)],[0,0],'k-','LineWidth',2);   %Plot the ground
axis(Extents); axis equal, axis manual; axis off;
pause(0.01)
    
end  %i, main plotting loop
    
end

function [x, y] = Foot_Points(P,Foot_Angle,Pos)

%Written by MPK

%Foot_Angle is the current angle of the foot
%Pos is the current position of the ankle joint on the foot

N = 16;   %Number of points to define the foot
R = P.robot.parm.r;   %Radius of the foot
d = P.robot.parm.d;   %distance from ankle joint to center of the foot
Arc_Angle= 0.54;    %Angle swept from heel to the toe
Angle_Offset = pi/2;    %Because of angle definitions

t = linspace(0,Arc_Angle,N-2) + Angle_Offset + Foot_Angle;

%Add point at the hinge of the foot, make hinge at origin
x = [0, R*cos(t)-d*cos(t(1)), 0] + Pos(1);
y = [0, R*sin(t)-d*sin(t(1)), 0] + Pos(2);

end


function rotation = Rot(A)

%Written by Pranav
%===================================================================
rotation = [cos(A) -sin(A); sin(A) cos(A)];

end