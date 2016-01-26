README.txt

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
C and C++ dynamics files -- Written by Pranav Bhounsule

doubleMEX.c, singleMEX.c, and heelstrikeMEX.c at all standard Matlab MEX 
functions. They are compiled by running the following lines of code:

mex doubleMEX.c
mex singleMEX.c
mex heelstrikeMEX.c

Once this has been completed, you can call them directly from Matlab. There are 
two other dynamics functions: doublestanceMEX.cpp and singlestanceMEX.cpp. These 
are designed to be used ONLY with the DOP853 integrator. This integration 
algorithm is a high-order variable-step method that runs FAST. You can access
the install files by unzipping the file DOP853_Integration_Algorithm.zip. This
should be installed on your local machine - don't extract the files to this folder.
NOTE - If you try to use the mex command to compile doublestanceMEX.cpp or
singlestanceMEX.cpp you will get an error. This is because they are automatically
compiled by DOP853.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Naming Conventions:

SS = Single Stance
DS = Double Stance
HS = Heel Strike

Actuator Step = A high level time step over which the actuator effort is 
    represented as a linear function of time.

ankle1 = The ankle joint that is on what was previously called the 'stance'
    leg. 'ankle' during SS is the same as 'ankle1' during DS.

ankle2 = The ankle joint that is on what was previously called the 'swing' 
    leg. ankle2 is not present in SS.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Low Level Dynamic State (passed to dop853 integration routines)
           
    Y(1)  = q1 = Stance Foot Abs. Angle             
    Y(2)  = u1 = Stance Foot Abs. Rate 
    Y(3)  = q2 = Stance Foot Joint Angle
    Y(4)  = u2 = Stance Foot Joint Rate
    Y(5)  = q6 = Stance Foot Motor Angle
    Y(6)  = u6 = Stance Foot Motor Rate
    Y(7)  = q3 = Hip Angle
    Y(8)  = u3 = Hip Rate
    Y(9)  = q4 = Swing Foot Joint Angle
    Y(10) = u4 = Swing Foot Joint Rate
    Y(11) = q5 = Swing Foot Motor Angle
    Y(12) = u5 = Swing Foot Motor Rate
    Y(13) = q1 = Integral of System Energy
    Y(14) = x_h = Hip X Coordinate       
    Y(15) = dx_h = Hip X Velocity
    Y(16) = y_h = Hip Y Coordinate
    Y(17) = dy_h = Hip Y Velocity

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
All Files should have the following header:

%FUNCTION:
%   Notes on how the function works go here
%
%INPUTS:
%   input_1 = details...
%   input_2 = ...
%   ...
%   input_N = blah blah...
%    
%OUTPUTS:
%   output_1 = details...
%   output_2 = ...
%   ...
%   output_N = blah blah...

Any inputs and outputs should be compatible with vectors. For example, if a
function acts on a Nx1 state, then it should also be able to handle a NxM 
matrix of successive states.


Atif is not Anoop!
