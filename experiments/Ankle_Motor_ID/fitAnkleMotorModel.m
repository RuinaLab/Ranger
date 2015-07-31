function [MotorModel, Info] = fitAnkleMotorModel(u,q,dq,ddq)
% [MotorModel, Info] = fitAnkleMotorModel(u,q,dq,ddq)
%
% This function uses non-linear least squares to fit a motor model to the
% processed data.
%
% INPUTS:
%   u = vector of input currents
%   q = joint angles
%   dq = joint rates
%   ddq = joint accelerations
%
% OUTPUTS:
%   MotorModel = best-fit for motor model paramters
%   Info = details about the quality of the model fitting.
%

%%%% Guess paramters, based mostly on Pranav's motor bench tests:
%%% Pretty good estimates of the following values:
guess.K = 0.018;  % motor constant 
guess.qSpring = 1.662;  % Mechanical parallel spring in joint - zero force angle
guess.kSpring = 0.612;  % Spring constant in mechanical spring
%%% Unsure estimate for the following values:
guess.c1 = 0.0;  % linear damping term
guess.c0 = 0.01;  % static friction term
guess.mu = 0.1;  % torque-dependent friction term
%%% Rough guesses, based on intuition and simple experiments:
guess.jointInertia = 0.05; % Effective inertia of the combined foot, transission, and rotor inertia
guess.smoothing = 0.02;  %Smoothing parameter for computing the smoothed sign() function for angular rate

%%%% Rough lower bounds, where applicable:
upp.K = 0.01;   % Bench testing can't be that far off...
upp.qSpring = 2.5;  %Based on joint limits
upp.kSpring = 2.0;  %Roughly based on motor torque and joint limits
upp.c0 = 1.0;
upp.c1 = 1.0;
upp.mu = 1.0;
upp.jointInertia = 0.25;   %The whole leg of the robot is 0.45
upp.smoothing = 0.25;

%%%% Rough upper bounds, where applicable:
upp.K = 0.04;   % Bench testing can't be that far off...
upp.qSpring = 0.0;  %Based on joint limits
upp.kSpring = 0.5;  %Roughly based on motor torque and joint limits
upp.c0 = 0.0;
upp.c1 = 0.0;
upp.mu = 0.0;
upp.jointInertia = 0.005;   %The whole leg of the robot is 0.45
upp.smoothing = 0.001;

%%%% Pack up into a decision variable vector:
zGuess = packDecVars(guess);
zLow = zeros(size(zGuess));
zUpp = packDecVars(upp);

%%%% Call fmincon
problem.objective = @(z)( objectiveFunction(z,u,q,dq,ddq) );
problem.x0 = zGuess;
problem.Aineq = [];
problem.bineq = [];
problem.Aeq = [];
problem.beq = [];
problem.lb = zLow;
problem.ub = zUpp;
problem.nonlcon = [];
problem.options = optimset('Display', 'iter');
problem.solver = 'fmincon';
[zSoln, objVal, exitFlag, output] = fmincon(problem);
[K, c1, c0, mu, qSpring, kSpring, jointInertia, smoothing] = unPackDecVars(zSoln);

%%%% Some error statistics:
ddqModel = predictAccel(u,q,dq, K, c1, c0, mu, qSpring, kSpring, jointInertia, smoothing);
Info = makeStruct(objVal, exitFlag, output, ddqModel);

%%%% Return useful output:
G = 34;  %Gearbox ratio
MotorModel = makeStruct(K, G, c1, c0, mu, qSpring, kSpring, jointInertia, smoothing);

end


function mse = objectiveFunction(z,u,q,dq,ddq)

[K, c1, c0, mu, qSpring, kSpring, jointInertia, smoothing] = unPackDecVars(z);

ddqBar = predictAccel(u,q,dq, K, c1, c0, mu, qSpring, kSpring, jointInertia, smoothing);

mse = mean((ddqBar-ddq).^2);

end


function [K, c1, c0, mu, qSpring, kSpring, jointInertia, smoothing] = unPackDecVars(z)

K = z(1);  % motor constant
c1 = z(2);  % linear damping term
c0 = z(3);  % static friction term
mu = z(4);  % torque-dependent friction term
qSpring = z(5);  % Mechanical parallel spring in joint - zero force angle
kSpring = z(6);  % Spring constant in mechanical spring
jointInertia = z(7); % Effective inertia of the combined foot, transission, and rotor inertia
smoothing = z(8);  %Smoothing parameter for computing the smoothed sign() function for angular rate

end

function z = packDecVars(Z)

z = [...
    Z.K;  % motor constant
    Z.c1;  % linear damping term
    Z.c0;  % static friction term
    Z.mu;  % torque-dependent friction term
    Z.qSpring;  % Mechanical parallel spring in joint - zero force angle
    Z.kSpring;  % Spring constant in mechanical spring
    Z.jointInertia; % Effective inertia of the combined foot, transission, and rotor inertia
    Z.smoothing];  %Smoothing parameter for computing the smoothed sign() function for angular rate

end

function ddqBar = predictAccel(current,angle,rate,...
    K, c1, c0, mu, qSpring, kSpring, jointInertia, smoothing)

%%%% Gearbox Ratio:
G = 34;

%%% Compute the net torque
direction = tanh(rate/smoothing);  %Motor direction
Tf = -c1*rate - c0*direction - mu*G*K*abs(current).*direction;   %Frictional terms
Te = G*K*current;    % Electrical terms
Ts = -kSpring*(angle-qSpring);     % Mechanical spring
T = Te + Tf + Ts;

%%%% Solve for acceleration:
ddqBar = T/jointInertia;

end
