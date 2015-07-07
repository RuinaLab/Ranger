% This data was collected with the robot hanging from the center of the top
% of the outer hip frame. The steady-state was recorded when the free leg
% was hanging vertically, by external torques applied to the other leg.
%
% Current was computed using the T/(KG )motor model, neglecting friction
% terms
%
%


%%%% HIP JOINT %%%%

data = [ ...% torque, angle
  0.0, 0.023;
  0.5, 0.055;
  1.0, 0.118;
  1.5, 0.220;
  2.0, 0.260;
  2.5, 0.350;
  3.0, 0.380;
  3.5, 0.44;
  4.0, 0.47;
  4.5, 0.53];    % Over temp limit in 1.5 seconds

dataMirror = -data; % Assume symmetry

torque = [data(:,1);dataMirror(:,1)];
angle = [data(:,2);dataMirror(:,2)];

figure(1); clf; hold on;
plot(angle, torque,'ko');

P = polyfit(angle, torque,1);

slope = P(1);

plot(angle, slope*angle,'k-');

fprintf('Hip Spring Constant: %3.3f Nm / rad \n',slope);
fprintf('Hip Spring Reference Angle: %3.3f rad \n',0.0);





%%%% ANKLE JOINT:

% Run tests alternating sign for consistent histeresis

data = [ ...% torque, angle
  -0.15, 0.20;  % Lower Limit
  -0.14, 0.42;
  -0.13, 0.65;
  -0.12, 1.01;
  -0.11, 0.78;
  -0.1, 1.28;
  -0.09, 1.37;
  -0.08, 1.53;

  0.08, 2.02;
  0.09, 2.11;
  0.1, 2.18;
  0.11, 2.46;
  0.12, 2.37;
  0.13, 2.66;
  0.14, 2.87;
  0.15, 2.68];  % Upper Limit

torque = data(:,1);
angle = data(:,2);

P = polyfit(angle,torque,1);

figure(2); clf; hold on;
plot(angle,torque,'ko');
plot(angle,polyval(P,angle),'k-');

fprintf('Ankle Spring Constant: %3.3f Nm / rad \n',P(1));
fprintf('Ankle Spring Reference Angle: %3.3f rad \n',-P(2)/P(1));

