% This data was collected with the robot hanging from the center of the top
% of the outer hip frame. The steady-state was recorded when the free leg
% was hanging vertically, by external torques applied to the other leg.
%
% Current was computed using the T/(KG )motor model, neglecting friction
% terms
%
%


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