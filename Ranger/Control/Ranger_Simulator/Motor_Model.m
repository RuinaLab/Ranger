function U = Motor_Model(U_Cmd,P)

I_MAX = P.robot.motor.I_MAX;
[N,M] = size(U_Cmd);

% For now, do a smooth saturation of the input:
    Bounds = [-I_MAX, I_MAX];   
    alpha = P.robot.motor.Sat_Smooth_Parameter;  %Smoothing parameter
    u_cmd = reshape(U_Cmd,N*M,1);   %Flatten to a vector
    u = Smooth_Saturation(u_cmd,Bounds,alpha);   %Do the saturation
    U = reshape(u,N,M);    %Convert back to a matrix

end