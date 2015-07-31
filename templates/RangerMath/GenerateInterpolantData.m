function GenerateInterpolantData()

% This function generates the data that is used by the quadratic
% interpolation function approximator that is used in RangerMath.m
%

fid = fopen('QuadInterpData.txt','w');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                               TANH()                                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

X = linspace(-4,4,201);
Y = tanh(X);
name = 'tanh';
writeQuadData(fid, X,Y, name);




fclose(fid);
end



function writeQuadData(fid, X,Y, name)

fprintf(fid,['// Data for quadratic interpolation of ' name '\n']);

fprintf(fid,'static float X[] = {');
for i=1:length(X)
    if i==1
        fprintf(fid, '%12.12f',X(i)); 
    else
        fprintf(fid, ', %12.12f',X(i)); 
    end
end
fprintf(fid,'};\n');

fprintf(fid,'static float Y[] = {');
for i=1:length(Y)
    if i==1
        fprintf(fid, '%12.12f',Y(i)); 
    else
        fprintf(fid, ', %12.12f',Y(i)); 
    end
end
fprintf(fid,'};\n');

h = mean(diff(X));
hCstScale = 1/(2*h*h);
hCstGrid = 1/(2*h);
nGrid = length(X);


fprintf(fid, 'static float nGrid = %d; \n',nGrid);
fprintf(fid, 'static float hCstScale = %12.12f; \n',hCstScale); 
fprintf(fid, 'static float hCstGrid = %12.12f; \n',hCstGrid); 

end
