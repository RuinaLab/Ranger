function Display_Results(Results)

figure(1)
plot(Results.Time,Results.State)
title('States vs. Time')

figure(2)
plot(Results.Mode,'LineWidth',3)
title('Dynamics Mode')
ylabel('1=DS, 2=SS, 3=HS')

end