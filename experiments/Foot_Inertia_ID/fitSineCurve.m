function channel = fitSineCurve(channel)

nData = length(channel.time);
tSpan = channel.time([1,end]);

t = linspace(tSpan(1),tSpan(2),nData);
x = interp1(channel.time',channel.data',t','pchip');

%%%% Figure out the frequency content:
Fs = (nData-1)/(tSpan(2) - tSpan(1));
NFFT = 2^nextpow2(nData); % Next power of 2 from length of y
X = fft(x,NFFT)/nData; 
Ampl = 2*abs(X(1:NFFT/2+1));
Freq = Fs/2*linspace(0,1,NFFT/2+1);

% % % Plot single-sided amplitude spectrum.
% % plot(Freq,Ampl) 
% % title('Single-Sided Amplitude Spectrum of x(t)')
% % xlabel('Frequency (Hz)')
% % ylabel('|Ampl(f)|')

% Ignore DC content below 0.5 Hz:
idxRm = Freq < 0.5;
Ampl(idxRm) = [];
Freq(idxRm) = [];
[~,idxPeak] = max(Ampl);

% Compute an initial guess at the underlying trig function:
guess.a = mean(channel.data);   %Offset
guess.b = 0.5*(max(x)-min(x));  %Amplitude
guess.c = 2*pi*Freq(idxPeak);   %Frequency

% Find a good guess at the phase shift:
getShift = @(d)( checkFit([guess.a;guess.b;guess.c;d],channel.time,channel.data) );
guess.d = fminbnd(getShift,0,2*pi);

% Find the actual solution:
z0 = [guess.a; guess.b; guess.c; guess.d];
userFun = @(z)( checkFit(z,channel.time,channel.data) );
zSoln = fminsearch(userFun,z0);

% Express signal as functions:
a = zSoln(1);
b = zSoln(2);
c = zSoln(3);
d = zSoln(4);
channel.fun = @(t)( a + b*sin(c*t + d) );
channel.dFun = @(t)( b*c*cos(c*t + d) );
channel.ddFun = @(t)( -b*c*c*sin(c*t + d) );
channel.coeff = [a,b,c,d];

end



function mse = checkFit(param,t,x)

a = param(1);
b = param(2);
c = param(3);
d = param(4);

xFit = a + b*sin(c*t + d);

mse = mean((x-xFit).^2);

end



