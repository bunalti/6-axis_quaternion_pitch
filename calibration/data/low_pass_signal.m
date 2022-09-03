
t = imu_time;                                                     %Time Array
s = imu_val;                                                      %Sample Array
Fs = 0.1;                                                             % Sampling Frequency
Ts = 1;                                                             % Sampling Interval
[sr,tr] = resample(s, t, Fs);                                       % Resample, Return Resampled Signal & New Time Vector
sre = sr(1:end-2);                                                  % Eliminate End Transient
tre = tr(1:end-2);                                                  % Eliminate End Transient
figure
plot(t, s)
hold on
plot(tre, sre, '--')
hold off
grid
legend('Original Signal', 'Resampled Signal')
L = numel(t);                                                       % Signal Length
Fn = Fs/2;                                                          % Nyquist Frequency
sm = sre - mean(sre);                                               % Subtract Mean
FTs = fft(sm)/L;                                                    % Scaled Fourier Transform
Fv = linspace(0, 1, fix(L/2)+1)*Fn;                                 % Frequency Vector
Iv = 1:numel(Fv);                                                   % Index Vector
figure
plot(Fv, abs(FTs(Iv))*2)
grid
title('Fourier Transform')
Wp = [0.05]/Fn;                                                     % Passband Frequency (Normalised)
Ws = [0.09]/Fn;                                                     % Stopband Frequency (Normalised)
Rp =  1;                                                            % Passband Ripple
Rs = 60;                                                            % Passband Ripple (Attenuation)
[n,Wp] = ellipord(Wp,Ws,Rp,Rs);                                     % Elliptic Order Calculation
[z,p,k] = ellip(n,Rp,Rs,Wp,'low');                                  % Elliptic Filter Design: Zero-Pole-Gain 
[sos,g] = zp2sos(z,p,k);                                            % Second-Order Section For Stability
figure
freqz(sos, 2^16, Fs)                                                % Filter Bode Plot
sre_filt = filtfilt(sos, g, sre);                                   % Filter Signal
figure
subplot(2,1,1)
plot(tre, sre)
grid
title('Resampled Signal')
subplot(2,1,2)
plot(tre, sre_filt, '-')
grid
title('Filtered Resampled Signal')