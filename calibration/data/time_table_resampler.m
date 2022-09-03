
imu_time = LD.imu_time;
imu_val = LD.imu_val;
DT = datetime(imu_time, 'ConvertFrom','datenum');
DT.Format = 'HH:mm:ss.SSSSSS';
T1 = table(DT,enc_val)
TT1 = table2timetable(T1);
Fs = 500;                                                       % Resampling  Frequency
TT1 = retime(TT1,'regular','SampleRate',Fs)

a=1;
q=1;

for i = 1:size(ZZ1.enc_val)
if isnan(ZZ1.enc_val(i))
    a = a + 1;
else
    for x = q:i
        ZZ1.enc_val(x) = ZZ1.enc_val(i);
    end
    a = 1;
    q = 1 + x;
end
end

figure
plot(DT, imu_val)
grid
title('Original')

figure
plot(b.DT,b.enc_val, '.', 'MarkerSize',0.1,'Color','red')
hold on;
plot(a.DT,a.imu_val, '.', 'MarkerSize',0.1,'Color','blue')
grid
title(sprintf('After ''retime'' (Fs = %0.1f Hz)',Fs))


figure;
plot(enc_time,enc_val);
hold on;
plot(imu_time,imu_val);
hold off;