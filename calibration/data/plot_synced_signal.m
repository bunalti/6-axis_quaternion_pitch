% [enc_val ,enc_time] = xlsread("ecnoder_quasi_2.xlsx");
% [imu_val ,imu_time] = xlsread("sd_quasi_2.xlsx");
% enc_time = datenum(enc_time,'hh:MM:ss.fff');
% imu_time = datenum(imu_time,'hh:MM:ss.fff');

imu_trap = trapz(imu_time, imu_val);

sum = 0
for x = 1:(size(imu_val)-1)

    sum = sum + imu_val(x)*(imu_time(x+1)-imu_time(x))


end

enc_trap = trapz(enc_time, enc_val);

time_avg = ((enc_time(end)-enc_time(1))+(imu_time(end)-imu_time(1)))/2;

mean_error = (enc_trap - imu_trap)/time_avg;
%mean_error = (enc_trap - sum)/time_avg;

subplot(2,1,1);
plot(imu_time,imu_val,'Color','blue'); 
title('Dynamic Test #2-1');
xlabel('Time');
ylabel('Degrees');
hold on; 
plot(enc_time,enc_val,'Color','red'); 
legend('IMU','Encoder');
t=text(enc_time(1), 50, ['mean error(deg) = ' num2str(mean_error)]);
t.FontSize = 25;
hold off;



subplot(2,1,2);
title('Dynamic Test #2-1 - Crank Speed');
xlabel('Time');
ylabel('Rotational Speed (RPM)');
plot(enc_time,enc_speed);
q=text(enc_time(1), 50, ['mean speed(rpm) = ' num2str(mean(enc_speed))]);
q.FontSize = 25;
