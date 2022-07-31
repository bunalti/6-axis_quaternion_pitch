
k = size(encoder) / size(imu);
k = k-rem(k,1);

[imu_pks,imu_locs] = findpeaks(imu,'MinPeakHeight',350);




for i = 1: (k*2)
    downsmp_enc = downsample(encoder,i);

    [encoder_pks,encoder_locs] = findpeaks(downsmp_enc,'MinPeakHeight',320);

    error1 = (encoder_locs(2)-encoder_locs(1)) - (imu_locs(2)-imu_locs(1));
    error2 = (encoder_locs(end)-encoder_locs(end-1)) - (imu_locs(end)-imu_locs(end-1));


    error = error1 + error2;

    error = error/2;


    if(abs(error) < 1)
        break;
    end
end

tail = size(downsmp_enc) - size(imu);
tail_project = tail * i ;

trim_tail_enc = encoder(1:(end-tail_project));

i=0;
for i = 1: (k*2)
    downsmp_enc = downsample(trim_tail_enc,i);

    [encoder_pks,encoder_locs] = findpeaks(downsmp_enc,'MinPeakHeight',320);

    error1 = (encoder_locs(2)-encoder_locs(1)) - (imu_locs(2)-imu_locs(1));
    error2 = (encoder_locs(end)-encoder_locs(end-1)) - (imu_locs(end)-imu_locs(end-1));


    error = error1 + error2;

    error = error/2;


    if(abs(error) < 1)
        break;
    end
end




delay=0;

for d = 1:size(encoder_locs)
    delay = delay + (encoder_locs(d)-imu_locs(d));
end

delay = delay / 17;

if(delay < 0)
    delay = abs(delay);
    delay = delay - rem(delay,1);
    imu_phase = imu(delay:end);
    [imu_pks,imu_locs] = findpeaks(imu_phase,'MinPeakHeight',320);
end



figure; plot(imu_phase,'Color','blue'); hold on; plot(downsmp_enc,'Color','red'); hold off;
title('105rpm');


for q=1:size(encoder_locs)
    peak_differences(q) = encoder_locs(q) - imu_locs(q);
end

[min_imu, dex_imu] = min(abs(peak_differences));

width = (imu_locs(dex_imu)-imu_locs(dex_imu-1));
width = width * 1.3;
width = width - rem(width,1);


starting_dex = imu_locs(dex_imu) - (width/2 - rem(width/2,1));

error = 0;
for e = 1:width

    error(e) = (downsmp_enc(starting_dex+e)-imu_phase(starting_dex+e));

end


mean_error = mean(error);
hold on;
t=text(0.2, 21, ['mean error = ' num2str(mean_error)]);
t.FontSize = 25;
hold off;


