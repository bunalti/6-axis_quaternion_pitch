 % Create test data set
 data1 = rand(1,900);
 time1 = 0:0.1:0.9;
 % Insert random displacement
 data2 = data1 + 0.1  * rand(1,10);
 time2 = time1 + 0.04 * rand(1,10);
 % plot the randomized data
 plot(time1,data1,'o-')
 hold on
 plot(time2,data2,'o-')
 % create timeseries
 ts1 = timeseries(data1,time1);
 ts2 = timeseries(data2,time2);

 % Synchronize timeseries via linear interpolation
 [ts1_sync, ts2_sync] = synchronize(ts1,ts2,'Union');
 % Plot synchronized timeseries
 figure
 plot(ts1_sync,'o-')
 hold on
 plot(ts2_sync,'o-')
 try
     plot(ts1_sync-ts2_sync,'o-')
 catch
     warning('Subtraction 2 did not work out.')
 end