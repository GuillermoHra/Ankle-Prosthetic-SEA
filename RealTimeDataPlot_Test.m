%Real-time plot test
clear
clc

a = arduino;
%v = readVoltage(a, 'A7');

% %Record and plot 10 seconds of voltage date
% cont = 0;
% t = [];
% voltageData = [];
% disp('Start time stamp')
% tic
% while toc < 31.5
%     cont = cont + 1;
%     % Read current voltage value
%     v = readVoltage(a,'A7');
%     voltageData(cont) = v;
%     % Get time since starting
%     t(cont) = toc;
% end
% % Post-process and plot the data
% % Plot temperature versus time
% figure
% plot(t,voltageData,'-')
% xlabel('Elapsed time (sec)')
% ylabel('Voltage (V)')
% title('Ten Seconds of Voltage Data from a pot')
% set(gca,'xlim',[t(1) t(cont)])
% 
% %Compute data acquisition rate without real-time plotting
% timeBetweenDataPoints = diff(t);
% averageTimePerDataPoint = mean(timeBetweenDataPoints);
% dataRateHz = 1/averageTimePerDataPoint

%% Acquire and display live data (in seconds)

% figure
% h = animatedline;
% ax = gca;
% ax.YGrid = 'on';
% ax.YLim = [0 5];
% 
% startTime = datetime('now');
% stop = 1;
% while(stop)
%     % Read current voltage value
%     v = readVoltage(a,'A7');  
%     % Get current time
%     t =  datetime('now') - startTime;
%     % Add points to animation
%     addpoints(h,datenum(t),v)
%     % Update axes
%     ax.XLim = datenum([t-seconds(15) t]);
%     datetick('x','keeplimits')
%     drawnow
%     %*Stop condition ??
%     if (v > 4.5)
%        stop = 0; 
%     end
% end

%% Acquire and display live data (in samples)

figure
h = animatedline;
ax = gca;
ax.YGrid = 'on';
ax.YLim = [0 5];
samples = 0;
stop = 1;
trt = [];
tic
while(stop)
    samples = samples + 1;
    % Read current voltage value
    v = readVoltage(a,'A7');  
    % Add points to animation
    addpoints(h,samples,v)
    % Update axes
    ax.XLim = [(samples - 100) samples]; 
    %datetick('x','keeplimits')
    drawnow
    trt(samples) = toc;
    %Stop condition
    if (v > 4.5)
       stop = 0; 
    end
end
toc
%Compute data acquisition rate during real-time plotting
timeBetweenDataPoints = diff(trt);
averageTimePerDataPoint = mean(timeBetweenDataPoints);
dataRateHz = 1/averageTimePerDataPoint

%Compute sampling frequency
sfreq = 0;
sfreq = samples / toc

%% Plot the recorded data
[samples,voltageLogs] = getpoints(h);
figure
plot(samples,voltageLogs)
xlabel('Samples')
ylabel('Voltage (V)')

% %% Save results to a file
% T = table(samples',voltageLogs','VariableNames',{'Samples','Voltage'});
% filename = 'Voltage_Data.txt';
% % Write table to file 
% writetable(T,filename)
% disp('Data saved to file')
