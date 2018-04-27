% Real-time plot test
clear
clc
close all

a = arduino('COM20', 'Mega2560');
%% Record and plot voltage data
cont = 0;
samples = [];
t = [];
voltageData = [];
stop = 1;

disp('Start time stamp')
tic
while stop
    writeDigitalPin(a, 'D11', 0);
    writeDigitalPin(a, 'D12', 1);
    cont = cont + 1;
    % Read current voltage value
    v = readVoltage(a,'A7');
    voltageData(cont) = v;
    samples(cont) = cont;
    % Get time since starting
    t(cont) = toc;
    % Stop condition
    if v < .4 % v < 0.4
        stop = 0;
    end
end
cont = cont + 1;
samples(cont) = toc;
toc

writeDigitalPin(a, 'D11', 0);
writeDigitalPin(a, 'D12', 0);
% Post-process and plot the data
[xS, yS] = size(samples);
figure
plot(samples(1,1:yS-1),voltageData,'-')
xlabel('Samples')
ylabel('Voltage (V)')
title('Voltage data from pot')
set(gca,'xlim',[1 cont])

% Compute data acquisition rate without real-time plotting
timeBetweenDataPoints = diff(t);
averageTimePerDataPoint = mean(timeBetweenDataPoints);
dataRateHz = 1/averageTimePerDataPoint
% Compute sampling frequency
sfreq = 0;
sfreq = cont / toc

% Add dataRateHz and sfreq
cont = cont + 1;
samples(cont) = cont;
voltageData(cont) = dataRateHz;
cont = cont + 1;
samples(cont) = cont;
voltageData(cont) = sfreq;

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
% figure
% h = animatedline;
% ax = gca;
% ax.YGrid = 'on';
% ax.YLim = [0 5];
% samples = 0;
% stop = 1;
% trt = [];
% 
% tic
% while(stop)
%     writeDigitalPin(a, 'D11', 1);
%     writeDigitalPin(a, 'D12', 0);
%     samples = samples + 1;
%     % Read current voltage value
%     v = readVoltage(a,'A7');  
%     % Add points to animation
%     addpoints(h,samples,v)
%     % Update axes
%     ax.XLim = [(samples - 100) samples]; 
%     %datetick('x','keeplimits')
%     drawnow
%     trt(samples) = toc;
%     % Stop condition
%     %w = waitforbuttonpress;
%     %if w == 1
%         %stop = 0;
%     %end
% end
% toc
% writeDigitalPin(a, 'D11', 0);
% writeDigitalPin(a, 'D12', 0);
% %Compute data acquisition rate during real-time plotting
% timeBetweenDataPoints = diff(trt);
% averageTimePerDataPoint = mean(timeBetweenDataPoints);
% dataRateHz = 1/averageTimePerDataPoint
% 
% %Compute sampling frequency
% sfreq = 0;
% sfreq = samples / toc
%% Plot the recorded data
% [samples,voltageLogs] = getpoints(h);
% figure
% plot(samples,voltageLogs)
% xlabel('Samples')
% ylabel('Voltage (V)')

%% Save results to a file
% T = table(samples',voltageData','VariableNames',{'Samples','Voltage'});
% filename = 'TF_B3.txt';
% % Write table to file 
% writetable(T,filename)
% disp('Data saved to file')
