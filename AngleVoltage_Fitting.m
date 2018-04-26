% Voltage - Angle
close all
clear
clc

angle_volt = [0.4008	22
0.4594	18
0.5279	14
0.5572	12
0.6061	10
0.6598	6
0.6794	5
0.69	4
0.7087	3
0.7527	1
0.7722	0
0.8456	-4
0.914	-6
0.9726	-11
1.0313	-16
1.1144	-19
1.2121	-24
1.2903	-28
1.4	-32];

% for i=1:20
%     angle_volt(i,3) = 90 - angle_volt(i,2);
% end
figure
plot(angle_volt(:,1), angle_volt(:,2));
xlabel('Voltage (V)');
ylabel('Angle (°)');
title('Voltage-Angle');

% Polinomial fitting
[p] = polyfit(angle_volt(:,1), angle_volt(:,2), 1);
f = polyval(p, angle_volt(:,1));
T = table(angle_volt(:,1), angle_volt(:,2), f, angle_volt(:,2)-f,'VariableNames',{'X','Y','Fit','FitError'})

