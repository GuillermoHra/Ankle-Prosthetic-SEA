% Voltage - Angle
close all
clear
clc

angle_volt = [0.1955	34
                0.2835	28
                0.3128	25
                0.3812	22
                0.4839	16
                0.5718	10
                0.6598	6
                0.7331	3
                0.7722	0
                0.8553	-5
                0.9677	-11
                1.0997	-20
                1.1828	-24
                1.2903	-29
                1.393	-34
                1.4809	-38
                1.5005	-40];

figure
plot(angle_volt(:,1), angle_volt(:,2));
xlabel('Voltage (V)');
ylabel('Angle (°)');
title('Voltage-Angle');
% Polinomial fitting
[p,S,mu] = polyfit(angle_volt(:,1), angle_volt(:,2), 2)

