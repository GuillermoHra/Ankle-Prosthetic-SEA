% Plot data from text files
close all
clear
clc

data = load('Voltage_DataU_3.txt', '-ascii');
figure
[sizex, sizey] = size(data);
plot(data(1:sizex-2,1), data(1:sizex-2,2));
xlabel('Sample');
ylabel('Voltage (V)');

hold on
data2 = load('Voltage_DataU_4.txt', '-ascii');
[sizex, sizey] = size(data2);
plot(data2(1:sizex-2,1), data2(1:sizex-2,2));

hold on
data3 = load('Voltage_DataU_5.txt', '-ascii');
[sizex, sizey] = size(data3);
plot(data3(1:sizex-2,1), data3(1:sizex-2,2));
