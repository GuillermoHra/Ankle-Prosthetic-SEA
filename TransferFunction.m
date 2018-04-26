% Transfer Function
close all
clear
clc

data = load('TF_S1.txt', '-ascii');
[xS, yS] = size(data);
voltage = data(1, 2) - data(xS-3,2);
time = data(xS-2,1);
constant(1,1) = voltage/time;
sampling_f(1,1) = data(xS-1,2);

data = load('TF_S2.txt', '-ascii');
[xS, yS] = size(data);
voltage = data(1, 2) - data(xS-3,2);
time = data(xS-2,1);
constant(2,1) = voltage/time;
sampling_f(2,1) = data(xS-1,2);

data = load('TF_S3.txt', '-ascii');
[xS, yS] = size(data);
voltage = data(1, 2) - data(xS-3,2);
time = data(xS-2,1);
constant(3,1) = voltage/time;
sampling_f(3,1) = data(xS-1,2);

constant(4,1) = mean(constant(1:3,1));
sampling_f(4,1) = mean(sampling_f(1:3,1));