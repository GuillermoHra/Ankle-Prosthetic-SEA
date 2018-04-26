% State-Machine for ankle prosthesis
% PID for position control
% TODO: Angle-Voltage Function (Done)
% TODO: State-Machine
close all
clear
clc

a = arduino('COM20', 'Mega2560');

% Initial state. theta = .77

% CP. theta = .4

% CD. theta = .77

% PP. theta = 1.4 (Max. Torque). Go back to CP.

%% PID Code
% PID parameters
Kp = 15; %4.061;
Kd = 0; %171.6;
Ki = 2; %.8816;

N = 300;
r = 1.5 * ones(N); %G(k)
T = .0338;

% Initialization of first samples
v = readVoltage(a,'A7');
c(1) = v;
e(1) = r(1)-c(1);
m(1) = (1/(2*T))*((2*T*Kp+Ki*(T^2)+4*Kd)*e(1));
if abs(m(1)) > 5
    writePWMVoltage(a, 'D10', 5);
    writePWMVoltage(a, 'D11', 0);
elseif abs(m(1)) > 2
    writePWMVoltage(a, 'D10', abs(m(1)));
    writePWMVoltage(a, 'D11', 0);
else
    writePWMVoltage(a, 'D10', 0);
    writePWMVoltage(a, 'D11', 0);
end

v = readVoltage(a,'A7');
c(2) = v;
e(2) = r(2)-c(2);
m(2) = (1/(2*T))*((2*T*Kp+Ki*(T^2)+4*Kd)*e(2) + (2*Ki*(T^2)-8*Kd)*e(1));
if abs(m(2)) > 5
    writePWMVoltage(a, 'D10', 5);
    writePWMVoltage(a, 'D11', 0);
elseif abs(m(2)) > 5
    writePWMVoltage(a, 'D10', abs(m(2)));
    writePWMVoltage(a, 'D11', 0);
else
    writePWMVoltage(a, 'D10', 0);
    writePWMVoltage(a, 'D11', 0);
end

%Main loop, calculation of process output c(k), error e(k), and manipulation m(k)
for k=3:1:N
    v = readVoltage(a,'A7');
    c(k) = v;
    e(k) = r(k)-c(k);
    m(k) = (1/(2*T))*((2*T*m(k-2)) + ((2*T*Kp+Ki*(T^2)+4*Kd)*e(k)) + ((2*Ki*(T^2)-8*Kd)*e(k-1)) + ((-2*T*Kp+Ki*(T^2)+4*Kd)*e(k-2)));
    if e(k) > 0
        if abs(m(k)) > 5
            writePWMVoltage(a, 'D10', 5);
            writePWMVoltage(a, 'D11', 0);
        elseif abs(m(k)) > 2
            writePWMVoltage(a, 'D10', abs(m(k)));
            writePWMVoltage(a, 'D11', 0);
        else
            writePWMVoltage(a, 'D10', 0);
            writePWMVoltage(a, 'D11', 0);
        end
    else
        if abs(m(k)) > 5
            writePWMVoltage(a, 'D10', 0);
            writePWMVoltage(a, 'D11', 5);
        elseif abs(m(k)) > 2
            writePWMVoltage(a, 'D10', 0);
            writePWMVoltage(a, 'D11', abs(m(k)));
        else
            writePWMVoltage(a, 'D10', 0);
            writePWMVoltage(a, 'D11', 0);
        end
    end
end

writeDigitalPin(a, 'D11', 0);
writeDigitalPin(a, 'D10', 0);

%% END of PID controller

% Plots
T=1*(1:N);
subplot(3,1,1),plot(T,c,'r-');
title('PID Controller');
xlabel('k');ylabel('output');grid;legend('c(k)');
subplot(3,1,2),plot(T,e,'b-');
xlabel('k');ylabel('error');grid;legend('e(k)');
subplot(3,1,3),plot(T,m,'k-');
xlabel('k');ylabel('manipulation');grid;legend('m(k)');
