% Force Controller
% Linear Pot: down=1, up=5, No movement=3.7

close all
clear
clc

a = arduino('/dev/cu.usbmodem1421', 'Mega2560');
aPositionPin = 'A7';
aForcePin = 'A8';
dForwardPin = 'D10';
dBackwardPin = 'D11';
pos = [];
pos_cont = 1;
force = [];
force_cont = 1;
% PID Code
        % PID parameters
        Kp = 15; %4.061;
        Kd = 0; %171.6;
        Ki = 2; %.8816;

        N = 2000; % Check if its enough
        r = 1.4 * ones(N,1); %G(k)
        T = .0338;

        % Initialization of first samples
        v = readVoltage(a,aPositionPin);
        c(1) = v;
        pos(pos_cont) = c(1);
        pos_cont = pos_cont + 1;
        f = readVoltage(a,aForcePin);
        force(force_cont) = f;
        force_cont = force_cont + 1;
        
        e(1) = r(1)-c(1);
        m(1) = (1/(2*T))*((2*T*Kp+Ki*(T^2)+4*Kd)*e(1));
        if abs(m(1)) > 5
            writePWMVoltage(a, dForwardPin, 5);
            writePWMVoltage(a, dBackwardPin, 0);
        elseif abs(m(1)) > 2
            writePWMVoltage(a, dForwardPin, abs(m(1)));
            writePWMVoltage(a, dBackwardPin, 0);
        else
            writePWMVoltage(a, dForwardPin, 0);
            writePWMVoltage(a, dBackwardPin, 0);
        end

        v = readVoltage(a,aPositionPin);
        c(2) = v;
        pos(pos_cont) = c(2);
        pos_cont = pos_cont + 1;
        f = readVoltage(a,aForcePin);
        force(force_cont) = f;
        force_cont = force_cont + 1;
        e(2) = r(2)-c(2);
        m(2) = (1/(2*T))*((2*T*Kp+Ki*(T^2)+4*Kd)*e(2) + (2*Ki*(T^2)-8*Kd)*e(1));
        if abs(m(2)) > 5
            writePWMVoltage(a, dForwardPin, 5);
            writePWMVoltage(a, dBackwardPin, 0);
        elseif abs(m(2)) > 5
            writePWMVoltage(a, dForwardPin, abs(m(2)));
            writePWMVoltage(a, dBackwardPin, 0);
        else
            writePWMVoltage(a, dForwardPin, 0);
            writePWMVoltage(a, dBackwardPin, 0);
        end

        %Main loop, calculation of process output c(k), error e(k), and manipulation m(k)
        for k=3:1:N
            v = readVoltage(a,aPositionPin);
            c(k) = v;
            pos(pos_cont) = c(k);
            pos_cont = pos_cont + 1;
            f = readVoltage(a,aForcePin);
            force(force_cont) = f;
            force_cont = force_cont + 1;
            e(k) = r(k)-c(k);
            m(k) = (1/(2*T))*((2*T*m(k-2)) + ((2*T*Kp+Ki*(T^2)+4*Kd)*e(k)) + ((2*Ki*(T^2)-8*Kd)*e(k-1)) + ((-2*T*Kp+Ki*(T^2)+4*Kd)*e(k-2)));
            if e(k) > 0
                if f > 2.4 && f < 4.0 % No external force
                    if abs(m(k)) > 5
                        writePWMVoltage(a, dForwardPin, 5);
                        writePWMVoltage(a, dBackwardPin, 0);
                    elseif abs(m(k)) > 2
                        writePWMVoltage(a, dForwardPin, abs(m(k)));
                        writePWMVoltage(a, dBackwardPin, 0);
                    else
                        writePWMVoltage(a, dForwardPin, 0);
                        writePWMVoltage(a, dBackwardPin, 0);
                    end
                else % External force
                    if f > 4.0 % External force going up
                        for i=k:k+3000
                            r(i) = 0.45;
                        end
                        if abs(m(k)) > 5
                            writePWMVoltage(a, dForwardPin, 0);
                            writePWMVoltage(a, dBackwardPin, 5);
                        elseif abs(m(k)) > 2
                            writePWMVoltage(a, dForwardPin, 0);
                            writePWMVoltage(a, dBackwardPin, abs(m(k)));
                        else
                            writePWMVoltage(a, dForwardPin, 0);
                            writePWMVoltage(a, dBackwardPin, 0);
                        end
                    end
                end
            else
                if f > 2.4 && f < 4.0 % No external force
                    if abs(m(k)) > 5
                        writePWMVoltage(a, dForwardPin, 0);
                        writePWMVoltage(a, dBackwardPin, 5);
                    elseif abs(m(k)) > 2
                        writePWMVoltage(a, dForwardPin, 0);
                        writePWMVoltage(a, dBackwardPin, abs(m(k)));
                    else
                        writePWMVoltage(a, dForwardPin, 0);
                        writePWMVoltage(a, dBackwardPin, 0);
                    end
                else % External force
                     if f < 2.4           % External force going down
                        for i=k+1:k+3000
                            r(i) = 1.4;
                        end
                        if abs(m(k)) > 5
                            writePWMVoltage(a, dForwardPin, 5);
                            writePWMVoltage(a, dBackwardPin, 0);
                        elseif abs(m(k)) > 2
                            writePWMVoltage(a, dForwardPin, abs(m(k)));
                            writePWMVoltage(a, dBackwardPin, 0);
                        else
                            writePWMVoltage(a, dForwardPin, 0);
                            writePWMVoltage(a, dBackwardPin, 0);
                        end
                    end
                end
            end
            k = k + 1;
        end

        writeDigitalPin(a, dForwardPin, 0);
        writeDigitalPin(a, dBackwardPin, 0);
        % Plots
        T=1*(1:500);
        subplot(2,1,1),plot(T,pos(1:500),'r-','LineWidth',3);
        title('Force Controller');
        xlabel('Samples');ylabel('Position (volts)');grid;legend('Ankle Position');
        subplot(2,1,2),plot(T,force(1:500),'b-','LineWidth',3);
        xlabel('Samples');ylabel('Force (volts)');grid;legend('External Force');
      
        