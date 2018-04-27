% Position Controller

close all
clear
clc

a = arduino('COM20', 'Mega2560');
pos = [];
pos_cont = 1;

state = 1;
while(1)
    if state == 1
        % PID Code
        % PID parameters
        Kp = 15; %4.061;
        Kd = 0; %171.6;
        Ki = 2; %.8816;

        N = 300; % Check if its enough
        r = 1.4 * ones(N); %G(k)
        T = .0338;

        % Initialization of first samples
        v = readVoltage(a,'A7');
        c(1) = v;
        pos(pos_cont) = c(1);
        pos_cont = pos_cont + 1;
        
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
        pos(pos_cont) = c(2);
        pos_cont = pos_cont + 1;
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
        k = 3;
        while v < 1.35 || v > 1.45 %k=3:1:N
            v = readVoltage(a,'A7');
            c(k) = v;
            pos(pos_cont) = c(k);
            pos_cont = pos_cont + 1
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
            k = k + 1;
        end

        writeDigitalPin(a, 'D11', 0);
        writeDigitalPin(a, 'D10', 0);
        state = 2;
    end

    if state == 2
        % PID Code
        % PID parameters
        Kp = 15; %4.061;
        Kd = 0; %171.6;
        Ki = 2; %.8816;

        N = 300;
        r = 0.45 * ones(N); %G(k)
        T = .0338;

        % Initialization of first samples
        c = [];
        e = [];
        m = [];
        v = readVoltage(a,'A7');
        c(1) = v;
        pos(pos_cont) = c(1);
        pos_cont = pos_cont + 1;
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
        pos(pos_cont) = c(2);
        pos_cont = pos_cont + 1;
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
        k = 3;
        while v < .42 || v > .49 
            v = readVoltage(a,'A7');
            c(k) = v;
            pos(pos_cont) = c(k);
            pos_cont = pos_cont + 1
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
            k = k + 1;
        end

        writeDigitalPin(a, 'D11', 0);
        writeDigitalPin(a, 'D10', 0);
        state = 1;
    end
end   
    
        % Plots
        T=1*(1:500);
        subplot(2,1,1),plot(T,pos(1:500),'r-','LineWidth',3);
        title('Position Controller with and without external perturbations');
        xlabel('Samples');ylabel('Position (volts)');grid;legend('Ankle Position with perturbations');
        ylim([0 2]);
     
        subplot(2,1,2),plot(T,pos(501:1000),'r-','LineWidth',3);
        xlabel('Samples');ylabel('Position (volts)');grid;legend('Ankle Position without perturbations');
        ylim([0 2]);
        