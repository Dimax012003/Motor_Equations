%%
des_pos = 5;
%w=0.5,2,5
%   The input setpoint is in Volts and can vary from 0 to 10 Volts because the position pot is refered to GND

V_7805=5.48;
Vref_arduino=5;

legacy = false;
tic
 
t=0;
timeData=[];

if ~exist('a','var')
    % clear
    % delete(instrfind({'Port'},{'COM7'}));    
    if legacy
        a=arduino('COM3'); % 'COM3' needs to be set manually into the correct port of Arduino
    else
        a = arduino;
    end
end

% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %
if legacy
    analogWrite(a,6,0);
    analogWrite(a,9,0);
else
    writePWMVoltage(a, 'D6', 0)
    writePWMVoltage(a, 'D9', 0)
end

%επιθυμητοι πολοι
p1 = 1.5;
p2 = 1.5;
p3 = 1.5;

%συστημα
%επιλεγω θετικα κερδη
km = 155.39;
Tm = 0.568;
ke = 1/36;
ko = 0.24667;
kt = 9.2667*0.0001;

%υπολογισμος κερδων
% κανονικα k1,ki ειναι με μειον
k1 = Tm*(p1*p2+p1*p3+p2*p3)/(km*ko*ke);
k2 = (Tm*(p1+p2+p3)/(km*kt))-(1/(km*kt));
ki = Tm*(p1*p2*p3)/(ko*ke*km);

%αρχικοποιηση πινακων
zData=[];
positionData = [];
velocityData = [];
uData = [];
timeData = [];

t=0;

% CLOSE ALL PREVIOUS FIGURES 
close all

% WAIT A KEY TO PROCEED
disp(['Connect cable from Arduino to Input Power Amplifier and then press enter to start controller']);
pause()



%START CLOCK
tic
i=0; 
z=0;
while(t<8)  
i = i+1;
if legacy
    velocity = analogRead(a,3);
    position = analogRead(a,5);
    theta = 3 * Vref_arduino * position / 1023;
    vtacho = 2 * (2 * velocity * Vref_arduino / 1023 - V_7805);
else
    position = readVoltage(a, 'A5'); % position
    velocity = readVoltage(a,'A3'); % velocity
    theta = 3 * Vref_arduino * position / 5;
    vtacho = 2 * (2 * velocity * Vref_arduino / 5 - V_7805);
end

 
u = -k1*theta-k2*vtacho-ki*z;
if(u>10)
    u=10;
end
if(u<-10)
    u=-10;
end

if u > 0
    if legacy
        analogWrite(a,6,0);
        analogWrite(a,9, min(round(u/2 * 255/ Vref_arduino) , 255)); %min is used to saturate
    else        
        writePWMVoltage(a, 'D6', 0);
        writePWMVoltage(a, 'D9', min(abs(u) / 2, 5));
    end
else
     if legacy
        analogWrite(a,9,0);
        analogWrite(a,6, min(round(-u/2 * 255/ Vref_arduino) , 255)); %min is used to saturate
    else        
        writePWMVoltage(a, 'D9', 0);
        writePWMVoltage(a, 'D6', min(abs(u) / 2, 5));
     end
end
t = toc;
if(i==1)
    dt = t;
    z = 0 +(theta -des_pos)*dt;
end
if(i>1)
    dt = t-timeData(i-1);
    z = zData(i-1) + (theta - des_pos)*dt;
end    
timeData = [timeData t];
positionData = [positionData theta];
velocityData = [velocityData vtacho];
uData = [uData u];
zData = [zData z];

end

% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %
if legacy
    analogWrite(a,6,0);
    analogWrite(a,9,0);
else
    writePWMVoltage(a, 'D6', 0)
    writePWMVoltage(a, 'D9', 0)
end


disp(['End of control Loop. Press enter to see diagramms']);
pause();

%%
figure
plot(timeData,positionData,timeData,velocityData,timeData,des_pos*ones(length(timeData)));
grid on;
legend('x1 θεση','x2 ταχυτητα','επιθυμητη θεση');
figure
plot(timeData,uData);
title('uin');
grid on;
% save(filename, 'epsc')
% close gcf;
%%
save('dynamic_feedback.mat','k1','k2','ki');
disp('Disonnect cable from Arduino to Input Power Amplifier and then press enter to stop controller');
pause();