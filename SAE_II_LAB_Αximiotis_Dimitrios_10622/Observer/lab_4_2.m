% % % %%
pos = 5;
%w=0.5,2,5
%   The input setpoint is in Volts and can vary from 0 to 10 Volts because the position pot is refered to GND

V_7805=5.48;
Vref_arduino=4.90607;

legacy = false;
tic
 
t=0;
timeData=[];

if ~exist('a','var')
      clear
      delete(instrfind({'Port'},{'COM7'}));    
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

des_pos=[];

%START CLOCK
%% επιλεγω θετικα κερδη
km = 155.39;
Tm = 0.568;
ke = 1/36;
ko = 0.24667;
kt = 9.2667*0.0001;
k1 =1;
k2 = (2*Tm*sqrt(((km*kt)/(Tm))*(-k1)*((-ke*ko)/kt))-1)/(km*kt);
p1=9;
p2=9;
l1=p1+p2-1/Tm;
l2=(kt/(ko*ke))*((l1/Tm)-p1*p2);
C=[1 0];
B=[0;km*kt/Tm];
A=[0,-ke*ko/kt;0,-1/Tm];
L=[l1;l2];
k=[k1,k2];
kr=-1/(C*(A-B*k)^(-1)*B);

x_e=[2;0];

tic
x1_edata=[];
x2_edata=[];
u=0;
i=0;
while(t<5)  
i=i+1;
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
 
 %%des_pos=[des_pos 5+2*sin(0.5*t)];
 u = -k*x_e+kr*5;
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


t=toc;
if(i==1)
    dt = t;

end
if(i>1)
    dt = t-timeData(i-1);

end    
x_e=x_e+(A*x_e+B*u+L*(theta-C*x_e))*dt;


x1_edata=[x1_edata x_e(1)];
x2_edata=[x2_edata -x_e(2)];
timeData = [timeData t];
positionData = [positionData theta];
velocityData = [velocityData vtacho];
uData = [uData u];

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
plot(timeData,positionData,timeData,velocityData,timeData,x1_edata,timeData,x2_edata);
grid on;
legend('x1 θεση','x2 ταχυτητα','x1_e θεση παρατηρητή','x2_e ταχύτητα παρατηρητή');
figure
plot(timeData,uData);
title('uin');
grid on;

%%
disp('Disonnect cable from Arduino to Input Power Amplifier and then press enter to stop controller');
pause();