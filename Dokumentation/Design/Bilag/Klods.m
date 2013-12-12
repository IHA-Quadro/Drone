% a = 5.7;    %Max lenght of plastic
% b = 1;      %Depth of plastic
% c = 1.5;    %Height of sonar
% d = 2;      %Sonar PCB's width
% e = 50;     %Length of deadspace
% 
% format shortg
% 
% P1 = [0, b];
% P2 = [0, b+c];
% P7 = [-a/2, 0];
% P6 = [-d/2, b];
% P8 = [-P7(1), 0];
% P9 = [-P6(1), b];
% 
% angleP7P6 = (b-P7(2))/P6(1)-P7(1);
% 
% P3=[(P7(1)-P6(1))/2+P6(1), b/2];
% angleP3P4 = -1/angleP7P6;
% 
% yP4 = abs(sin(atan(angleP3P4))*c/sin(90))+P3(2);
% P4b = P3(2)-angleP3P4*P3(1);
% P4 = [(yP4-P4b)/angleP3P4, yP4];
% 
% distP4P3 = sqrt((P3(1)-P4(1))^2+(P3(2)-P4(2))^2);
% 
% distP2P4 = sqrt((P2(1)-P4(1))^2+(P2(2)-P4(2))^2);
% Deadspace = (distP2P4 + 2*e)/2
% 
% Data = [
%     P7(1) 0 P8(1) P9(1) P1(1) P2(1) P6(1) P3(1) P4(1);
%     P7(2) 0 P8(2) P9(2) P1(2) P2(2) P6(2) P3(2) P4(2)
%     ];
% Data = Data';
% 
% Connections = [
%     0 1 0 0 0 0 0 1 0; %P7
%     1 0 1 0 0 0 0 0 0; %Origo
%     0 0 1 0 0 0 0 0 0; %P8
%     0 0 1 0 0 0 1 0 0; %P9
%     0 0 0 0 0 1 1 0 0; %P1
%     0 0 0 0 0 0 0 0 1; %P2
%     0 0 0 0 0 0 0 1 0; %P6
%     0 0 0 0 0 0 0 0 1; %P3
%     0 0 0 0 0 0 0 0 0];%P4 
% 
% Points = P6;
% 
% figure(1)
% gplot(Connections, Data, '-*')
% axis square;
% axis on;
% axis([-4 4 -0.5 3]);


a = 5.7;    %Max lenght of plastic
d = 1;      %Depth of plastic
c = 1.5;    %Height of sonar
b = 2;      %Sonar PCB's width
e = 50;     %Length of deadspace
SonarAngle = 67;

format shortg

P1 = [0, 0];
P2 = [b/2, 0];
P8 = [b/2, c];
P9 = [0, 0];

distP8P7 = 2.6; %Assume distance between tip of sonars are 3.5 - adjust to see effects

angleP7P9P8 = acosd((e^2+e^2-distP8P7^2)/(2*e*e));
angleP7P8P9 = (180 - angleP7P9P8)/2;

arc1 = SonarAngle/2 + angleP7P8P9 - 90;
aP7P8 = tand(arc1);
bP7P8 = P8(2)-aP7P8*P8(1);
P7y = P8(2) - sind(atand(aP7P8))*distP8P7;
P7x = (P7y - bP7P8)/aP7P8;
P7 = [P7x, P7y];


arc2 = SonarAngle/2 + angleP7P8P9;
aP7P6 = tand(arc2);
bP7P6 = P7(2) - aP7P6 * P7(1);
P6y = P7(2) + sind(atand(aP7P6)) * c;
P6x = (P6y - bP7P6) / aP7P6;
P6 = [P6x, -1/2*d];

aP6P1 = -1/aP7P6;
vinkel = atand(aP6P1)
P5=[P6(1)*2, -d];

P3= [P2(1)*2, 0];
P4= [-P5(1)+P3(1), P5(2)];



%Dataplot
Data = [
    P1(1) P2(1) P3(1) P4(1) P5(1) P6(1) P7(1) P8(1) P9(1);
    P1(2) P2(2) P3(2) P4(2) P5(2) P6(2) P7(2) P8(2) P9(2)
    ];
Data = Data';

Connections = [
   %1 2 3 4 5 6 7 8 9
    1 1 0 0 0 1 0 0 0; %P1
    1 1 1 0 0 0 0 1 0; %P2
    0 1 1 1 0 0 0 0 0; %P3
    0 0 1 1 1 0 0 0 0; %P4
    0 0 0 1 1 1 0 0 0; %P5
    1 0 0 0 1 1 1 0 0; %P6
    0 0 0 0 0 1 1 0 0; %P7
    0 1 0 0 0 0 0 1 0; %P8
    0 0 0 0 0 0 0 0 0];%P9 

figure(2)
gplot(Connections, Data, '-*')
%axis square;
axis on;
axis equal;
axis([-1.3 3 -1.2 1.7]);

    