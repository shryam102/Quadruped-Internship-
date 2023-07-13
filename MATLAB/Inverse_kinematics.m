% clc; 
% close all;

x = -5;                                
y = 0;         %Desired position of end-effector
z = -20;
L1 = 5;
L2 = 10;       %Link Lengths
L3 = 10;


X = -x;
Y = y;
Z = -z;
a = (sqrt(Y^2+Z^2) - L1);
R = sqrt(a^2 + X^2);

theta1 = atan2(Y,Z);
theta3 = acos((a^2 + X^2 - L2^2 - L3^2)/(2*L2*L3));
theta2 = asin(X/R) - atan(L3*sin(theta3)/(L2 + L3*cos(theta3)));

theta3_2 = -acos((a^2 + X^2 - L2^2 - L3^2)/(2*L2*L3));
theta2_2 = asin(X/R) - atan(L3*sin(theta3_2)/(L2 + L3*cos(theta3_2)));

Theta1 = 180*theta1/pi;
Theta2 = 180*theta2/pi;
Theta3 = 180*theta3/pi;

Theta2_2 = 180*theta2_2/pi;
Theta3_2 = 180*theta3_2/pi;
     

disp("Configuration 1");                                  
fprintf("Theta1 : %f\t",Theta1);
fprintf("Theta2 : %f\t",Theta2);
fprintf("Theta3 : %f\n",Theta3);

disp("Configuration 2");
fprintf("Theta1 : %f\t",Theta1);
fprintf("Theta2 : %f\t",Theta2_2);
fprintf("Theta3 : %f\n",Theta3_2);


%-------------------Visualisation----------------------------------


P0 = [0,0,0;
      0, L1*sin(theta1), -L1*cos(theta1);
    -L2*sin(theta2), L1*sin(theta1) + L2*cos(theta2)*sin(theta1), -L1*cos(theta1) - L2*cos(theta1)*cos(theta2)];

P1 = [0, L1*sin(theta1), -L1*cos(theta1);
     -L2*sin(theta2), L1*sin(theta1) + L2*cos(theta2)*sin(theta1), -L1*cos(theta1) - L2*cos(theta1)*cos(theta2);
     - L2*sin(theta2) - L3*cos(theta2)*sin(theta3) - L3*cos(theta3)*sin(theta2), L1*sin(theta1) + L2*cos(theta2)*sin(theta1) + L3*cos(theta2)*cos(theta3)*sin(theta1) - L3*sin(theta1)*sin(theta2)*sin(theta3),L3*cos(theta1)*sin(theta2)*sin(theta3) - L2*cos(theta1)*cos(theta2) - L3*cos(theta1)*cos(theta2)*cos(theta3) - L1*cos(theta1) ];

X1 = round([P0(:,1) P1(:,1)],5);
Y1 = round([P0(:,2) P1(:,2)],5);
Z1 = round([P0(:,3) P1(:,3)],5);

P0_2 = [0,0,0;
        0, L1*sin(theta1), -L1*cos(theta1);
    -L2*sin(theta2_2), L1*sin(theta1) + L2*cos(theta2_2)*sin(theta1), -L1*cos(theta1) - L2*cos(theta1)*cos(theta2_2)];

P1_2 = [0, L1*sin(theta1), -L1*cos(theta1);
     -L2*sin(theta2_2), L1*sin(theta1) + L2*cos(theta2_2)*sin(theta1), -L1*cos(theta1) - L2*cos(theta1)*cos(theta2_2);
     - L2*sin(theta2_2) - L3*cos(theta2_2)*sin(theta3_2) - L3*cos(theta3_2)*sin(theta2_2), L1*sin(theta1) + L2*cos(theta2_2)*sin(theta1) + L3*cos(theta2_2)*cos(theta3_2)*sin(theta1) - L3*sin(theta1)*sin(theta2_2)*sin(theta3_2),L3*cos(theta1)*sin(theta2_2)*sin(theta3_2) - L2*cos(theta1)*cos(theta2_2) - L3*cos(theta1)*cos(theta2_2)*cos(theta3_2) - L1*cos(theta1) ];

X2 = round([P0_2(:,1) P1_2(:,1)],5);
Y2 = round([P0_2(:,2) P1_2(:,2)],5);
Z2 = round([P0_2(:,3) P1_2(:,3)],5);

plot3(X1',Y1',Z1','b',LineWidth=3) 
hold on
plot3(X2',Y2',Z2','r',LineWidth=3) 
grid on
xlabel('X axis') 
ylabel('Y axis')
zlabel('Z axis')
title('Visualization')
fprintf('\nBlue is the configuration_1 plot and Red is the configuration_2 plot')





    
