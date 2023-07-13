%Function used to evaluate the joint angles using the cartesian coordinates in Trajectory Plotting

function coord = InverseK(leg,L1,L2,L3,x,y,z)
         X = -x;
         Y = y;
         Z = -z;
         a = (sqrt(Y^2+Z^2) - L1);
         R = sqrt(a^2 + X^2);

         if leg == "LF" || leg == "RH"

             theta1 = atan2(Y,Z);
             theta3 = -acos((a^2 + X^2 - L2^2 - L3^2)/(2*L2*L3));
             theta2 = asin(X/R) - atan(L3*sin(theta3)/(L2 + L3*cos(theta3)));
    
%              Theta1 = 180*theta1/pi;
%              Theta2 = 180*theta2/pi;
%              Theta3 = 180*theta3/pi;
%     
             coord = [theta1 theta2 theta3];
          
         elseif leg == "LH" || leg =="RF"

             theta1 = atan2(Y,Z);
             theta3 = acos((a^2 + X^2 - L2^2 - L3^2)/(2*L2*L3));
             theta2 = asin(X/R) - atan(L3*sin(theta3)/(L2 + L3*cos(theta3)));
    
%              Theta1 = 180*theta1/pi;
%              Theta2 = 180*theta2/pi;
%              Theta3 = 180*theta3/pi;
    
             coord = [theta1 theta2 theta3];

         end

end
