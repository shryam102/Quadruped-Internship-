%Function used in Trajectory_Plot
function y = draw(leg,tf,L,W,Initial_pos,Final_pos,Initial_vel,Final_vel)

    %% Intial and final end-effector postions
    X_init =  Initial_pos(1); Y_init = Initial_pos(2);  Z_init = Initial_pos(3);
    X_fin  =  Final_pos(1) ; Y_fin = Final_pos(2);  Z_fin  = Final_pos(3); 
    
    
    %% Intial and final end-effector velocties 
    Xdot_init = Initial_vel(1); Ydot_init = Initial_vel(2);  Zdot_init = Initial_vel(3);
    Xdot_fin = Final_vel(1);  Ydot_fin = Final_vel(2);  Zdot_fin = Final_vel(3);
    
    Pdot_init = [Xdot_init;Ydot_init;Zdot_init];
    Pdot_fin = [Xdot_fin;Ydot_fin;Zdot_fin];
   
    t = 0:0.1:tf;
    L1 = 5; L2 = 10; L3 = 10;
    Th1 = zeros(1,length(t));
    Th2 = zeros(1,length(t));
    Th3 = zeros(1,length(t));
        
    X_ef = zeros(1,length(t));
    Y_ef = zeros(1,length(t));
    Z_ef = zeros(1,length(t));
        
    X_kfe = zeros(1,length(t));
    Y_kfe = zeros(1,length(t));
    Z_kfe = zeros(1,length(t));
        
    X_hfe = zeros(1,length(t));
    Y_hfe = zeros(1,length(t));
    Z_hfe = zeros(1,length(t));

    A = [1 0 0 0;
         0 1 0 0;
         1 tf tf^2 tf^3;
         0 1 2*tf 3*tf^2;];

    if leg == "LF"
        %% Joint position intial and final
        Th_init = InverseK("LF",L1,L2,L3,X_init,Y_init,Z_init);
        Th_fin =  InverseK("LF",L1,L2,L3,X_fin,Y_fin,Z_fin);
        
        J_init = Jacobian_plot("LF",L1,L2,L3,Th_init);
        
        J_fin = Jacobian_plot("LF",L1,L2,L3,Th_fin);
        
        Thdot_init = J_init\Pdot_init;
        Thdot_fin = J_fin\Pdot_fin;
        
        
        b1 = [Th_init(1); Thdot_init(1); Th_fin(1); Thdot_fin(1);];
        b2 = [Th_init(2); Thdot_init(2); Th_fin(2); Thdot_fin(2);];
        b3 = [Th_init(3); Thdot_init(3); Th_fin(3); Thdot_fin(3);];
        Tc1 = A\b1;
        Tc2 = A\b2;
        Tc3 = A\b3;
        
        for i = 1:length(t)
            %% Joint space trajectory
            Th1(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc1,3);
            Th2(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc2,3);
            Th3(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc3,3);
        
            %% Task space trajectory
                         %% HFE
            X_hfe(i) =  L/2;
            Y_hfe(i) =  W/2+L1*sin(Th1(i));
            Z_hfe(i) = -L1*cos(Th1(i));
        
                         %% KFE
            X_kfe(i) = L/2-L2*sin(Th2(i));
            Y_kfe(i) =  W/2+ L1*sin(Th1(i)) + L2*cos(Th2(i))*sin(Th1(i));
            Z_kfe(i) = -L1*cos(Th1(i)) - L2*cos(Th1(i))*cos(Th2(i));
        
                         %% End Effector
            X_ef(i) = L/2 -L2*sin(Th2(i))-L3*sin(Th2(i) + Th3(i));
            Y_ef(i) = W/2+sin(Th1(i))*(L1 + L2*cos(Th2(i)) + L3*cos(Th2(i) + Th3(i)));
            Z_ef(i) = -cos(Th1(i))*(L1 + L2*cos(Th2(i)) + L3*cos(Th2(i) + Th3(i))); 

        end
    y = [X_hfe;Y_hfe;Z_hfe;X_kfe;Y_kfe;Z_kfe;X_ef;Y_ef;Z_ef];

    elseif leg == "LH"
        %% Joint position intial and final
        Th_init = InverseK("LH",L1,L2,L3,X_init,Y_init,Z_init);
        Th_fin =  InverseK("LH",L1,L2,L3,X_fin,Y_fin,Z_fin);
        
        J_init = Jacobian_plot("LH",L1,L2,L3,Th_init);
        J_fin = Jacobian_plot("LH",L1,L2,L3,Th_fin);
        
        Thdot_init = J_init\Pdot_init;
        Thdot_fin = J_fin\Pdot_fin;
        
        b1 = [Th_init(1); Thdot_init(1); Th_fin(1); Thdot_fin(1);];
        b2 = [Th_init(2); Thdot_init(2); Th_fin(2); Thdot_fin(2);];
        b3 = [Th_init(3); Thdot_init(3); Th_fin(3); Thdot_fin(3);];
        Tc1 = A\b1;
        Tc2 = A\b2;
        Tc3 = A\b3;
        for i = 1:length(t)
            %% Joint space trajectory
            Th1(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc1,3);
            Th2(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc2,3);
            Th3(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc3,3);
        
            %% Task space trajectory
                         %% HFE
            X_hfe(i) =  -L/2;
            Y_hfe(i) =  W/2+L1*sin(Th1(i));
            Z_hfe(i) = -L1*cos(Th1(i));
        
                         %% KFE
            X_kfe(i) = -L/2-L2*sin(Th2(i));
            Y_kfe(i) =  W/2+ L1*sin(Th1(i)) + L2*cos(Th2(i))*sin(Th1(i));
            Z_kfe(i) = -L1*cos(Th1(i)) - L2*cos(Th1(i))*cos(Th2(i));
        
                         %% End Effector
            X_ef(i) = -L/2 -L2*sin(Th2(i))-L3*sin(Th2(i) + Th3(i));
            Y_ef(i) =  W/2+sin(Th1(i))*(L1 + L2*cos(Th2(i)) + L3*cos(Th2(i) + Th3(i)));
            Z_ef(i) = -cos(Th1(i))*(L1 + L2*cos(Th2(i)) + L3*cos(Th2(i) + Th3(i))); 
        end
    y = [X_hfe;Y_hfe;Z_hfe;X_kfe;Y_kfe;Z_kfe;X_ef;Y_ef;Z_ef];

    elseif leg == "RF"
        %% Joint position intial and final
        Th_init = InverseK("RF",L1,L2,L3,X_init,Y_init,Z_init);
        Th_fin =  InverseK("RF",L1,L2,L3,X_fin,Y_fin,Z_fin);
        
        J_init = Jacobian_plot("RF",L1,L2,L3,Th_init);
        J_fin = Jacobian_plot("RF",L1,L2,L3,Th_fin);
        
        Thdot_init = J_init\Pdot_init;
        Thdot_fin = J_fin\Pdot_fin;
        
        b1 = [Th_init(1); Thdot_init(1); Th_fin(1); Thdot_fin(1);];
        b2 = [Th_init(2); Thdot_init(2); Th_fin(2); Thdot_fin(2);];
        b3 = [Th_init(3); Thdot_init(3); Th_fin(3); Thdot_fin(3);];
        Tc1 = A\b1;
        Tc2 = A\b2;
        Tc3 = A\b3;
        for i = 1:length(t)
            %% Joint space trajectory
            Th1(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc1,3);
            Th2(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc2,3);
            Th3(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc3,3);
        
            %% Task space trajectory
                         %% HFE
            X_hfe(i) =  L/2;
            Y_hfe(i) =  -W/2+L1*sin(Th1(i));
            Z_hfe(i) = -L1*cos(Th1(i));
        
                         %% KFE
            X_kfe(i) = L/2+L2*sin(Th2(i));
            Y_kfe(i) = -W/2+ L1*sin(Th1(i)) + L2*cos(Th2(i))*sin(Th1(i));
            Z_kfe(i) = -L1*cos(Th1(i)) - L2*cos(Th1(i))*cos(Th2(i));
        
                         %% End Effector
            X_ef(i) = L/2 +L2*sin(Th2(i))+L3*sin(Th2(i) + Th3(i));
            Y_ef(i) = -W/2+sin(Th1(i))*(L1 + L2*cos(Th2(i)) + L3*cos(Th2(i) + Th3(i)));
            Z_ef(i) = -cos(Th1(i))*(L1 + L2*cos(Th2(i)) + L3*cos(Th2(i) + Th3(i))); 
        end
    y = [X_hfe;Y_hfe;Z_hfe;X_kfe;Y_kfe;Z_kfe;X_ef;Y_ef;Z_ef];

    elseif leg == "RH"
        %% Joint position intial and final
        Th_init = InverseK("RH",L1,L2,L3,X_init,Y_init,Z_init);
        Th_fin =  InverseK("RH",L1,L2,L3,X_fin,Y_fin,Z_fin);
        
        J_init = Jacobian_plot("RH",L1,L2,L3,Th_init);
        J_fin = Jacobian_plot("RH",L1,L2,L3,Th_fin);
        
        Thdot_init = J_init\Pdot_init;
        Thdot_fin = J_fin\Pdot_fin;
        
        b1 = [Th_init(1); Thdot_init(1); Th_fin(1); Thdot_fin(1);];
        b2 = [Th_init(2); Thdot_init(2); Th_fin(2); Thdot_fin(2);];
        b3 = [Th_init(3); Thdot_init(3); Th_fin(3); Thdot_fin(3);];
        Tc1 = A\b1;
        Tc2 = A\b2;
        Tc3 = A\b3;
        for i = 1:length(t)
            %% Joint space trajectory
            Th1(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc1,3);
            Th2(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc2,3);
            Th3(i) = round([1 t(i) t(i)^2 t(i)^3]*Tc3,3);
        
            %% Task space trajectory
                         %% HFE
            X_hfe(i) =  -L/2;
            Y_hfe(i) =  -W/2+L1*sin(Th1(i));
            Z_hfe(i) = -L1*cos(Th1(i));
        
                         %% KFE
            X_kfe(i) = -L/2+L2*sin(Th2(i));
            Y_kfe(i) = -W/2+ L1*sin(Th1(i)) + L2*cos(Th2(i))*sin(Th1(i));
            Z_kfe(i) = -L1*cos(Th1(i)) - L2*cos(Th1(i))*cos(Th2(i));
        
                         %% End Effector
            X_ef(i) = -L/2 +L2*sin(Th2(i))+L3*sin(Th2(i) + Th3(i));
            Y_ef(i) = -W/2+sin(Th1(i))*(L1 + L2*cos(Th2(i)) + L3*cos(Th2(i) + Th3(i)));
            Z_ef(i) = -cos(Th1(i))*(L1 + L2*cos(Th2(i)) + L3*cos(Th2(i) + Th3(i))); 
        end
    y = [X_hfe;Y_hfe;Z_hfe;X_kfe;Y_kfe;Z_kfe;X_ef;Y_ef;Z_ef];



    end
end
