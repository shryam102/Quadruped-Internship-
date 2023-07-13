clc; 
close all;


Stride_length = 10;
Stride_height = 6;
Height_ground = 20;

L = 45;
W = 20;
L1 = 5;
L2 = 10;
L3 = 10;

tf = 10;
t = 0:0.1:tf;

%-------------------Leg(LF)-------------------------------------

initial_pos_1_LF = [-Stride_length/2 0 -Height_ground];
final_pos_1_LF = [0 0 -(Height_ground-Stride_height)];
initial_vel_1_LF = [0.3 0 3.4];
Final_vel_1_LF = [0.3 0 0];
initial_pos_2_LF = [0 0 -(Height_ground-Stride_height)];
final_pos_2_LF = [Stride_length/2 0 -Height_ground];
initial_vel_2_LF = [0.3 0 0];
Final_vel_2_LF = [0.3 0 -3.4];

Plot1_LF = draw("LF",tf/2,L,W,initial_pos_1_LF,final_pos_1_LF,initial_vel_1_LF,Final_vel_1_LF);
Plot2_LF = draw("LF",tf/2,L,W,initial_pos_2_LF,final_pos_2_LF,initial_vel_2_LF,Final_vel_2_LF);

X_hfe_LF = [Plot1_LF(1,1:45) Plot2_LF(1,5:51)];
Y_hfe_LF = [Plot1_LF(2,1:45) Plot2_LF(2,5:51)];
Z_hfe_LF = [Plot1_LF(3,1:45) Plot2_LF(3,5:51)];
X_kfe_LF = [Plot1_LF(4,1:45) Plot2_LF(4,5:51)];
Y_kfe_LF = [Plot1_LF(5,1:45) Plot2_LF(5,5:51)];
Z_kfe_LF = [Plot1_LF(6,1:45) Plot2_LF(6,5:51)];
X_ef_LF =  [Plot1_LF(7,1:45) Plot2_LF(7,5:51)];
Y_ef_LF =  [Plot1_LF(8,1:45) Plot2_LF(8,5:51)];
Z_ef_LF =  [Plot1_LF(9,1:45) Plot2_LF(9,5:51)];

%-------------------Leg(RH)-------------------------------------

initial_pos_1_RH = [Stride_length/2 0 -Height_ground];
final_pos_1_RH = [0 0 -(Height_ground-Stride_height)];
initial_vel_1_RH = [0.3 0 3.4];
Final_vel_1_RH = [0.3 0 0];
initial_pos_2_RH = [0 0 -(Height_ground-Stride_height)];
final_pos_2_RH = [-Stride_length/2 0 -Height_ground];
initial_vel_2_RH = [0.3 0 0];
Final_vel_2_RH = [0.3 0 -3.4];

Plot1_RH = draw("RH",tf/2,L,W,initial_pos_1_RH,final_pos_1_RH,initial_vel_1_RH,Final_vel_1_RH);
Plot2_RH = draw("RH",tf/2,L,W,initial_pos_2_RH,final_pos_2_RH,initial_vel_2_RH,Final_vel_2_RH);

X_hfe_RH = [Plot1_RH(1,1:45) Plot2_RH(1,5:51)];
Y_hfe_RH = [Plot1_RH(2,1:45) Plot2_RH(2,5:51)];
Z_hfe_RH = [Plot1_RH(3,1:45) Plot2_RH(3,5:51)];
X_kfe_RH = [Plot1_RH(4,1:45) Plot2_RH(4,5:51)];
Y_kfe_RH = [Plot1_RH(5,1:45) Plot2_RH(5,5:51)];
Z_kfe_RH = [Plot1_RH(6,1:45) Plot2_RH(6,5:51)];
X_ef_RH =  [Plot1_RH(7,1:45) Plot2_RH(7,5:51)];
Y_ef_RH =  [Plot1_RH(8,1:45) Plot2_RH(8,5:51)];
Z_ef_RH =  [Plot1_RH(9,1:45) Plot2_RH(9,5:51)];

%-------------------Leg(RF)-------------------------------------

initial_pos_1_RF = [Stride_length/2 0 -Height_ground];
final_pos_1_RF = [0 0 -(Height_ground-Stride_height)];
initial_vel_1_RF = [0.3 0 3.4];
Final_vel_1_RF = [0.3 0 0];
initial_pos_2_RF = [0 0 -(Height_ground-Stride_height)];
final_pos_2_RF = [-Stride_length/2 0 -Height_ground];
initial_vel_2_RF = [0.3 0 0];
Final_vel_2_RF = [0.3 0 -3.4];

Plot1_RF = draw("RF",tf/2,L,W,initial_pos_1_RF,final_pos_1_RF,initial_vel_1_RF,Final_vel_1_RF);
Plot2_RF = draw("RF",tf/2,L,W,initial_pos_2_RF,final_pos_2_RF,initial_vel_2_RF,Final_vel_2_RF);

X_hfe_RF = [Plot1_RF(1,1:45) Plot2_RF(1,5:51)];
Y_hfe_RF = [Plot1_RF(2,1:45) Plot2_RF(2,5:51)];
Z_hfe_RF = [Plot1_RF(3,1:45) Plot2_RF(3,5:51)];
X_kfe_RF = [Plot1_RF(4,1:45) Plot2_RF(4,5:51)];
Y_kfe_RF = [Plot1_RF(5,1:45) Plot2_RF(5,5:51)];
Z_kfe_RF = [Plot1_RF(6,1:45) Plot2_RF(6,5:51)];
X_ef_RF=   [Plot1_RF(7,1:45) Plot2_RF(7,5:51)];
Y_ef_RF =  [Plot1_RF(8,1:45) Plot2_RF(8,5:51)];
Z_ef_RF =  [Plot1_RF(9,1:45) Plot2_RF(9,5:51)];

%-------------------Leg(LH)-------------------------------------

initial_pos_1_LH = [-Stride_length/2 0 -Height_ground];
final_pos_1_LH = [0 0 -(Height_ground-Stride_height)];
initial_vel_1_LH = [0.3 0 3.4];
Final_vel_1_LH = [0.3 0 0];
initial_pos_2_LH = [0 0 -(Height_ground-Stride_height)];
final_pos_2_LH = [Stride_length/2 0 -Height_ground];
initial_vel_2_LH = [0.3 0 0];
Final_vel_2_LH = [0.3 0 -3.4];

Plot1_LH = draw("LH",tf/2,L,W,initial_pos_1_LH,final_pos_1_LH,initial_vel_1_LH,Final_vel_1_LH);
Plot2_LH = draw("LH",tf/2,L,W,initial_pos_2_LH,final_pos_2_LH,initial_vel_2_LH,Final_vel_2_LH);

X_hfe_LH = [Plot1_LH(1,1:45) Plot2_LH(1,5:51)];
Y_hfe_LH = [Plot1_LH(2,1:45) Plot2_LH(2,5:51)];
Z_hfe_LH = [Plot1_LH(3,1:45) Plot2_LH(3,5:51)];
X_kfe_LH = [Plot1_LH(4,1:45) Plot2_LH(4,5:51)];
Y_kfe_LH = [Plot1_LH(5,1:45) Plot2_LH(5,5:51)];
Z_kfe_LH = [Plot1_LH(6,1:45) Plot2_LH(6,5:51)];
X_ef_LH =  [Plot1_LH(7,1:45) Plot2_LH(7,5:51)];
Y_ef_LH =  [Plot1_LH(8,1:45) Plot2_LH(8,5:51)];
Z_ef_LH =  [Plot1_LH(9,1:45) Plot2_LH(9,5:51)];


figure

% plot3([0 L/2], [0 W/2],[0 0],'b',LineWidth=3)
% hold on

for i = 1:length(Z_ef_LF)

    %-------------------Body Frame----------------

    B0 = [0 0 0;
          0 0 0;];

    B1 = [L/2 W/2 0;
          L/2 -W/2 0];

    B3 = [-L/2 W/2 0;
          -L/2 -W/2 0;];

    Xf = [B0(:,1) B1(:,1)];
    Yf = [B0(:,2) B1(:,2)];
    Zf = [B0(:,3) B1(:,3)];

    Xb = [B0(:,1) B3(:,1)];
    Yb = [B0(:,2) B3(:,2)];
    Zb = [B0(:,3) B3(:,3)];
    
    plot3(Xf',Yf',Zf','K',LineWidth=3) 
    hold on
    plot3(Xb',Yb',Zb','b',LineWidth=3)
    hold on
    %-------------------------Leg(RF)------------------------------
    Angle_Rf = InverseK("RF",L1,L2,L3,Stride_length/2,0,-Height_ground);
    Rf0 = [L/2,-W/2, 0;
           L/2, -W/2+L1*sin(Angle_Rf(1)), -L1*cos(Angle_Rf(1));
           L/2+L2*sin(Angle_Rf(2)), -W/2+L1*sin(Angle_Rf(1))+L2*cos(Angle_Rf(2))*sin(Angle_Rf(1)), -L1*cos(Angle_Rf(1))-L2*cos(Angle_Rf(1))*cos(Angle_Rf(2));];

    Rf1 = [L/2, -W/2+L1*sin(Angle_Rf(1)), -L1*cos(Angle_Rf(1));
           L/2+L2*sin(Angle_Rf(2)), -W/2+L1*sin(Angle_Rf(1))+L2*cos(Angle_Rf(2))*sin(Angle_Rf(1)), -L1*cos(Angle_Rf(1))-L2*cos(Angle_Rf(1))*cos(Angle_Rf(2));
           L/2+L2*sin(Angle_Rf(2))+L3*sin(Angle_Rf(2)+Angle_Rf(3)), -W/2+sin(Angle_Rf(1))*(L1 + L2*cos(Angle_Rf(2)) + L3*cos(Angle_Rf(2) + Angle_Rf(3))), -cos(Angle_Rf(1))*(L1 + L2*cos(Angle_Rf(2)) + L3*cos(Angle_Rf(2) + Angle_Rf(3)))];

    X_RF = round([Rf0(:,1) Rf1(:,1)],5);
    Y_RF = round([Rf0(:,2) Rf1(:,2)],5);
    Z_RF = round([Rf0(:,3) Rf1(:,3)],5);

    plot3(X_RF',Y_RF',Z_RF','c',LineWidth=3) 
    hold on

    %------------------------------Leg(LH)---------------------------------

    Angle_Lh = InverseK("LH",L1,L2,L3,-Stride_length/2,0,-Height_ground);
    Lh0 = [-L/2,W/2, 0;
           -L/2, W/2+L1*sin(Angle_Lh(1)), -L1*cos(Angle_Lh(1));
           -L/2-L2*sin(Angle_Lh(2)), W/2+L1*sin(Angle_Lh(1))+L2*cos(Angle_Lh(2))*sin(Angle_Lh(1)), -L1*cos(Angle_Lh(1))-L2*cos(Angle_Lh(1))*cos(Angle_Lh(2));];

    Lh1 = [-L/2, W/2+L1*sin(Angle_Lh(1)), -L1*cos(Angle_Lh(1));
           -L/2-L2*sin(Angle_Lh(2)), W/2+L1*sin(Angle_Lh(1))+L2*cos(Angle_Lh(2))*sin(Angle_Lh(1)), -L1*cos(Angle_Lh(1))-L2*cos(Angle_Lh(1))*cos(Angle_Lh(2));
           -L/2-L2*sin(Angle_Lh(2))-L3*sin(Angle_Lh(2)+Angle_Lh(3)), W/2+sin(Angle_Lh(1))*(L1 + L2*cos(Angle_Lh(2)) + L3*cos(Angle_Lh(2) + Angle_Lh(3))), -cos(Angle_Lh(1))*(L1 + L2*cos(Angle_Lh(2)) + L3*cos(Angle_Lh(2) + Angle_Lh(3)));];

    X_LH = round([Lh0(:,1) Lh1(:,1)],5);
    Y_LH = round([Lh0(:,2) Lh1(:,2)],5);
    Z_LH = round([Lh0(:,3) Lh1(:,3)],5);

    plot3(X_LH',Y_LH',Z_LH','c',LineWidth=3) 
    hold on

    
    %------------------------Leg(LF)---------------------------------
    P0_LF = [L/2 W/2 0;
            X_hfe_LF(i) Y_hfe_LF(i) Z_hfe_LF(i);
            X_kfe_LF(i) Y_kfe_LF(i) Z_kfe_LF(i);];

    P1_LF = [X_hfe_LF(i) Y_hfe_LF(i) Z_hfe_LF(i);
             X_kfe_LF(i) Y_kfe_LF(i) Z_kfe_LF(i);
             X_ef_LF(i) Y_ef_LF(i) Z_ef_LF(i);];

    X_LF = [P0_LF(:,1) P1_LF(:,1)];
    Y_LF = [P0_LF(:,2) P1_LF(:,2)];
    Z_LF = [P0_LF(:,3) P1_LF(:,3)];
    plot3(X_LF',Y_LF',Z_LF','r',LineWidth= 3)
    grid on
    plot3(X_ef_LF(1:i),Y_ef_LF(1:i), Z_ef_LF(1:i),'k',LineWidth=1)
    %-----------------------Leg(RH)---------------------------------
    P0_RH = [-L/2 -W/2 0;
            X_hfe_RH(i) Y_hfe_RH(i) Z_hfe_RH(i);
            X_kfe_RH(i) Y_kfe_RH(i) Z_kfe_RH(i);];

    P1_RH = [X_hfe_RH(i) Y_hfe_RH(i) Z_hfe_RH(i);
             X_kfe_RH(i) Y_kfe_RH(i) Z_kfe_RH(i);
             X_ef_RH(i) Y_ef_RH(i) Z_ef_RH(i);];

    X_RH = [P0_RH(:,1) P1_RH(:,1)];
    Y_RH = [P0_RH(:,2) P1_RH(:,2)];
    Z_RH = [P0_RH(:,3) P1_RH(:,3)];
    plot3(X_RH',Y_RH',Z_RH','r',LineWidth= 3)
    grid on
    plot3(X_ef_RH(1:i),Y_ef_RH(1:i), Z_ef_RH(1:i),'k',LineWidth=1)
    view(165,30)
    axis([-(L/2+Stride_length) (L/2+Stride_length)  -(W/2+Stride_length) (W/2+Stride_length) -(L1 +L2 +L3 +10) 20])
    pause(0.001)
    hold off
end


for i = 1:length(Z_ef_LH)

    %-------------------Body Frame----------------

    B0 = [0 0 0;
          0 0 0;];

    B1 = [L/2 W/2 0;
          L/2 -W/2 0];

    B3 = [-L/2 W/2 0;
          -L/2 -W/2 0;];

    Xf = [B0(:,1) B1(:,1)];
    Yf = [B0(:,2) B1(:,2)];
    Zf = [B0(:,3) B1(:,3)];

    Xb = [B0(:,1) B3(:,1)];
    Yb = [B0(:,2) B3(:,2)];
    Zb = [B0(:,3) B3(:,3)];
    
    plot3(Xf',Yf',Zf','K',LineWidth=3) 
    hold on
    plot3(Xb',Yb',Zb','b',LineWidth=3)
    hold on
    %-------------------------Leg(RH)------------------------------
%     Angle_Rf = InverseK("RF",L1,L2,L3,Stride_length/2,0,-Height_ground);
%     Rf0 = [L/2,-W/2, 0;
%            L/2, -W/2+L1*sin(Angle_Rf(1)), -L1*cos(Angle_Rf(1));
%            L/2+L2*sin(Angle_Rf(2)), -W/2+L1*sin(Angle_Rf(1))+L2*cos(Angle_Rf(2))*sin(Angle_Rf(1)), -L1*cos(Angle_Rf(1))-L2*cos(Angle_Rf(1))*cos(Angle_Rf(2));];
% 
%     Rf1 = [L/2, -W/2+L1*sin(Angle_Rf(1)), -L1*cos(Angle_Rf(1));
%            L/2+L2*sin(Angle_Rf(2)), -W/2+L1*sin(Angle_Rf(1))+L2*cos(Angle_Rf(2))*sin(Angle_Rf(1)), -L1*cos(Angle_Rf(1))-L2*cos(Angle_Rf(1))*cos(Angle_Rf(2));
%            L/2+L2*sin(Angle_Rf(2))+L3*sin(Angle_Rf(2)+Angle_Rf(3)), -W/2+sin(Angle_Rf(1))*(L1 + L2*cos(Angle_Rf(2)) + L3*cos(Angle_Rf(2) + Angle_Rf(3))), -cos(Angle_Rf(1))*(L1 + L2*cos(Angle_Rf(2)) + L3*cos(Angle_Rf(2) + Angle_Rf(3)))];

%     X_RF = round([Rf0(:,1) Rf1(:,1)],5);
%     Y_RF = round([Rf0(:,2) Rf1(:,2)],5);
%     Z_RF = round([Rf0(:,3) Rf1(:,3)],5);

    plot3(X_RH',Y_RH',Z_RH','r',LineWidth=3) 
    hold on

    %------------------------------Leg(LF)---------------------------------

%     Angle_Lh = InverseK("LH",L1,L2,L3,-Stride_length/2,0,-Height_ground);
%     Lh0 = [-L/2,W/2, 0;
%            -L/2, W/2+L1*sin(Angle_Lh(1)), -L1*cos(Angle_Lh(1));
%            -L/2-L2*sin(Angle_Lh(2)), W/2+L1*sin(Angle_Lh(1))+L2*cos(Angle_Lh(2))*sin(Angle_Lh(1)), -L1*cos(Angle_Lh(1))-L2*cos(Angle_Lh(1))*cos(Angle_Lh(2));];
% 
%     Lh1 = [-L/2, W/2+L1*sin(Angle_Lh(1)), -L1*cos(Angle_Lh(1));
%            -L/2-L2*sin(Angle_Lh(2)), W/2+L1*sin(Angle_Lh(1))+L2*cos(Angle_Lh(2))*sin(Angle_Lh(1)), -L1*cos(Angle_Lh(1))-L2*cos(Angle_Lh(1))*cos(Angle_Lh(2));
%            -L/2-L2*sin(Angle_Lh(2))-L3*sin(Angle_Lh(2)+Angle_Lh(3)), W/2+sin(Angle_Lh(1))*(L1 + L2*cos(Angle_Lh(2)) + L3*cos(Angle_Lh(2) + Angle_Lh(3))), -cos(Angle_Lh(1))*(L1 + L2*cos(Angle_Lh(2)) + L3*cos(Angle_Lh(2) + Angle_Lh(3)));];

%     X_LH = round([Lh0(:,1) Lh1(:,1)],5);
%     Y_LH = round([Lh0(:,2) Lh1(:,2)],5);
%     Z_LH = round([Lh0(:,3) Lh1(:,3)],5);

    plot3(X_LF',Y_LF',Z_LF','r',LineWidth=3) 
    hold on

    
    %------------------------Leg(LH)---------------------------------
    P0_LH = [-L/2 W/2 0;
            X_hfe_LH(i) Y_hfe_LH(i) Z_hfe_LH(i);
            X_kfe_LH(i) Y_kfe_LH(i) Z_kfe_LH(i);];

    P1_LH = [X_hfe_LH(i) Y_hfe_LH(i) Z_hfe_LH(i);
             X_kfe_LH(i) Y_kfe_LH(i) Z_kfe_LH(i);
             X_ef_LH(i) Y_ef_LH(i) Z_ef_LH(i);];

    X_LH = [P0_LH(:,1) P1_LH(:,1)];
    Y_LH = [P0_LH(:,2) P1_LH(:,2)];
    Z_LH = [P0_LH(:,3) P1_LH(:,3)];
    plot3(X_LH',Y_LH',Z_LH','c',LineWidth= 3)
    grid on
    plot3(X_ef_LH(1:i),Y_ef_LH(1:i), Z_ef_LH(1:i),'k',LineWidth=1)
    %-----------------------Leg(RF)---------------------------------
    P0_RF = [L/2 -W/2 0;
            X_hfe_RF(i) Y_hfe_RF(i) Z_hfe_RF(i);
            X_kfe_RF(i) Y_kfe_RF(i) Z_kfe_RF(i);];

    P1_RF = [X_hfe_RF(i) Y_hfe_RF(i) Z_hfe_RF(i);
             X_kfe_RF(i) Y_kfe_RF(i) Z_kfe_RF(i);
             X_ef_RF(i) Y_ef_RF(i) Z_ef_RF(i);];

    X_RF = [P0_RF(:,1) P1_RF(:,1)];
    Y_RF = [P0_RF(:,2) P1_RF(:,2)];
    Z_RF = [P0_RF(:,3) P1_RF(:,3)];
    plot3(X_RF',Y_RF',Z_RF','c',LineWidth= 3)
    grid on
    plot3(X_ef_RF(1:i),Y_ef_RF(1:i), Z_ef_RF(1:i),'k',LineWidth=1)
    view(165,30)
    axis([-(L/2+Stride_length) (L/2+Stride_length)  -(W/2+Stride_length) (W/2+Stride_length) -(L1 +L2 +L3 +10) 20])
    pause(0.001)
    hold off
end


