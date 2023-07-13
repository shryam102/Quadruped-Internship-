% clc;
% clear;close all;
syms L W L1 L2 L3 qLF_HAA qLF_HFE qLF_KFE qRF_HAA qRF_HFE qRF_KFE qLH_HAA qLH_HFE qLH_KFE qRH_HAA qRH_HFE qRH_KFE

Q = [qLF_HAA;qLF_HFE; qLF_KFE; qRF_HAA; qRF_HFE; qRF_KFE; qLH_HAA; qLH_HFE; qLH_KFE; qRH_HAA; qRH_HFE; qRH_KFE;];


%-----------Leg(LF)--------------

V_LF_HAA = [L/2 W/2 0];                                 %Hip joint1 vector 
Rb_LF_HAA = round(Ry(pi/2),3);                          %Hip joint1 rotation wrt body frame
Theta_1 = [qLF_HAA qLF_HFE qLF_KFE];
D_1 = [0 0 0];                                          %DH table parameters
A_1 = [L1 L2 L3];
Alpha_1 = [-pi/2 0 0];

Matrix_LF = cell(length(Theta_1),1);

for i =1:length(Theta_1)
    Matrix_LF{i} = [cos(Theta_1(i)) round(-cos(Alpha_1(i)),3)*sin(Theta_1(i)) round(sin(Alpha_1(i)),3)*sin(Theta_1(i))  A_1(i)*cos(Theta_1(i));
                    sin(Theta_1(i)) round(cos(Alpha_1(i)),3)*cos(Theta_1(i))  round(-sin(Alpha_1(i)),3)*cos(Theta_1(i)) A_1(i)*sin(Theta_1(i));
                      0             round(sin(Alpha_1(i)),3)                  round(cos(Alpha_1(i)),3)                  D_1(i);
                      0                      0                                  0                                       1;];
end
TLF_HAA_LF_HFE = Matrix_LF{1};
TLF_HFE_LF_KFE = Matrix_LF{2};
TLF_KFE_LF_EF = Matrix_LF{3};
TLF_HAA_LF_EF = TLF_HAA_LF_HFE*TLF_HFE_LF_KFE*TLF_KFE_LF_EF;


% Ti_LF_EF = Ti_b*Tb_LF_HAA*TLF_HAA_LF_EF;       %Tranformation Matrix of Leg 1 end effector w.r.t inertial frame

Tb_LF_HAA = Tranformation_matrix(Rb_LF_HAA,V_LF_HAA);
Tb_LF_HFE = Tb_LF_HAA*TLF_HAA_LF_HFE;
Tb_LF_KFE = Tb_LF_HFE*TLF_HFE_LF_KFE; 
Tb_LF_EF = Tb_LF_HAA*TLF_HAA_LF_EF;

Rot_1 = [Tb_LF_HAA(1:3,3) Tb_LF_HFE(1:3,3) Tb_LF_KFE(1:3,3)];

jacobian_1 = sym('A',[6,length(Q)]);            %Jacobian matrix of leg 1

for i = 1:3
    for j =1:length(Q)
        jacobian_1(i,j) = diff(Tb_LF_EF(i,4),Q(j));
    end
end
for i = 1:length(Rot_1)
    jacobian_1(4:6,i) = Rot_1(:,i);
end
jacobian_1(4:6,length(Rot_1)+1:length(Q)) = 0;

% disp(jacobian_1)

%-----------Leg(RF)--------------

V_RF_HAA = [L/2 -W/2 0];
Rb_RF_HAA = round(Ry(pi/2),3);
Theta_2 = [qRF_HAA qRF_HFE qRF_KFE];
D_2 = [0 0 0];
A_2 = [L1 L2 L3];
Alpha_2 = [pi/2 0 0];

Matrix_RF = cell(length(Theta_2),1);

for i =1:length(Theta_2)
    Matrix_RF{i} = [cos(Theta_2(i)) round(-cos(Alpha_2(i)),3)*sin(Theta_2(i)) round(sin(Alpha_2(i)),3)*sin(Theta_2(i))  A_2(i)*cos(Theta_2(i));
                    sin(Theta_2(i)) round(cos(Alpha_2(i)),3)*cos(Theta_2(i))  round(-sin(Alpha_2(i)),3)*cos(Theta_2(i)) A_2(i)*sin(Theta_2(i));
                      0             round(sin(Alpha_2(i)),3)                  round(cos(Alpha_2(i)),3)                  D_2(i);
                      0                      0                                  0                                       1;];
end
TRF_HAA_RF_HFE = Matrix_RF{1};
TRF_HFE_RF_KFE = Matrix_RF{2};
TRF_KFE_RF_EF =  Matrix_RF{3};
TRF_HAA_RF_EF = TRF_HAA_RF_HFE*TRF_HFE_RF_KFE*TRF_KFE_RF_EF;

Tb_RF_HAA = Tranformation_matrix(Rb_RF_HAA,V_RF_HAA); 
Tb_RF_HFE = Tb_RF_HAA*TRF_HAA_RF_HFE;
Tb_RF_KFE = Tb_RF_HFE*TRF_HFE_RF_KFE;
Tb_RF_EF = Tb_RF_HAA*TRF_HAA_RF_EF;

Rot_2 = [Tb_RF_HAA(1:3,3) Tb_RF_HFE(1:3,3) Tb_RF_KFE(1:3,3)];

jacobian_2 = sym('A',[6,length(Q)]);

for i = 1:3
    for j =1:length(Q)
        jacobian_2(i,j) = diff(Tb_RF_EF(i,4),Q(j));
    end
end
for i = 1:length(Rot_2)
    jacobian_2(4:6,i) = Rot_2(:,i);
end
jacobian_2(4:6,length(Rot_2)+1:length(Q)) = 0;

% disp(jacobian_2)


%-----------Leg(LH)--------------

V_LH_HAA = [-L/2 W/2 0];

Rb_LH_HAA = round(Ry(pi/2),3);
Theta_3 = [qLH_HAA qLH_HFE qLH_KFE];
D_3 = [0 0 0];
A_3 = [L1 L2 L3];
Alpha_3 = [-pi/2 0 0];

Matrix_LH = cell(length(Theta_3),1);

for i =1:length(Theta_3)
    Matrix_LH{i} = [cos(Theta_3(i)) round(-cos(Alpha_3(i)),3)*sin(Theta_3(i)) round(sin(Alpha_3(i)),3)*sin(Theta_3(i))  A_3(i)*cos(Theta_3(i));
                    sin(Theta_3(i)) round(cos(Alpha_3(i)),3)*cos(Theta_3(i))  round(-sin(Alpha_3(i)),3)*cos(Theta_3(i)) A_3(i)*sin(Theta_3(i));
                      0             round(sin(Alpha_3(i)),3)                  round(cos(Alpha_3(i)),3)                  D_3(i);
                      0                      0                                  0                                       1;];
end
TLH_HAA_LH_HFE = Matrix_LH{1};
TLH_HFE_LH_KFE = Matrix_LH{2};
TLH_KFE_LH_EF =  Matrix_LH{3};
TLH_HAA_LH_EF = TLH_HAA_LH_HFE*TLH_HFE_LH_KFE*TLH_KFE_LH_EF;

Tb_LH_HAA = Tranformation_matrix(Rb_LH_HAA,V_LH_HAA);
Tb_LH_HFE = Tb_LH_HAA*TLH_HAA_LH_HFE;
Tb_LH_KFE = Tb_LH_HFE*TLH_HFE_LH_KFE; 
Tb_LH_EF = Tb_LH_HAA*TLH_HAA_LH_EF;

Rot_3 = [Tb_LH_HAA(1:3,3) Tb_LH_HFE(1:3,3) Tb_LH_KFE(1:3,3)];

jacobian_3 = sym('A',[6,length(Q)]);

for i = 1:3
    for j =1:length(Q)
        jacobian_3(i,j) = diff(Tb_LH_EF(i,4),Q(j));
    end
end
for i = 1:length(Rot_3)
    jacobian_3(4:6,i) = Rot_3(:,i);
end
jacobian_3(4:6,length(Rot_3)+1:length(Q)) = 0;

% disp(jacobian_3)

%-----------Leg(RH)--------------

V_RH_HAA = [-L/2 -W/2 0];

Rb_RH_HAA = round(Ry(pi/2),3);
Theta_4 = [qRH_HAA qRH_HFE qRH_KFE];
D_4 = [0 0 0];
A_4 = [L1 L2 L3];
Alpha_4 = [pi/2 0 0];

Matrix_RH = cell(length(Theta_4),1);

for i =1:length(Theta_4)
    Matrix_RH{i} = [cos(Theta_4(i)) round(-cos(Alpha_4(i)),3)*sin(Theta_4(i)) round(sin(Alpha_4(i)),3)*sin(Theta_4(i))  A_4(i)*cos(Theta_4(i));
                    sin(Theta_4(i)) round(cos(Alpha_4(i)),3)*cos(Theta_4(i))  round(-sin(Alpha_4(i)),3)*cos(Theta_4(i)) A_4(i)*sin(Theta_4(i));
                      0             round(sin(Alpha_4(i)),3)                  round(cos(Alpha_4(i)),3)                  D_4(i);
                      0                      0                                  0                                       1;];
end
TRH_HAA_RH_HFE = Matrix_RH{1};
TRH_HFE_RH_KFE = Matrix_RH{2};
TRH_KFE_RH_EF =  Matrix_RH{3};
TRH_HAA_RH_EF = TRH_HAA_RH_HFE*TRH_HFE_RH_KFE*TRH_KFE_RH_EF;

Tb_RH_HAA = Tranformation_matrix(Rb_RH_HAA,V_RH_HAA); 
Tb_RH_HFE = Tb_RH_HAA*TRH_HAA_RH_HFE;
Tb_RH_KFE = Tb_RH_HFE*TRH_HFE_RH_KFE;
Tb_RH_EF = Tb_RH_HAA*TRH_HAA_RH_EF;

Rot_4 = [Tb_RH_HAA(1:3,3) Tb_RH_HFE(1:3,3) Tb_RH_KFE(1:3,3)];

jacobian_4 = sym('A',[6,length(Q)]);

for i = 1:3
    for j =1:length(Q)
        jacobian_4(i,j) = diff(Tb_RH_EF(i,4),Q(j));
    end
end
for i = 1:length(Rot_4)
    jacobian_4(4:6,i) = Rot_4(:,i);
end
jacobian_4(4:6,length(Rot_4)+1:length(Q)) = 0;

% disp(jacobian_4)


