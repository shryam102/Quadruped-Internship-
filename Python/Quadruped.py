import matplotlib.pyplot as plt
from numpy import *

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#------------------------------------------Defining Functions --------------------------------------------------------

def Tranform(R,Q):
    v = [0,0,0,1]
    Matrix = []
    for i in range(3):
        x = list(R[i])
        x.append(Q[i])
        Matrix.append(x)
    Matrix.append(v)
    return Matrix

def EulerAngle_Rotation(Rx,Ry,Rz,X):
    X = list(X)
    Dict = {'Z':Rz,'Y':Ry,'X':Rx}
    val1 = dot(Dict[X[0]],Dict[X[1]])
    Rotation_matrix = dot(val1,Dict[X[2]])
    return Rotation_matrix
    
def Rx(theta):
    return [[1,0,0],
                     [0,round(cos(theta),3),-round(sin(theta),3)],
                     [0,round(sin(theta),3),round(cos(theta),3)]]
def Ry(phi):
    return [[round(cos(phi),3),0,round(sin(phi),3)],
                     [0,1,0],
                     [-round(sin(phi),3),0,round(cos(phi),3)]]
def Rz(si):
    return [[round(cos(si),3),-round(sin(si),3),0],
                     [round(sin(si),3),round(cos(si),3),0],
                     [0,0,1]]

def Tranform_leg_joints(Matrix,T,N):
    if N == 1:
        final = Matrix[0]
        final = dot(T,final)
    else:
        final = Matrix[0]
        for i in range(1,N):
            final = dot(final,Matrix[i])
        final = dot(T,final)
    return final

def dh(theta,d,a,alpha):
    Matrix = []
    for i in range(len(theta)):
        x =[[round(cos(theta[i]),3), -round(sin(theta[i])*cos(alpha[i]),3), round(sin(theta[i])*sin(alpha[i]),3), round(a[i]*cos(theta[i]),3)],
            [round(sin(theta[i]),3), round(cos(theta[i])*cos(alpha[i]),3), -round(cos(theta[i])*sin(alpha[i]),3), round(a[i]*sin(theta[i]),3)],
            [0, round(sin(alpha[i]),3), round(cos(alpha[i]),3), d[i]],
            [0, 0, 0, 1]]
        Matrix.append(x)
    return Matrix
#--------------------------------------------End---------------------------------------------------------------

#-----------------------------------------Inertial Frame to Body Frame------------------------------------------

Alpha = 0                                                           # Angle rotation about X axis of body frame w.r.t to inertial frame
Beta = 0                                                            # Angle rotation about Y axis of body frame w.r.t to inertial frame              
Gamma = 0                                                           # Angle rotation about Z axis of body frame w.r.t to inertial frame

Ri_b = EulerAngle_Rotation(Rz(Gamma),Ry(Beta),Rx(Alpha),"ZYX")      #Rotation Matrix of Body frame wrt Inertial Frame
L = 100                                                             #Lenght of the middle plate
W = 50                                                              #Width of the middle plate  
qx = 0                                                              # x displacement of body frame w.r.t inertial frame
qy = 0                                                              # y displacement of body frame w.r.t inertial frame
qz = 0                                                              # z displacement of body frame w.r.t inertial frame
Bv = [qx, qy, qz]                                                   # Vector displacement of Body frame w.r.t interial frame (qx,qy,qz)  

Ti_b = Tranform(Ri_b,Bv)                                            #Tranformation matrix of body frame wrt inertial frame

Ti_b_matrix = matrix(Ti_b)
# print(Ti_b_matrix)                                                (Uncomment to see the Tranformation matrix of body frame wrt inertial frame )

link1 = 10                                                          # |                                        |
link2 = 30                                                          # |       Lenghts of Different Links       |
link3 = 30                                                          # |                                        |

#----------------------------------------------Leg 1 (LF)------------------------------------------------------------

VB_LF_HAA = [L/2,W/2,0]                              #Hip joint1 vector 
Rb_LF_HAA = Ry(pi/2)                                 #Hip joint1 rotation wrt body frame
Tb_LF_HAA = Tranform(Rb_LF_HAA,VB_LF_HAA)            #Tranformation matrix of Hip joint1 wrt body frame
Ti_LF_HAA = dot(Ti_b,Tb_LF_HAA)                      #Tranformation matrix of Hip joint1 wrt interial frame  

qLF_HAA = 00                                       # Rotation about Hip abduction
qLF_HFE = pi/5                                     # Rotation about Hip extension
qLF_KFE = -pi/4                                    # Rotation about Knee flexion

Theta_1 = [qLF_HAA,qLF_HFE,qLF_KFE]                  #|                               |
D_1 = [0,0,0]                                        #|     Dh_parameters table       |
A_1 = [link1,link2,link3]                            #|                               |
Alpha_1 = [-pi/2,0,0]                                #|                               |
Matrix_1 = dh(Theta_1,D_1,A_1,Alpha_1)

Ti_LF_HFE = Tranform_leg_joints(Matrix_1,Ti_LF_HAA,1)
Ti_LF_KFE = Tranform_leg_joints(Matrix_1,Ti_LF_HAA,2)
Ti_LF_EF = Tranform_leg_joints(Matrix_1,Ti_LF_HAA,3)             #EF : End effector

Ti_LF_EF_matrix = matrix(Ti_LF_EF)
#print(Ti_LF_EF_matrix)                                          #Tranformation matrix of end effector w.r.t inertial frame

#----------------------------------------------------------Plotting---------------------------------------------------------------------

#Vector1
ax.quiver(Bv[0],Bv[1],Bv[2],Ti_LF_HAA[0][3]-Bv[0],Ti_LF_HAA[1][3]-Bv[1],Ti_LF_HAA[2][3]-Bv[2], color = 'r', arrow_length_ratio=0)
#Vector2
ax.quiver(Ti_LF_HAA[0][3],Ti_LF_HAA[1][3],Ti_LF_HAA[2][3],Ti_LF_HFE[0][3]-Ti_LF_HAA[0][3],Ti_LF_HFE[1][3]-Ti_LF_HAA[1][3],Ti_LF_HFE[2][3]-Ti_LF_HAA[2][3], color = 'b', arrow_length_ratio=0)
#Vector3
ax.quiver(Ti_LF_HFE[0][3],Ti_LF_HFE[1][3],Ti_LF_HFE[2][3],Ti_LF_KFE[0][3]-Ti_LF_HFE[0][3],Ti_LF_KFE[1][3]-Ti_LF_HFE[1][3],Ti_LF_KFE[2][3]-Ti_LF_HFE[2][3], color = 'b', arrow_length_ratio=0)
#Vector4
ax.quiver(Ti_LF_KFE[0][3],Ti_LF_KFE[1][3],Ti_LF_KFE[2][3],Ti_LF_EF[0][3]-Ti_LF_KFE[0][3],Ti_LF_EF[1][3]-Ti_LF_KFE[1][3],Ti_LF_EF[2][3]-Ti_LF_KFE[2][3], color = 'b', arrow_length_ratio=0)

#----------------------------------------------Leg 2(RF)------------------------------------------------------------

Vb_RF_HAA = [L/2,-W/2,0]                              #Hip joint1 vector 
Rb_RF_HAA = Ry(pi/2)                                  #Hip joint1 rotation wrt body frame
Tb_RF_HAA = Tranform(Rb_RF_HAA,Vb_RF_HAA)             #Tranformation matrix of Hip joint1 wrt body frame
Ti_RF_HAA = dot(Ti_b,Tb_RF_HAA)                       #Tranformation matrix of Hip joint1 wrt interial frame  

qRF_HAA = 0
qRF_HFE = -pi/5
qRF_KFE = pi/4

Theta_2 = [qRF_HAA,qRF_HFE,qRF_KFE]
D_2 = [0,0,0]
A_2 = [link1,link2,link3]
Alpha_2 = [pi/2,0,0]
Matrix_2 = dh(Theta_2,D_2,A_2,Alpha_2)

Ti_RF_HFE = Tranform_leg_joints(Matrix_2,Ti_RF_HAA,1)
Ti_RF_KFE = Tranform_leg_joints(Matrix_2,Ti_RF_HAA,2)
Ti_RF_EF = Tranform_leg_joints(Matrix_2,Ti_RF_HAA,3)

Ti_RF_EF_matrix = matrix(Ti_RF_EF)
#print(Ti_RF_EF_matrix)                                              #Tranformation matrix of end effector w.r.t inertial frame                                         

#----------------------------------------------------------Plotting---------------------------------------------------------------------

#Vector5
ax.quiver(Bv[0],Bv[1],Bv[2],Ti_RF_HAA[0][3]-Bv[0],Ti_RF_HAA[1][3]-Bv[1],Ti_RF_HAA[2][3]-Bv[2], color = 'r', arrow_length_ratio=0)
#Vector6
ax.quiver(Ti_RF_HAA[0][3],Ti_RF_HAA[1][3],Ti_RF_HAA[2][3],Ti_RF_HFE[0][3]-Ti_RF_HAA[0][3],Ti_RF_HFE[1][3]-Ti_RF_HAA[1][3],Ti_RF_HFE[2][3]-Ti_RF_HAA[2][3], color = 'b', arrow_length_ratio=0)
#Vector7
ax.quiver(Ti_RF_HFE[0][3],Ti_RF_HFE[1][3],Ti_RF_HFE[2][3],Ti_RF_KFE[0][3]-Ti_RF_HFE[0][3],Ti_RF_KFE[1][3]-Ti_RF_HFE[1][3],Ti_RF_KFE[2][3]-Ti_RF_HFE[2][3], color = 'b', arrow_length_ratio=0)
#Vector8
ax.quiver(Ti_RF_KFE[0][3],Ti_RF_KFE[1][3],Ti_RF_KFE[2][3],Ti_RF_EF[0][3]-Ti_RF_KFE[0][3],Ti_RF_EF[1][3]-Ti_RF_KFE[1][3],Ti_RF_EF[2][3]-Ti_RF_KFE[2][3], color = 'b', arrow_length_ratio=0)


#----------------------------------------------Leg 3(LH)------------------------------------------------------------

Vb_LH_HAA = [-L/2,W/2,0]                              #Hip joint1 vector 
Rb_LH_HAA = Ry(pi/2)                                  #Hip joint1 rotation wrt body frame
Tb_LH_HAA = Tranform(Rb_LH_HAA,Vb_LH_HAA)             #Tranformation matrix of Hip joint1 wrt body frame
Ti_LH_HAA = dot(Ti_b,Tb_LH_HAA)                       #Tranformation matrix of Hip joint1 wrt interial frame  

qLH_HAA = 0
qLH_HFE = -pi/5
qLH_KFE = pi/4

Theta_3 = [qLH_HAA,qLH_HFE,qLH_KFE]
D_3 = [0,0,0]
A_3 = [link1,link2,link3]
Alpha_3 = [-pi/2,0,0]
Matrix_3 = dh(Theta_3,D_3,A_3,Alpha_3)

Ti_LH_HFE = Tranform_leg_joints(Matrix_3,Ti_LH_HAA,1)
Ti_LH_KFE = Tranform_leg_joints(Matrix_3,Ti_LH_HAA,2)
Ti_LH_EF = Tranform_leg_joints(Matrix_3,Ti_LH_HAA,3)

Ti_LH_EF_matrix = matrix(Ti_LH_EF)
#print(Ti_LH_EF_matrix)                                              #Tranformation matrix of end effector w.r.t inertial frame

#----------------------------------------------------------Plotting---------------------------------------------------------------------

#Vector9
ax.quiver(Bv[0],Bv[1],Bv[2],Ti_LH_HAA[0][3]-Bv[0],Ti_LH_HAA[1][3]-Bv[1],Ti_LH_HAA[2][3]-Bv[2], color = 'r', arrow_length_ratio=0)
#Vector10
ax.quiver(Ti_LH_HAA[0][3],Ti_LH_HAA[1][3],Ti_LH_HAA[2][3],Ti_LH_HFE[0][3]-Ti_LH_HAA[0][3],Ti_LH_HFE[1][3]-Ti_LH_HAA[1][3],Ti_LH_HFE[2][3]-Ti_LH_HAA[2][3], color = 'r', arrow_length_ratio=0)
#Vector11
ax.quiver(Ti_LH_HFE[0][3],Ti_LH_HFE[1][3],Ti_LH_HFE[2][3],Ti_LH_KFE[0][3]-Ti_LH_HFE[0][3],Ti_LH_KFE[1][3]-Ti_LH_HFE[1][3],Ti_LH_KFE[2][3]-Ti_LH_HFE[2][3], color = 'r', arrow_length_ratio=0)
#Vector12
ax.quiver(Ti_LH_KFE[0][3],Ti_LH_KFE[1][3],Ti_LH_KFE[2][3],Ti_LH_EF[0][3]-Ti_LH_KFE[0][3],Ti_LH_EF[1][3]-Ti_LH_KFE[1][3],Ti_LH_EF[2][3]-Ti_LH_KFE[2][3], color = 'r', arrow_length_ratio=0)


#----------------------------------------------Leg 4(RH)------------------------------------------------------------

Vb_RH_HAA = [-L/2,-W/2,0]                              #Hip joint1 vector 
Rb_RH_HAA = Ry(pi/2)                                   #Hip joint1 rotation wrt body frame
Tb_RH_HAA = Tranform(Rb_RH_HAA,Vb_RH_HAA)              #Tranformation matrix of Hip joint1 wrt body frame
Ti_RH_HAA = dot(Ti_b,Tb_RH_HAA)                        #Tranformation matrix of Hip joint1 wrt interial frame  

qRH_HAA = 0
qRH_HFE = pi/5
qRH_KFE = -pi/4

Theta_4 = [qRH_HAA,qRH_HFE,qRH_KFE]
D_4 = [0,0,0]
A_4 = [link1,link2,link3]
Alpha_4 = [pi/2,0,0]
Matrix_4 = dh(Theta_4,D_4,A_4,Alpha_4)

Ti_RH_HFE = Tranform_leg_joints(Matrix_4,Ti_RH_HAA,1)
Ti_RH_KFE = Tranform_leg_joints(Matrix_4,Ti_RH_HAA,2)
Ti_RH_EF = Tranform_leg_joints(Matrix_4,Ti_RH_HAA,3)

Ti_RH_EF_matrix = matrix(Ti_RH_EF)
print(Ti_RH_EF_matrix)                                              #Tranformation matrix of end effector w.r.t inertial frame

#----------------------------------------------------------Plotting---------------------------------------------------------------------

#Vector13
ax.quiver(Bv[0],Bv[1],Bv[2],Ti_RH_HAA[0][3]-Bv[0],Ti_RH_HAA[1][3]-Bv[1],Ti_RH_HAA[2][3]-Bv[2], color = 'r', arrow_length_ratio=0)
#Vector14
ax.quiver(Ti_RH_HAA[0][3],Ti_RH_HAA[1][3],Ti_RH_HAA[2][3],Ti_RH_HFE[0][3]-Ti_RH_HAA[0][3],Ti_RH_HFE[1][3]-Ti_RH_HAA[1][3],Ti_RH_HFE[2][3]-Ti_RH_HAA[2][3], color = 'r', arrow_length_ratio=0)
#Vector15
ax.quiver(Ti_RH_HFE[0][3],Ti_RH_HFE[1][3],Ti_RH_HFE[2][3],Ti_RH_KFE[0][3]-Ti_RH_HFE[0][3],Ti_RH_KFE[1][3]-Ti_RH_HFE[1][3],Ti_RH_KFE[2][3]-Ti_RH_HFE[2][3], color = 'r', arrow_length_ratio=0)
#Vector16
ax.quiver(Ti_RH_KFE[0][3],Ti_RH_KFE[1][3],Ti_RH_KFE[2][3],Ti_RH_EF[0][3]-Ti_RH_KFE[0][3],Ti_RH_EF[1][3]-Ti_RH_KFE[1][3],Ti_RH_EF[2][3]-Ti_RH_KFE[2][3], color = 'r', arrow_length_ratio=0)


 
#Vector17
ax.quiver(Ti_LF_HAA[0][3],Ti_LF_HAA[1][3],Ti_LF_HAA[2][3],Ti_LH_HAA[0][3]-Ti_LF_HAA[0][3],Ti_LH_HAA[1][3]-Ti_LF_HAA[1][3],Ti_LH_HAA[2][3]-Ti_LF_HAA[2][3],color = 'black',arrow_length_ratio=0)
#Vector18
ax.quiver(Ti_LH_HAA[0][3],Ti_LH_HAA[1][3],Ti_LH_HAA[2][3],Ti_RH_HAA[0][3]-Ti_LH_HAA[0][3],Ti_RH_HAA[1][3]-Ti_LH_HAA[1][3],Ti_RH_HAA[2][3]-Ti_LH_HAA[2][3],color = 'black',arrow_length_ratio=0)
#Vector19
ax.quiver(Ti_RH_HAA[0][3],Ti_RH_HAA[1][3],Ti_RH_HAA[2][3],Ti_RF_HAA[0][3]-Ti_RH_HAA[0][3],Ti_RF_HAA[1][3]-Ti_RH_HAA[1][3],Ti_RF_HAA[2][3]-Ti_RH_HAA[2][3],color = 'black',arrow_length_ratio=0)
#Vector20
ax.quiver(Ti_RF_HAA[0][3],Ti_RF_HAA[1][3],Ti_RF_HAA[2][3],Ti_LF_HAA[0][3]-Ti_RF_HAA[0][3],Ti_LF_HAA[1][3]-Ti_RF_HAA[1][3],Ti_LF_HAA[2][3]-Ti_RF_HAA[2][3],color = 'black',arrow_length_ratio=0)


#--------------------------------------------------------Defining 3d axes ------------------------------------------------------------
ax.set_xlim([-100, 100])
ax.set_ylim([-100, 100])
ax.set_zlim([-100, 100])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Quadruped')
plt.legend(["Back leg","Front Leg"], )


plt.show()
#----------------------------------------------------------End------------------------------------------------------------------