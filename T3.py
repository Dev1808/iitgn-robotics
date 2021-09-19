# Name - Dev Patel
# Roll No - 18110113

import numpy as np

links = int(input("Enter no of links:")) # User inputs for number of links

# Take values of DH parameters[theta, a, d, alpha] from user
DH_parameters = np.array([])
for i in range(links):
    if(i==0):
        DH_i = list(map(float, input("DH parameters for link {}:".format(i)).split()))
        DH_parameters = np.array(DH_i).reshape(1,4)
    else:
        DH_i = list(map(float, input("DH parameters for link {}:".format(i)).split()))
        DH_parameters = np.r_[DH_parameters,np.array(DH_i).reshape(1,4)]

print(DH_parameters)

# gives end effector position
def end_Effector_position():
    T = np.eye(4)
    for i in range(links):
        A = np.array([[np.cos(DH_parameters[i,0]), -np.sin(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), np.sin(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.cos(DH_parameters[i,0])],\
                [np.sin(DH_parameters[i,0]), np.cos(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), -np.cos(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.sin(DH_parameters[i,0])],\
                    [0,np.sin(DH_parameters[i,3]),np.cos(DH_parameters[i,3]),DH_parameters[i,2]],\
                        [0,0,0,1]])

        T = np.dot(T,A)
    return(print('x=',T[0,3],'y=',T[1,3],'z=',T[2,3]))

# Get a skew symmetric matrix for a given 3x1 matrix
def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

# Get the jacobian matrix for a robot
def Jacobian():
    T = np.eye(4)
    O = np.array([0,0,0])
    Z = np.array([0,0,1])
    for i in range(links):
        A = np.array([[np.cos(DH_parameters[i,0]), -np.sin(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), np.sin(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.cos(DH_parameters[i,0])],\
                [np.sin(DH_parameters[i,0]), np.cos(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), -np.cos(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.sin(DH_parameters[i,0])],\
                    [0,np.sin(DH_parameters[i,3]),np.cos(DH_parameters[i,3]),DH_parameters[i,2]],\
                        [0,0,0,1]])

        T = np.dot(T,A)
        O = np.c_[O,np.transpose(T[:3,3])]
        Z = np.c_[Z,np.transpose(T[:3,2])]
    print(O)
    print(Z)
    Ans = input("Do you want to add prismatic joints? (y/n): ")
    if(Ans=="n"):
        J = np.r_[np.dot(skew(Z[:,0]),np.transpose(O[:,links]) - np.transpose(O[:,0])), np.transpose(Z[:,0])]
        for i in range(1,links):
            J = np.c_[J,np.r_[np.dot(skew(Z[:,i]),np.transpose(O[:,links]) - np.transpose(O[:,i])), np.transpose(Z[:,i])]]
    else:
        Prismatic = list(map(float, input("Mention the prismatic joints: ").split()))
        P = np.array(Prismatic)
        J = np.r_[np.dot(skew(Z[:,0]),np.transpose(O[:,links]) - np.transpose(O[:,0])), np.transpose(Z[:,0])]
        for i in range(1,links):
            if((i in P) == True):
                J = np.c_[J,np.r_[np.transpose(Z[:,i]),np.transpose([0,0,0])]]
            else:
                J = np.c_[J,np.r_[np.dot(skew(Z[:,i]),np.transpose(O[:,links]) - np.transpose(O[:,i])), np.transpose(Z[:,i])]]
    return(J)

# Gives end effector velocity matrix
def end_effector_velocity():
    J = Jacobian()
    q = list(map(float, input("velocity of joints: ").split()))
    q_dot = np.array(q).reshape(links,1)
    velocity = np.dot(J,q_dot)
    return(velocity)

# print end effector position, velocity and jacobian
j = Jacobian()
print(j)
end_Effector_position()
v = end_effector_velocity()
print(v)