# Name - Dev Patel
# Roll No - 18110113

from operator import eq
import numpy as np
from sympy import *


links = int(input("Enter no of links: ")) # User input to get no of links


Q_ddot = [sympify("q{}_ddot".format(j)) for j in range(links)] # Define q" - (from q"(0) to q"(n-1))
Q_dot = [sympify("q{}_dot".format(j)) for j in range(links)]   # Define q' - (from q'(0) to q'(n-1))
Q = [sympify("q{}".format(j)) for j in range(links)]           # Define q - joint angles (from q(0) to q(n-1))

V = sympify(input("enter V: ")) # Get expression of potential in terms of joint angles form user

# Get the D(q) matrix from the user in terms of q
D = eye(links)
for i in range(links):
    for j in range(links):
        D[i,j] = sympify(input("enter element d({},{}): ".format(i,j)))


# Forming the equations of motion 

for k in range(links):
    a=0
    b=0
    for j in range(links):
        a += D[k,j]*Q_ddot[j]
    for i in range(links):
        for j in range(links):
            b += (diff(D[k,j],Q[i]) - 0.5*diff(D[i,j],Q[k])) * Q_dot[i]*Q_dot[j]
    c = diff(V,Q[k])
    T = sympify("T({})".format(k)) # define the torques applied on the links

    lhs = sympify(a+b-c)
    rhs = T

    print("{} = {}".format(lhs,rhs)) # print the equations