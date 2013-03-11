#!/usr/bin/isympy

from sympy import *
#Matrix, Symbol, Integral, diff, sin, cos, tan, pprint

from sympy import sin, cos, Matrix, trigsimp
from sympy.utilities.codegen import codegen
from sympy.abc import rho, phi
X = Matrix([rho*cos(phi), rho*sin(phi), rho**2])
Y = Matrix([rho, phi])

print "X=",X
print "Y=",Y
print "Test jacobian ..."
J = X.jacobian(Y)
print " = ",J

#---------------------------------------------
#---------------------------------------------
#   Forward Kinematics
#
#  Variables: j17 , j18 , j19 , j20 , j21 , j22
#  Forward Kinematics sympy matrix
j17 , j18 , j19 , j20 , j21 , j22 = symbols('j17 , j18 , j19 , j20 , j21 , j22')

q = Matrix([j17 , j18 , j19 , j20 , j21 , j22])
T=Matrix([
        [(-(-sin(j17)*sin(j18)*sin(j19) + cos(j17)*cos(j19))*sin(j20) + (-sin(j17)*sin(j18)*cos(j19) - sin(j19)*cos(j17))*cos(j20))*sin(j21) + ((-sin(j17)*sin(j18)*sin(j19) + cos(j17)*cos(j19))*cos(j20) + (-sin(j17)*sin(j18)*cos(j19) - sin(j19)*cos(j17))*sin(j20))*cos(j21)  ,  -((-(-sin(j17)*sin(j18)*sin(j19) + cos(j17)*cos(j19))*sin(j20) + (-sin(j17)*sin(j18)*cos(j19) - sin(j19)*cos(j17))*cos(j20))*cos(j21) - ((-sin(j17)*sin(j18)*sin(j19) + cos(j17)*cos(j19))*cos(j20) + (-sin(j17)*sin(j18)*cos(j19) - sin(j19)*cos(j17))*sin(j20))*sin(j21))*sin(j22) - sin(j17)*cos(j18)*cos(j22)  ,  -((-(-sin(j17)*sin(j18)*sin(j19) + cos(j17)*cos(j19))*sin(j20) + (-sin(j17)*sin(j18)*cos(j19) - sin(j19)*cos(j17))*cos(j20))*cos(j21) - ((-sin(j17)*sin(j18)*sin(j19) + cos(j17)*cos(j19))*cos(j20) + (-sin(j17)*sin(j18)*cos(j19) - sin(j19)*cos(j17))*sin(j20))*sin(j21))*cos(j22) + sin(j17)*sin(j22)*cos(j18)  ,  -211*(-sin(j17)*sin(j18)*sin(j19) + cos(j17)*cos(j19))*sin(j20)/500 + 211*(-sin(j17)*sin(j18)*cos(j19) - sin(j19)*cos(j17))*cos(j20)/500 + sin(j17)*sin(j18)*sin(j19)/20 - 187*sin(j17)*sin(j18)*cos(j19)/500 - sin(j17)*sin(j18)/20 - 187*sin(j19)*cos(j17)/500 - cos(j17)*cos(j19)/20 + cos(j17)/20],
        [((-sin(j17)*sin(j19) + sin(j18)*cos(j17)*cos(j19))*sin(j20) + (sin(j17)*cos(j19) + sin(j18)*sin(j19)*cos(j17))*cos(j20))*cos(j21) + ((-sin(j17)*sin(j19) + sin(j18)*cos(j17)*cos(j19))*cos(j20) - (sin(j17)*cos(j19) + sin(j18)*sin(j19)*cos(j17))*sin(j20))*sin(j21)  ,  -(-((-sin(j17)*sin(j19) + sin(j18)*cos(j17)*cos(j19))*sin(j20) + (sin(j17)*cos(j19) + sin(j18)*sin(j19)*cos(j17))*cos(j20))*sin(j21) + ((-sin(j17)*sin(j19) + sin(j18)*cos(j17)*cos(j19))*cos(j20) - (sin(j17)*cos(j19) + sin(j18)*sin(j19)*cos(j17))*sin(j20))*cos(j21))*sin(j22) + cos(j17)*cos(j18)*cos(j22)  ,  -(-((-sin(j17)*sin(j19) + sin(j18)*cos(j17)*cos(j19))*sin(j20) + (sin(j17)*cos(j19) + sin(j18)*sin(j19)*cos(j17))*cos(j20))*sin(j21) + ((-sin(j17)*sin(j19) + sin(j18)*cos(j17)*cos(j19))*cos(j20) - (sin(j17)*cos(j19) + sin(j18)*sin(j19)*cos(j17))*sin(j20))*cos(j21))*cos(j22) - sin(j22)*cos(j17)*cos(j18)  ,  211*(-sin(j17)*sin(j19) + sin(j18)*cos(j17)*cos(j19))*cos(j20)/500 - 211*(sin(j17)*cos(j19) + sin(j18)*sin(j19)*cos(j17))*sin(j20)/500 - 187*sin(j17)*sin(j19)/500 - sin(j17)*cos(j19)/20 + sin(j17)/20 - sin(j18)*sin(j19)*cos(j17)/20 + 187*sin(j18)*cos(j17)*cos(j19)/500 + sin(j18)*cos(j17)/20 + 89/1000],
        [(sin(j19)*sin(j20)*cos(j18) - cos(j18)*cos(j19)*cos(j20))*sin(j21) + (-sin(j19)*cos(j18)*cos(j20) - sin(j20)*cos(j18)*cos(j19))*cos(j21)  ,  -((sin(j19)*sin(j20)*cos(j18) - cos(j18)*cos(j19)*cos(j20))*cos(j21) - (-sin(j19)*cos(j18)*cos(j20) - sin(j20)*cos(j18)*cos(j19))*sin(j21))*sin(j22) + sin(j18)*cos(j22)  ,  -((sin(j19)*sin(j20)*cos(j18) - cos(j18)*cos(j19)*cos(j20))*cos(j21) - (-sin(j19)*cos(j18)*cos(j20) - sin(j20)*cos(j18)*cos(j19))*sin(j21))*cos(j22) - sin(j18)*sin(j22)  ,  211*sin(j19)*sin(j20)*cos(j18)/500 + sin(j19)*cos(j18)/20 - 211*cos(j18)*cos(j19)*cos(j20)/500 - 187*cos(j18)*cos(j19)/500 - cos(j18)/20],
        [0  ,  0  ,  0  ,  1]
      ])

P = T[:3,3] # extract the position vector


#print "T=",T

print "calc jacobian of position vector..."
Jp = P.jacobian(q)


Jps = trigsimp(Jp[0,0])
Jps1 = simplify(Jps)
Jps2 = trigsimp(Jps1)

print "P=",P
pprint(P)
print "Jp="
pprint(Jp)
print "Jps(0,0)="
pprint(Jps)
print "Jps1="
pprint(Jps1)
print "Jps2="
pprint(Jps2)
print "floating point version="
NJps2 = N(Jps2,10)
pprint(NJps2)

[(c_name, c_code), (h_name, c_header)] = codegen( ("Jp00", NJps2), "C", "test")
print c_name
print h_name
print c_header
print c_code

print "size(Jp) = ",Jp.shape

Tr = trigsimp(T[0,0] + T[1,1] + T[2,2])
S = trigsimp(1/(sqrt(Tr + 1)))


print "Tr=",Tr
print "S =",S

print "From euclideanspace website"
Qw = trigsimp(1/(4*S))
Qw = simplify(trigsimp(1/(4*S)))
print "Qw="
pprint(Qw)
Qx = simplify(trigsimp((T[2,1] - T[1,2])*S))
print "Qx="
pprint(Qx)
Qy = simplify(trigsimp((T[0,2] - T[2,0])*S))
print "Qy="
pprint(Qy)
Qz = simplify(trigsimp((T[1,0] - T[0,1])*S))
print "Qz="
pprint(Qz)
Qnes = simplify(trigsimp(sqrt(Qw*Qw + Qx*Qx + Qy*Qy + Qz*Qz)))

print "---------------------------------------"
print "---------------------------------------"
print "---------------------------------------"
print "From S&S pg. 36 (less sign term)"
Qw = simplify(trigsimp(sqrt(Tr + 1)/2))
print "Qw="
pprint(Qw)
Qx = simplify(trigsimp(sqrt(T[0,0] - T[1,1] - T[2,2] + 1)))
print "Qx="
pprint(Qx)

Qy = simplify(trigsimp(sqrt(T[1,1] - T[2,2] - T[1,1] + 1)))
print "Qy="
pprint(Qy)

Qz = simplify(trigsimp(sqrt(T[0,0] - T[2,2] - T[0,0] + 1)))
print "Qz="
pprint(Qz)


Qn = simplify(trigsimp(sqrt(Qw*Qw + Qx*Qx + Qy*Qy + Qz*Qz)))
print "Qn =", Qn

Q0 = simplify(Qn-Qnes)
print "Q0=",Q0

