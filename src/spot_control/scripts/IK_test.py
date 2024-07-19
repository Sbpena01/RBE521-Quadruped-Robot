import numpy as np
import math




def legIK(x,y,z):

    """
    x/y/z=Position of the Foot in Leg-Space

    F=Length of shoulder-point to target-point on x/y only
    G=length we need to reach to the point on x/y
    H=3-Dimensional length we need to reach
    """
    l1=25
    l2=20
    l3=80
    l4=80

    F=np.sqrt(x**2+y**2-l1**2)
    G=F-l2  
    H=np.sqrt(G**2+z**2)

    theta1=-math.atan2(y,x)-math.atan2(F,-l1)

    D=(H**2-l3**2-l4**2)/(2*l3*l4)
    theta3=math.acos(D) 

    theta2=math.atan2(z,G)-math.atan2(l4*math.sin(theta3),l3+l4*math.cos(theta3))

    return(theta1,theta2,theta3)

print(legIK(0, 40, 20))