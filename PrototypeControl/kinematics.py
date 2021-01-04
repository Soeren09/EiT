import math
import kinematics

class kinematics:
    def __init__(self, L1, L2, offset):
        self._L1 = L1
        self._L2 = L2
        self._offset = math.radians(offset)

    def forward(self, q1_degs, q2_degs):
        q1 = math.radians(q1_degs)
        q2 = math.radians(q2_degs)
        x = self._L1*math.cos(self._offset + q1) + self._L2*(math.cos(self._offset + q1)*math.cos(q2) - math.sin(self._offset + q1)*math.sin(q2))
        y = self._L1*math.sin(self._offset + q1) + self._L2*(math.cos(self._offset + q1)*math.sin(q2) + math.sin(self._offset + q1)*math.cos(q2))
        return x, y

    def invers(self, x,y):
        cos_q2 = (x*x + y*y - self._L1*self._L1  - self._L2*self._L2 ) /(2*self._L1*self._L2)
        q2 = math.acos(cos_q2)
        q1 = math.atan2(y,x)-math.atan2((self._L2*math.sin(q2)),(self._L1+self._L2*cos_q2) )
        q1_out = math.degrees(q1-self._offset)
        q2_out = math.degrees(q2)
        return q1_out, q2_out

def LinDecomp(start, end, steps):
    kin = kinematics(L1=0.055,L2=0.105,offset=-17)
    output=[]
    for i in range(steps):
        step = (1+i)/steps
        x = start[0]+((end[0]-start[0])*step)
        y = start[1]+((end[1]-start[1])*step)
        q1,q2 = kin.invers(x,y)
        if q1 < 0 or q1 > 215 or q2 < 0 or q2 > 90:
            print("Error - inv kin solution is out of bounds")
        output.append([q1,q2])
    return output


if __name__ == "__main__":
    # Tests of forward and invers kinematics.
    kin = kinematics(L1=0.055,L2=0.105,offset=-17)
    x,y = kin.forward(67,40)
    print(x, y)
    q1,q2 = kin.invers(x,y)
    print(q1,q2)
    # Metoden nedenfor laver en list af Q-values, som skal afspilles med jævn fart, så burde C-space linear movement opnåes.
    out = LinDecomp([0.06, 0.143],[-0.06, 0.143],100)
