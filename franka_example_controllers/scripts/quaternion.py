import math

class Quaternion:
#    basic quaternion class
    x = 0
    y = 0
    z = 0
    w = 0
    
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def setValues(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def print(self):
        print("Q: [x,y,z,w]  = [" + self.x + " , " + self.y + " , " + self.z + " , "+ self.w )

    def multiplyWith(self, q2):
        self.x =  self.x * q2.w + self.y * q2.z - self.z * q2.y + self.w * q2.x;
        self.y = -self.x * q2.z + self.y * q2.w + self.z * q2.x + self.w * q2.y;
        self.z =  self.x * q2.y - self.y * q2.x + self.z * q2.w + self.w * q2.z;
        self.w = -self.x * q2.x - self.y * q2.y - self.z * q2.z + self.w * q2.w;
        

def eulerToQuaternion(yaw, pitch, roll):
    cy = math.cos(math.radians(yaw * 0.5))
    sy = math.sin(math.radians(yaw * 0.5))
    cp = math.cos(math.radians(pitch * 0.5))
    sp = math.sin(math.radians(pitch * 0.5))
    cr = math.cos(math.radians(roll * 0.5))
    sr = math.sin(math.radians(roll * 0.5))

    q = Quaternion(0,0,0,0);
    q.w = (cr * cp * cy) + (sr * sp * sy)
    q.x = (sr * cp * cy) - (cr * sp * sy)
    q.y = (cr * sp * cy) + (sr * cp * sy);
    q.z = (cr * cp * sy) - (sr * sp * cy);
    
    return q
