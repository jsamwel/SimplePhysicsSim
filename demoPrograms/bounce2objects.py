
import numpy as np
import open3d as o3d
import time

def main():
    sim = simulation()
    sim.generateBalls()
    
    print(sim.calculateCollision(sim.balls[0], sim.balls[1]))

class ball:
    def __init__(self, **kwargs):
        self.pos = kwargs.get("pos", np.array([0,0,0]))
        self.rot = kwargs.get("rot", np.array([0,0,0]))
        self.vel = kwargs.get("vel", np.array([0,0,0]))
        self.freq = 1 / kwargs.get("herz", 60)
        self.mass = kwargs.get("mass", 1)
        self.rad = kwargs.get("rad", 0.1)
    def update(self):
        pass
        
    def posAfterCollision(self):
        pass
        
    def getFuturePos(self):
        return self.pos + self.vel * self.freq

class simulation:
    def __init__(self):
        self.balls = []
        self.herz = 50
    
    def generateBalls(self):
        self.balls.append(ball(vel=np.array([4,1,0]),
                                herz=self.herz))
        self.balls.append(ball(pos=np.array([.3,0,0]),
                                vel=np.array([-4,0,0]),
                                herz=self.herz))
    
    def calculateCollision(self, ball1, ball2):
        if self.calculateDistance(ball1, ball2, 0) < 0:
            distance = self.calculateDistance(ball1, ball2, 1)
            
            m1 = ball1.mass
            m2 = ball2.mass
            
            v1 = ball1.vel
            v2 = ball2.vel
        
            v1s = np.sqrt(np.square(v1[0])+np.square(v1[1])+np.square(v1[1]))
            v2s = np.sqrt(np.square(v2[0])+np.square(v2[1])+np.square(v2[1]))
        
            v1n = (v1*(m1-m2)+2*m2*v2)/(m1+m2)
            v2n = (v2*(m2-m1)+2*m1*v1)/(m1+m2)
            
            s = distance / (v1s + v2s)
            
            cpos1 = ball1.pos + ball1.vel * s
            cpos2 = ball2.pos + ball2.vel * s
            return cpos1, cpos2
        
    def calculateDistance(self, ball1, ball2, current):
        if not current:
            dPos = ball1.getFuturePos() - ball2.getFuturePos()
        else:
            dPos = ball1.pos - ball2.pos
        
        dDis = np.sqrt(np.square(dPos[0])+np.square(dPos[1])+np.square(dPos[1]))
        
        return dDis - ball1.rad - ball2.rad
    
if __name__ == "__main__":
    main()