
import numpy as np
import open3d as o3d
import time
from random import randint, seed

def main():
    herz = 50
    debug = False

    sim = simulation(herz=herz)
    sim.generateSpheres()
    
    for i in range(herz*5):
        sim.update()
        
        if debug:
            spherepos = []
            for sphere in sim.spheres:
                spherepos.append(sphere.pos)
            
            print(spherepos)
           
        time.sleep(1/herz)
    
    sim.Scene.run()

class boundary:
    def __init__(self, **kwargs):
        pass

class sphere:
    def __init__(self, **kwargs):
        self.pos = kwargs.get("pos", np.array([0,0,0]))
        self.rot = kwargs.get("rot", np.array([0,0,0]))
        self.vel = kwargs.get("vel", np.array([0,0,0]))
        self.freq = 1 / kwargs.get("herz", 60)
        self.mass = kwargs.get("mass", 1)
        self.rad = kwargs.get("rad", 0.1)
        self.updated = 0
        
    def update(self):
        self.pos = self.pos + self.vel * self.freq
        
        self.updated = 1
        
    def getFuturePos(self):
        return self.pos + self.vel * self.freq

class simulation:
    def __init__(self, **kwargs):
        self.spheres = []
        self.geo = []
        
        self.boundaries = np.array([[-2,2],[-2,2],[-2,2]])
        self.herz = kwargs.get("herz", 60)
        
        self.Scene = o3d.visualization.VisualizerWithKeyCallback()
        self.Scene.create_window()
        self.Scene.register_key_callback(ord("C"), self.center_view)
        #self.Scene.set_full_screen(True)
        
        self.ctr = self.Scene.get_view_control() 
    
    def center_view(self, vis):
        vis.reset_view_point(True)
    
    def runSim(self, time):
        pass
    
    def generateSpheres(self):
        self.spheres.append(sphere(vel=np.array([5,-.05,-.05]),
                                herz=self.herz))
        self.spheres.append(sphere(pos=np.array([1,0,0]),
                                vel=np.array([-5,0,0]),
                                herz=self.herz))
        
        for i in range(len(self.spheres)):
            self.geo.append(o3d.geometry.TriangleMesh.create_sphere(radius=self.spheres[i].rad))
            self.geo[i].compute_vertex_normals()
            self.geo[i].paint_uniform_color([0.1, 0.1, 0.7])
            
            self.Scene.add_geometry(self.geo[i])
            
        self.ctr.set_zoom(10.0)
        self.ctr.set_front([1,0,2])
        self.ctr.set_constant_z_far(5)
        self.ctr.set_constant_z_near(2)
        #self.ctr.translate(0,-10)
    
    def calculateCollision(self, sphere1, sphere2):
        #https://en.wikipedia.org/wiki/Elastic_collision
        
        distance = self.calculateDistance(sphere1, sphere2, 1)
        
        m1, m2 = sphere1.mass, sphere2.mass
        v1, v2 = sphere1.vel, sphere2.vel
    
        v1s = np.sqrt(np.square(v1[0])+np.square(v1[1])+np.square(v1[1]))
        v2s = np.sqrt(np.square(v2[0])+np.square(v2[1])+np.square(v2[1]))
        
        s = distance / (v1s + v2s)
        
        #Calculate the position at the time of collision
        p1 = sphere1.pos + sphere1.vel * s
        p2 = sphere2.pos + sphere2.vel * s
        
        v1n = v1 - (2 * m2 / (m1 + m2)) * np.dot(v1 - v2, p1 - p2) / np.linalg.norm(p1 - p2) ** 2 * (p1 - p2)
        v2n = v2 - (2 * m1 / (m2 + m1)) * np.dot(v2 - v1, p2 - p1) / np.linalg.norm(p2 - p1) ** 2 * (p2 - p1)
        
        # Update the positions and velocities of the spheres
        sphere1.pos = p1 + v1n * (1 / self.herz - s)
        sphere2.pos = p2 + v2n * (1 / self.herz - s)
        
        sphere1.vel, sphere2.vel = v1n, v2n        
        sphere1.updated, sphere2.updated = 1, 1
        
    def checkBoundaries(self, sphere1):
        fpos = sphere1.getFuturePos()
        returnVar = 0
        
        for i in range(len(fpos)):
            if fpos[i] - sphere1.rad < self.boundaries[i][0]:
                # Calculate position after collision
                edge = self.boundaries[i][0]
                pos = sphere1.pos[i] - sphere1.rad
                
                b2 = edge - (-sphere1.vel[i]) * ((pos-edge)/-sphere1.vel[i])
                sphere1.pos[i] = -sphere1.vel[i] * 1/self.herz + b2 + sphere1.rad
                
                # Revert speed
                sphere1.vel[i] = -sphere1.vel[i]
                returnVar = 1
                
            elif fpos[i] + sphere1.rad > self.boundaries[i][1]:
                # Calculate position after collision
                edge = self.boundaries[i][1]
                pos = sphere1.pos[i] + sphere1.rad
                
                b2 = edge - (-sphere1.vel[i]) * ((pos-edge)/-sphere1.vel[i])
                sphere1.pos[i] = -sphere1.vel[i] * 1/self.herz + b2 - sphere1.rad
                
                # Revert speed
                sphere1.vel[i] = -sphere1.vel[i]
                returnVar = 1
        
        return returnVar
        
    def calculateDistance(self, sphere1, sphere2, current):
        if not current:
            dPos = sphere1.getFuturePos() - sphere2.getFuturePos()
        else:
            dPos = sphere1.pos - sphere2.pos
        
        dDis = np.sqrt(np.square(dPos[0])+np.square(dPos[1])+np.square(dPos[2]))
        
        return dDis - sphere1.rad - sphere2.rad
        
    def update(self):
        numSpheres = len(self.spheres)
    
        for currentSphere in range(numSpheres):
            sphere1 = self.spheres[currentSphere]
            
            #Check for each sphere in the list if it will collide with the other spheres
            if currentSphere < (numSpheres - 1) and not sphere1.updated:
                for checkSphere in range(currentSphere + 1, numSpheres):
                    sphere2 = self.spheres[checkSphere]
                    
                    if self.calculateDistance(sphere1, sphere2, 0) < 0:
                        self.calculateCollision(sphere1, sphere2)
                        
            #If the sphere does not collide update its position normally
            if not sphere1.updated:
                if not self.checkBoundaries(sphere1):
                    sphere1.update()
        
        #Update the scene with all the new positions
        for x in range(numSpheres):
            self.geo[x].translate(self.spheres[x].pos, relative=False)
            self.Scene.update_geometry(self.geo[x])
            self.Scene.poll_events()
            self.Scene.update_renderer()
            self.spheres[x].updated = 0
    
if __name__ == "__main__":
    main()