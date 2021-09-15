
import numpy as np
import open3d as o3d
import time

def main():
    herz = 50
    debug = False

    sim = simulation(herz=herz)
    
    for i in range(herz*5):
        sim.update()
        
        if debug:
            spherepos = []
            for sphere in sim.spheres:
                spherepos.append(sphere.pos)
            
            print(spherepos)
           
        time.sleep(1/herz)
    
    sim.Scene.run()

class classBoundary:
    def __init__(self, **kwargs):
        pass

class classSphere:
    def __init__(self, **kwargs):
        self.pos = kwargs.get("pos", np.array([0,0,0]))
        self.rot = kwargs.get("rot", np.array([0,0,0]))
        self.vel = kwargs.get("vel", np.array([0,0,0]))
        self.freq = 1 / kwargs.get("herz", 60)
        self.mass = kwargs.get("mass", 1)
        self.rad = kwargs.get("rad", 0.1)
        self.updated = 0
        
        self.__createVisual()
        
    def updateVisual(self):        
        self.geo.translate(self.pos, relative=False)
        
    def getFuturePos(self):
        return self.pos + self.vel * self.freq
        
    def __createVisual(self):
        self.geo = o3d.geometry.TriangleMesh.create_sphere(radius=self.rad)
        self.geo.compute_vertex_normals()
        self.geo.paint_uniform_color([0.1, 0.1, 0.7])

class simulation:
    def __init__(self, **kwargs):
        self.herz = kwargs.get("herz", 60)
        
        self.geo = []
        
        self.boundaries = np.array([[-2,2],[-2,2],[-2,2]])
        
        # Create scene for visualisation
        self.Scene = o3d.visualization.Visualizer()
        self.Scene.create_window()
        
        # Create objects for in simulation
        self.generateSpheres()
        
        # Place camera of the scene
        self.ctr = self.Scene.get_view_control() 
        self.ctr.set_zoom(10.0)
        self.ctr.set_front([1,0,2])
        self.ctr.set_constant_z_far(5)
        self.ctr.set_constant_z_near(2)
        #self.ctr.translate(0,-10)
        
    
    def generateSpheres(self):
        self.spheres = []
    
        self.spheres.append(classSphere(vel=np.array([5,-.05,-.05]),
                                herz=self.herz))
        self.spheres.append(classSphere(pos=np.array([1,0,0]),
                                vel=np.array([-5,0,0]),
                                herz=self.herz))
        
        # Add visuals of the spheres to the scene
        for sphere in self.spheres:            
            self.Scene.add_geometry(sphere.geo)
    
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
        
    def updatePosSphere(self, sphere):
        fpos = sphere.getFuturePos()
        
        for i in range(len(fpos)):
            if fpos[i] - sphere.rad < self.boundaries[i][0]:
                # Calculate position after collision
                edge = self.boundaries[i][0]
                pos = sphere.pos[i] - sphere.rad
                
                b2 = edge - (-sphere.vel[i]) * ((pos-edge)/-sphere.vel[i])
                sphere.pos[i] = -sphere.vel[i] * 1/self.herz + b2 + sphere.rad
                
                # Revert speed
                sphere.vel[i] = -sphere.vel[i]
                sphere.updated = 1
                
            elif fpos[i] + sphere.rad > self.boundaries[i][1]:
                # Calculate position after collision
                edge = self.boundaries[i][1]
                pos = sphere.pos[i] + sphere.rad
                
                b2 = edge - (-sphere.vel[i]) * ((pos-edge)/-sphere.vel[i])
                sphere.pos[i] = -sphere.vel[i] * 1/self.herz + b2 - sphere.rad
                
                # Revert speed
                sphere.vel[i] = -sphere.vel[i]
                sphere.updated = 1
        
        if not sphere.updated:
            sphere.pos = sphere.pos + sphere.vel * sphere.freq
        
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
            if currentSphere < (numSpheres - 1):
                for checkSphere in range(currentSphere + 1, numSpheres):
                    sphere2 = self.spheres[checkSphere]
                    
                    if self.calculateDistance(sphere1, sphere2, 0) < 0:
                        self.calculateCollision(sphere1, sphere2)
                        
            #If the sphere does not collide update its position normally
            if not sphere1.updated:
                self.updatePosSphere(sphere1)
                    
            sphere1.updateVisual()
        
            #Update the scene with all the new positions
            self.Scene.update_geometry(sphere1.geo)
            sphere1.updated = 0
            
        self.Scene.poll_events()
        self.Scene.update_renderer()
    
if __name__ == "__main__":
    main()