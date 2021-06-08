
import numpy as np
import open3d as o3d
import time

def main():
    herz = 50

    sim = simulation(herz=herz)
    sim.generateSpheres()
    
    for i in range(herz*1):
        sim.update()
    
        spherepos = []
        for sphere in sim.spheres:
            spherepos.append(sphere.pos)
        
        print(spherepos)
    
    sim.Scene.run()

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
        self.herz = kwargs.get("herz", 60)
        
        self.Scene = o3d.visualization.Visualizer()
        self.Scene.create_window()
        
        self.ctr = self.Scene.get_view_control() 
    
    def generateSpheres(self):
        self.spheres.append(sphere(vel=np.array([.5,0,0]),
                                herz=self.herz))
        self.spheres.append(sphere(pos=np.array([1,.1,0]),
                                vel=np.array([-0.5,0,0]),
                                herz=self.herz))
        
        for i in range(len(self.spheres)):
            self.geo.append(o3d.geometry.TriangleMesh.create_sphere(radius=self.spheres[i].rad))
            self.geo[i].compute_vertex_normals()
            self.geo[i].paint_uniform_color([0.1, 0.1, 0.7])
            
            self.Scene.add_geometry(self.geo[i])
            
        self.ctr.set_zoom(10.0)
        self.ctr.set_front([0,0,1])
    
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

        sphere1.pos = p1 + v1n * (1 / self.herz - s)
        sphere2.pos = p2 + v2n * (1 / self.herz - s)
        
        sphere1.vel = v1n
        sphere2.vel = v2n
        
        sphere1.updated = 1
        sphere2.updated = 1
        
    def calculateDistance(self, sphere1, sphere2, current):
        if not current:
            dPos = sphere1.getFuturePos() - sphere2.getFuturePos()
        else:
            dPos = sphere1.pos - sphere2.pos
        
        dDis = np.sqrt(np.square(dPos[0])+np.square(dPos[1])+np.square(dPos[2]))
        
        return dDis - sphere1.rad - sphere2.rad
        
    def update(self):
        amount = len(self.spheres)
    
        for i in range(amount):
            sphere1 = self.spheres[i]
            
            if i < (amount - 1) and not sphere1.updated:
                for y in range(i + 1, amount):
                    sphere2 = self.spheres[y]
                    
                    if self.calculateDistance(sphere1, sphere2, 0) < 0:
                        self.calculateCollision(sphere1, sphere2)
                        print("collision")
                    elif (y == amount - 1) and not sphere1.updated:
                        sphere1.update()
                        
            elif not sphere1.updated:
                sphere1.update()
                
        for i in range(len(self.spheres)):
            self.geo[i].translate(self.spheres[i].pos, relative=False)
            self.Scene.update_geometry(self.geo[i])
            self.Scene.poll_events()
            self.Scene.update_renderer()
            self.spheres[i].updated = 0
    
if __name__ == "__main__":
    main()