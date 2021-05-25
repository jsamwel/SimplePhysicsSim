
import numpy as np
import open3d as o3d
import time

def main():
    herz = 60

    Scene = o3d.visualization.Visualizer()
    Scene.create_window()
    
    #XZY
    meshSphere = objectPhys(geo="sphere", vel=np.array([0.1,-0.2,0.5]), pos=np.array([0,0,0]))
    
    walls = []
    wallsPos = np.array([   [[0,-.5,0], [0,0,0]],
                            [[0,0,-.5], [np.pi/2,0,0]],
                            [[0,0,.5], [np.pi/2,0,0]],
                            [[.5,0,0], [0,0,np.pi/2]],
                            [[-.5,0,0], [0,0,np.pi/2]],
                            [[0,.5,0], [0,0,0]]])

    for i in wallsPos:
        walls.append(objectPhys(geo="wall", pos=i[0], rot=i[1], herz=herz))
    
    # Add objects to the scene
    Scene.add_geometry(meshSphere.geo)
    for wall in walls:
        Scene.add_geometry(wall.geo)
    
    for i in range(10*herz):
        
        checkCollision(meshSphere, walls[0])
        
        for i in range(len(meshSphere.pos)):
            if meshSphere.pos[i] + 0.1 > 0.5:
                meshSphere.vel[i] = -meshSphere.vel[i]
            elif meshSphere.pos[i] - 0.1 < -0.5:
                meshSphere.vel[i] = -meshSphere.vel[i]
        
        
        meshSphere.update()
        for wall in walls:
            wall.update()
        
        Scene.update_geometry(meshSphere.geo)
        Scene.poll_events()
        Scene.update_renderer()
        
        time.sleep(1/herz)
        
    Scene.destroy_window()

def checkCollision(obj1, obj2):
    futurePos = [obj1.checkFuture(), obj2.checkFuture()]
    
class objectPhys:
    def __init__(self, *args, **kwargs):
        self.pos = kwargs.get("pos", np.array([0,0,0]))
        self.rot = kwargs.get("rot", np.array([0,0,0]))
        self.vel = kwargs.get("vel", np.array([0,0,0]))
        self.freq = 1 / kwargs.get("herz", 60)
        
        if kwargs["geo"] == "sphere":
            self.mass = kwargs.get("mass", 1)
            rad = kwargs.get("rad", 0.1)
            
            self.createSphere(rad)
        elif kwargs["geo"] == "box":
            self.mass = kwargs.get("mass", 1)
            self.createBox()
        elif kwargs["geo"] == "wall":
            self.mass = kwargs.get("mass", 0)
            self.createWall()
        
    def createSphere(self, rad):
        self.geo = o3d.geometry.TriangleMesh.create_sphere(radius=rad)
        self.geo.compute_vertex_normals()
        self.geo.paint_uniform_color([0.1, 0.1, 0.7])
        
    def createBox(self):
        pass
        
    def createWall(self):
        self.geo = o3d.geometry.LineSet()
        
        points = [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]]
        lines = [[0, 1], [1, 2], [2, 3], [3, 0]]
        colors = [[1, 0, 0] for i in range(len(lines))]
        
        self.geo.points = o3d.utility.Vector3dVector(points)
        self.geo.lines = o3d.utility.Vector2iVector(lines)
        self.geo.colors = o3d.utility.Vector3dVector(colors)        
        
    def update(self):
        self.pos = self.pos + self.vel * self.freq
        
        self.geo.translate(self.pos, relative=False)
        
        rotMatrix = o3d.geometry.get_rotation_matrix_from_xyz(self.rot)
        self.geo.rotate(rotMatrix, self.pos)
     
    def checkFuture(self):
        return self.pos + self.vel * self.freq
   
if __name__ == "__main__":
    main()