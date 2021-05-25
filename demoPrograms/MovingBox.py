
import numpy as np
import open3d as o3d
import time

Scene = o3d.visualization.Visualizer()
Scene.create_window()

mesh_box = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
mesh_box.compute_vertex_normals()
mesh_box.paint_uniform_color([0.9, 0.1, 0.1])

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[-2, -2, -2])

Scene.add_geometry(mesh_box)
Scene.add_geometry(mesh_frame)

for i in range(10):
    mesh_box.translate(np.array([0,-0.1,0]))

    Scene.update_geometry(mesh_box)
    Scene.poll_events()
    Scene.update_renderer()
    
    time.sleep(0.1)
    
Scene.destroy_window()	