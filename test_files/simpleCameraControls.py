
import open3d as o3d
import numpy as np
import time

vis = o3d.visualization.Visualizer()
vis.create_window()

mesh_box = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
mesh_box.compute_vertex_normals()
mesh_box.paint_uniform_color([0.9, 0.1, 0.1])

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[-2, -2, -2])

vis.add_geometry(mesh_box)
vis.add_geometry(mesh_frame)

ctr = vis.get_view_control()

#vis.update_renderer()

""" Change field of view
print("Field of view (before changing) %.2f" % ctr.get_field_of_view())
time.sleep(1)
ctr.change_field_of_view(step=90.0)
print("Field of view (after changing) %.2f" % ctr.get_field_of_view())
"""

# Zoom camera in or out
ctr.set_zoom(2.0)

#ctr.set_up([-2,-1,0])
ctr.set_front([1,1,1])

vis.update_renderer()

#time.sleep(1)
vis.run()
vis.destroy_window()