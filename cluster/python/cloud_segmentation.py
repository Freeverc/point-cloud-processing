import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Load data
file_path = 'dataset/example1.ply'
point_cloud = o3d.io.read_point_cloud(file_path)
print(len(point_cloud.points))

# Plane segmentation
plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.5,
                                                 ransac_n=10,
                                                 num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
inlier_cloud = point_cloud.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([0, 1, 0])
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud], height=600, width=800)

# Remove outliers
cl, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=20,
                                                     std_ratio=2.0)
point_cloud = outlier_cloud.select_by_index(ind)
o3d.visualization.draw_geometries([point_cloud], height=600, width=800)

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(point_cloud.cluster_dbscan(eps=1, min_points=100, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([point_cloud], height=600, width=800)

# Get volume
label_list = set(labels)
show_cloud = []
for l in label_list:
    if l < 0:
        continue
    indx = np.where(labels == l)[0].tolist()
    cluster = point_cloud.select_by_index(indx)
    hull, _ = cluster.compute_convex_hull()
    v = hull.get_volume()
    print("cluster : ", l, " points: ", len(cluster.points), " volume: ", v)
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))
    show_cloud.append(cluster)
    show_cloud.append(hull_ls)

o3d.visualization.draw_geometries(show_cloud,  height=600, width=800)