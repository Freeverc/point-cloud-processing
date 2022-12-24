import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D



# 参数,用来调整聚类的大小(1-3)
ep = 2

# Load data
# 加载数据
file_path = r'C:\Users\Freeverc\Projects\point-cloud-processing\data\cluster\data0.ply'
point_cloud = o3d.io.read_point_cloud(file_path)
print(len(point_cloud.points))

# Plane segmentation
# 检测平面
plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.5,
                                                 ransac_n=10,
                                                 num_iterations=1000)
# 打印平面参数
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
inlier_cloud = point_cloud.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
# 去除平面上的点
outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([0, 1, 0])
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud], height=600, width=800)
o3d.io.write_point_cloud("inlier.ply", inlier_cloud)  # 输出文件
o3d.io.write_point_cloud("outlier.ply", outlier_cloud)  # 输出文件

# Remove outliers
# 去除噪点
cl, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=20,
                                                   std_ratio=2.0)
point_cloud = outlier_cloud.select_by_index(ind)
# 显示去除平面并且去噪的点云
o3d.visualization.draw_geometries([point_cloud], height=600, width=800)
o3d.io.write_point_cloud("clean.ply", point_cloud)  # 输出文件

# 计算聚类
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(point_cloud.cluster_dbscan(eps=ep, min_points=100, print_progress=True))

# 给每个聚类染上不同颜色
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
# 显示每个聚类
o3d.visualization.draw_geometries([point_cloud], height=600, width=800)
o3d.io.write_point_cloud("color.ply", point_cloud)  # 输出文件
# Get volume
# 计算体积
label_list = set(labels)
show_cloud = []
for l in label_list:
    if l < 0:
        continue
    indx = np.where(labels == l)[0].tolist()
    cluster = point_cloud.select_by_index(indx)
    # 计算凸包
    hull, _ = cluster.compute_convex_hull()
    # 给每个凸包 计算体积
    v = hull.get_volume()
    print("cluster : ", l, " points: ", len(cluster.points), " volume: ", v)
    # 计算三角化的网格，用于显示
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))
    show_cloud.append(cluster)
    show_cloud.append(hull_ls)
# 显示每个聚类和它的凸包
o3d.visualization.draw_geometries(show_cloud, height=600, width=800)
