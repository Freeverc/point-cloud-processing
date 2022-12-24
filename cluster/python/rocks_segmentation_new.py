import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import csv
from mpl_toolkits.mplot3d import Axes3D



# 参数,用来调整聚类的大小(1-3)
# ep = 0.02
rock_ep = 0.045
rock_size = 20
# 加载数据
file_path = r'./data/xianchang1.ply'
# file_path = r'C:\Users\Freeverc\Projects\point-cloud-processing\data\cluster\data3.ply'
point_cloud = o3d.io.read_point_cloud(file_path)
print(len(point_cloud.points))
mean = np.mean(point_cloud.points, 0)
print(mean)
points = point_cloud.points - mean
# for point in point_cloud.points:
#     point[0] = point[0] - mean[0]
#     point[1] = point[1] - mean[1]
#     point[2] = point[2] - mean[2]
#     points.append(point)

# point_cloud.points = points
point_cloud.points = o3d.utility.Vector3dVector(points)

# 降采样
point_cloud = point_cloud.uniform_down_sample(every_k_points=20)

# 去除噪点
cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=20,
                                                 std_ratio=1.0)
point_cloud = point_cloud.select_by_index(ind)

# 显示去除平面并且去噪的点云
# o3d.visualization.draw_geometries([point_cloud], height=600, width=800)
o3d.io.write_point_cloud("clean.ply", point_cloud)  # 输出文件

# # Plane segmentation
# plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.003,
#                                                  ransac_n=20,
#                                                  num_iterations=1000)
# [a, b, c, d] = plane_model
# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
# inlier_cloud = point_cloud.select_by_index(inliers)
# inlier_cloud.paint_uniform_color([1.0, 0, 0])
# outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
# print("in : ", len(inlier_cloud.points))
# print("out : ", len(outlier_cloud.points))
# outlier_cloud.paint_uniform_color([0, 1, 0])
# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud], height=600, width=800)

# point_cloud = outlier_cloud

# 去除噪点
cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=20,
                                                 std_ratio=1.0)
point_cloud = point_cloud.select_by_index(ind)

# 计算聚类
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(point_cloud.cluster_dbscan(eps=rock_ep, min_points=rock_size, print_progress=True))

# 给每个聚类染上不同颜色
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

# 显示每个聚类
# o3d.visualization.draw_geometries([point_cloud], height=600, width=800)
o3d.io.write_point_cloud("color.ply", point_cloud)  # 输出文件
# Get volume
# 计算体积
label_list = set(labels)
show_cloud = []
header = ['石头ID', '长度x', '长度y', '长度z', '长度max', '点数', '体积']
with open('rock.csv', 'w') as myFile:
    my_writer = csv.writer(myFile)
    my_writer.writerow(header)
    for l in label_list:
        if l < 0:
            continue
        indx = np.where(labels == l)[0].tolist()
        cluster = point_cloud.select_by_index(indx)
        if len(cluster.points) < 4:
            continue

        aabb = cluster.get_axis_aligned_bounding_box()
        obb = cluster.get_oriented_bounding_box()
        aabb_len = aabb.get_extent()
        obb_len = max(obb.get_max_bound() - obb.get_min_bound())
        # 计算凸包
        hull, _ = cluster.compute_convex_hull()
        # 给每个凸包 计算体积
        if not hull.is_watertight():
            continue

        v = hull.get_volume()
        my_writer.writerow([l,aabb_len[0],aabb_len[1],aabb_len[2],obb_len, len(cluster.points), v])
        print("石头 : ", l, " 长度: ",obb_len," 点数 : ", len(cluster.points), " 体积 : ", v)
        # 计算三角化的网格，用于显示
        hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
        hull_ls.paint_uniform_color((1, 0, 0))
        show_cloud.append(cluster)
        show_cloud.append(hull_ls)
    # 显示每个聚类和它的凸包
    o3d.visualization.draw_geometries(show_cloud, height=600, width=800)
