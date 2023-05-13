import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


def project_point_on_plane(point, plane):
    """
    Project a point onto a plane.
    Args:
        point (np.array): a 3D point (x,y,z)
        plane (tuple): a plane defined by its coefficients (a,b,c,d)
    Returns:
        np.array: the projection of the point onto the plane
    """
    a, b, c, d = plane
    P = np.array(point)
    distance = np.abs(np.dot(P, [a, b, c]) + d) / np.sqrt(a**2 + b**2 + c**2)
    P_proj = P - distance * np.array([a, b, c]) / np.sqrt(a**2 + b**2 + c**2)
    if abs(np.dot(P_proj, [a, b, c]) + d) > 0.1:
        P_proj = P + distance * \
            np.array([a, b, c]) / np.sqrt(a**2 + b**2 + c**2)
    return P_proj


def complete_and_get_volume(plane_model, point_cloud):
    alpha = 10
    print(f"alpha={alpha:.3f}")
    print('Running alpha shapes surface reconstruction ...')
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
        point_cloud, alpha)
    mesh.compute_triangle_normals(normalized=False)
    mesh.orient_triangles()
    print("Displaying reconstructed mesh ...")
    o3d.visualization.draw_geometries(
        [mesh], mesh_show_back_face=False, mesh_show_wireframe=True)
    # 返回三角化后的体积值
    print(mesh.is_orientable())
    volume = mesh.get_volume()
    return volume, mesh


def main():
    # Load data
    file_path = '../../data/volume/output.xyz'
    point_cloud = o3d.io.read_point_cloud(file_path)
    print(len(point_cloud.points))

    # Plane segmentation
    plane_model, inliers_bad = point_cloud.segment_plane(distance_threshold=0.1,
                                                         ransac_n=10,
                                                         num_iterations=1000)
    plane_model_bad, inliers = point_cloud.segment_plane(distance_threshold=10,
                                                         ransac_n=20,
                                                         num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = point_cloud.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
    outlier_cloud.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries(
        [inlier_cloud, outlier_cloud], height=600, width=800)
    # o3d.visualization.draw_geometries(
    #     [inlier_cloud], height=600, width=800)
    # o3d.visualization.draw_geometries(
    #     [outlier_cloud], height=600, width=800)
    # o3d.io.write_point_cloud("../../data/volume/inlier.ply", inlier_cloud)
    # o3d.io.write_point_cloud("../../data/volume/outlier.ply", outlier_cloud)

    # 计算聚类
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(outlier_cloud.cluster_dbscan(
            eps=25, min_points=200, print_progress=True))

    # 给每个聚类染上不同颜色
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(
        labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    label_list = set(labels)
    clusters = []
    for l in label_list:
        if l < 0:
            continue
        indx = np.where(labels == l)[0].tolist()
        cluster = outlier_cloud.select_by_index(indx)
        clusters.append(cluster)
    # 显示每个聚类
    o3d.visualization.draw_geometries(clusters, height=600, width=800)

    show_cloud = []
    for cluster in clusters:
        # 投影操作
        points_prj = []
        points_b = np.asarray(cluster.points)
        for p in points_b:
            prj = project_point_on_plane(p, plane_model)
            points_prj.append(prj)

        points_c = np.asarray(points_prj)
        point_cloud_c = o3d.geometry.PointCloud()
        point_cloud_c.points = o3d.utility.Vector3dVector(points_c)

        # 拼接点云b和投影上来的点为点云c
        point_cloud_d = cluster + point_cloud_c
        o3d.visualization.draw_geometries(
            [inlier_cloud, point_cloud_d], height=600, width=800)

        # 计算凸包
        hull, _ = cluster.compute_convex_hull()
        # 给每个凸包 计算体积
        if not hull.is_watertight():
            continue

        v = hull.get_volume()
        print("volume : ", v)

        # 计算三角化的网格，用于显示
        hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
        hull_ls.paint_uniform_color((1, 0, 0))
        show_cloud.append(cluster)
        show_cloud.append(hull_ls)
    # 显示每个聚类和它的凸包
    o3d.visualization.draw_geometries(show_cloud, height=600, width=800)

    # volume, mesh = complete_and_get_volume(plane_model, point_cloud_d)


if __name__ == "__main__":
    main()
