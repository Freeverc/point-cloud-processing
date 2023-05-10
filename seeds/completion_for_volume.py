import open3d as o3d
import numpy as np
import random


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
    distance = np.abs(np.dot(P, [a,b,c]) + d) / np.sqrt(a**2 + b**2 + c**2)
    P_proj = P - distance * np.array([a,b,c]) / np.sqrt(a**2 + b**2 + c**2)
    return P_proj


def complete_and_get_volume(plane_model, point_cloud):
    alpha = 10
    print(f"alpha={alpha:.3f}")
    print('Running alpha shapes surface reconstruction ...')
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape( point_cloud, alpha)
    mesh.compute_triangle_normals(normalized=False)
    mesh.orient_triangles()
    print("Displaying reconstructed mesh ...")
    # o3d.visualization.draw_geometries([mesh], mesh_show_back_face=False, mesh_show_wireframe=True)
    # 返回三角化后的体积值
    print(mesh.is_orientable())
    volume = mesh.get_volume()
    return volume, mesh

def main():
    # 加载点云a
    point_cloud_a = o3d.io.read_point_cloud("../data/seeds/plane/cloudPlane0.pcd")

    # 使用点云a拟合平面
    plane_model, inliers = point_cloud_a.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    print("plane : ", a, b, c, d)

    for id in range(22):
        print(id)

        # 加载点云b
        point_cloud_b = o3d.io.read_point_cloud('../data/seeds/seeds/cloud_cluster_{}.pcd'.format(id))

        # 获取点云b的点集
        points_b = np.asarray(point_cloud_b.points)

        # 投影操作
        points_prj = []
        for p in points_b:
            prj = project_point_on_plane(p, plane_model)
            points_prj.append(prj)

        points_c = np.asarray(points_prj)
        point_cloud_c = o3d.geometry.PointCloud()
        point_cloud_c.points = o3d.utility.Vector3dVector(points_c)

        # 拼接点云b和投影上来的点为点云c
        point_cloud_d =point_cloud_b+point_cloud_c
        random.shuffle(point_cloud_d.points)
        # o3d.visualization.draw_geometries([point_cloud_d])
        volume, mesh = complete_and_get_volume(plane_model, point_cloud_d)
        print(id, " volume : ", volume)

        # 保存点云c为pcd文件
        o3d.io.write_triangle_mesh("../data/seeds/result/mesh_{}_volume_{}.ply".format(id, volume), mesh)


if __name__ == "__main__":
    main()

