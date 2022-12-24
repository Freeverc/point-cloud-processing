from scipy import optimize
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def xclinerrors(para, points): 
    """法向量x矢量最大，圆柱面拟合误差"""
    # points为由点集构成的n*3矩阵，每一行为一个点的x,y,z坐标值
    a0 = b0 = c0 = 0
    x0 = 0
    y0, z0, r0 = para
    return (points[:, 0] - x0) ** 2 + (points[:, 1] - y0) ** 2 + (points[:, 2] - z0) ** 2 - (
            a0 * (points[:, 0] - x0) + b0 * (points[:, 1] - y0) + c0 * (points[:, 2] - z0)) ** 2 - r0 ** 2


def yclinerrors(para, points):
    """法向量y矢量最大，圆柱面拟合误差"""
    a0 = b0 = c0 = 0
    y0 = 0

    x0, z0, r0 = para
    return (points[:, 0] - x0) ** 2 +  (points[:, 2] - z0) ** 2 - r0 ** 2


def zclinerrors(para, points):
    """法向量z矢量最大，圆柱面拟合误差"""
    a0 = b0 = c0 = 0
    z0 = 0
    x0, y0, r0 = para
    return (points[:, 0] - x0) ** 2 + (points[:, 1] - y0) ** 2 + (points[:, 2] - z0) ** 2 - (
            a0 * (points[:, 0] - x0) + b0 * (points[:, 1] - y0) + c0 * (points[:, 2] - z0)) ** 2 - r0 ** 2


def cluster_point_cloud(point_cloud):
    clusters = []
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        # labels = np.array(
        #     point_cloud.cluster_dbscan(eps=0.55, min_points=1000, print_progress=True))
        labels = np.array(
            point_cloud.cluster_dbscan(eps=0.6, min_points=1000, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    for i in range(0, max_label + 1):
        idx = np.where(labels == i)[0].tolist()
        print(len(idx))
        clusters.append(point_cloud.select_by_index(idx))
        # o3d.visualization.draw_geometries([clusters[-1]])
    return clusters
    # o3d.visualization.draw_geometries([point_cloud],
    #                                   zoom=0.455,
    #                                   front=[-0.4999, -0.1659, -0.8499],
    #                                   lookat=[2.1813, 2.0619, 2.0999],
    #                                   up=[0.1204, -0.9852, 0.1215])

def fit(clusters):
    app = gui.Application.instance
    app.initialize()

    vis = o3d.visualization.O3DVisualizer("Open3D", 1024, 768)

    # 计算点集的x,y,z均值，x，y的范围，用于拟合参数初值
    params = []
    show =[]
    i = 0
    for cluster in clusters:
        i += 1
        points = np.asarray(cluster.points)
        print(np.shape(points))
        xm = np.mean(points[:,0])    # 特征点集x均值
        ym = np.mean(points[:,1])
        zm = np.mean(points[:,2])         

        # xm = -17
        # ym = np.mean(points[:,1])
        # zm = 97
        print('init : ', xm, ym, zm)
        xrange = np.max(points[:,0]) - np.min(points[:,0])    # 特征点集x范围
        yrange = np.max(points[:,1]) - np.min(points[:,1])
        zrange = np.max(points[:,2]) - np.min(points[:,2]) #
        r0 = max(xrange, zrange)
        # r0 =15
        a = 0
        b = 1
        c = 0

        # 最小二乘拟合 
        tparac = optimize.leastsq(yclinerrors, [xm, zm, r0], points , full_output=1)
        # tparac = optimize.leastsq(yclinerrors, [xm, zm, r0], points , full_output=1, maxfev=1000, xtol=0.1, factor=0.2, ftol=0.2, epsfcn=0.01)
        print(np.shape(tparac))
        print(tparac[2]['fvec'])
        parac = np.array([tparac[0][0], ym, tparac[0][1], tparac[0][2]])
        params.append(parac)
        # ec = np.mean(np.abs(tparac[2]['fvec'])) / parac[6]
        print(np.shape(parac))
        print(parac)

        mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=parac[3],
                                                                height=yrange)
        mesh_cylinder.compute_vertex_normals()
        mesh_cylinder.paint_uniform_color([0.45*i, 1-0.45 * i, 0.45])
        mesh_cylinder = mesh_cylinder.translate([0,0,0], relative=False)
        R = np.array([[1, 0, 0],[0, 0, 1],[0, 1, 0]])
        mesh_cylinder = mesh_cylinder.rotate(R)
        mesh_cylinder = mesh_cylinder.translate([parac[0], parac[1], parac[2]], relative=False)
        pts = [
            [parac[0], parac[1], parac[2]+parac[3]],
            [parac[0], parac[1], parac[2]-parac[3]],
            [parac[0]+parac[3], parac[1], parac[2]],
            [parac[0]-parac[3], parac[1], parac[2]],
        ]
        lines = [
            [0, 1],
            [0, 2],
            [0, 3],
            [1, 2],
            [1, 3],
            [2, 3],
        ]

        colors = [[1, 0, 0] for i in range(len(lines))]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(pts),
            lines=o3d.utility.Vector2iVector(lines),
        )

        # o3d.visualization.draw_geometries([cluster, mesh_cylinder])
        show.append(cluster)
        show.append(mesh_cylinder)
        vis.add_geometry("Points" + str(i), cluster)
        vis.add_geometry("Cylinder" + str(i), mesh_cylinder)
        vis.add_3d_label([parac[0], parac[1], parac[2] - parac[3]],  "x0 : " + str(parac[0]) + " z0 : " + str(parac[2]) + " r : " + str(parac[3]))

    # show[1] = show[1].translate([parac[0], parac[1], parac[2]], relative=False)
    r_diff = params[0][-1] - params[1][-1]
    print('r_diff', r_diff)
    vis.add_3d_label([params[0][0], params[0][1]+ params[0][3], params[0][2]], "r diff : " + str(r_diff))
    # o3d.visualization.draw_geometries(show)

    vis.show_settings = True
    # eye = [params[0][0], params[0][1], params[0][2]]
    # n = [0,0,1]
    vis.reset_camera_to_default()

    app.add_window(vis)
    app.run()

    return params

def main():

    # Load data
    file_path = 'data/clouds.pcd'
    point_cloud = o3d.io.read_point_cloud(file_path)
    print(len(point_cloud.points))

    # o3d.visualization.draw_geometries([point_cloud])

    clusters = cluster_point_cloud(point_cloud)
    params = fit(clusters)
    print(params)
    

if __name__ == '__main__':
    main()
 

# # x分量最大，则法向量必然与yz平面相交，轴点初值中x可设为0
# # cnv为公法线
# if cnv[0] >= cnv[1] and cnv[0] >= cnv[2]:   
#     tparac = optimize.leastsq(xclinerrors, [ym, zm, cnv[0], cnv[1], cnv[2], yrange/2], points,
#                                               full_output=1)
#     parac = np.array([0, tparac[0][0], tparac[0][1], tparac[0][2], tparac[0][3], tparac[0][4], tparac[0][5]])
#     # 圆度误差均值
#     ec = np.mean(np.abs(tparac[2]['fvec'])) / parac[6]
    
# if cnv[2] >= cnv[0] and cnv[2] >= cnv[1]:
#     tparac = optimize.leastsq(zclinerrors, [xm, ym, cnv[0], cnv[1], cnv[2], (xrange + yrange) / 2],
#                                              points,full_output=1)
#     parac = np.array([tparac[0][0], tparac[0][1], 0, tparac[0][2], tparac[0][3], tparac[0][4], tparac[0][5]])
#     ec = np.mean(np.abs(tparac[2]['fvec'])) / parac[6]
