import open3d as o3d
import numpy as np
import math


def get_distance2(a, b):
    return (a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2]) **2

if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("points.ply")
    # o3d.visualization.draw_geometries([pcd])
    points = np.asarray(pcd.points)
    labels = np.zeros([points.shape[0], 1], int)
    print(points.shape)
    print(labels.shape)
    k = 10
    colormap = np.array([
                     [0, 0, 255],
                     [0, 255, 0],
                     [255, 0, 0],
                     [0, 128, 128],
                     [128, 0, 128],
                     [128, 128, 0],
                     [100, 0, 0],
                     [0, 100, 0],
                     [0, 0, 100],
                     [155, 50, 100],
                     ])
    colormap = np.divide(colormap, 255)
    init_id = np.random.randint(0, points.shape[0], k)
    print(init_id)
    core = points[init_id, :]
    print(core)

    for epoch in range(20):
        for i in range(points.shape[0]):
            label_i = 0
            min_distance = float('inf')
            for j in range(k):
                curr_distance = get_distance2(points[i], core[j])
                if curr_distance < min_distance:
                    label_i = j
                    min_distance = curr_distance
            labels[i] = label_i
        # print(labels)

        cluster_index_list = [[]] * k
        cluster_cloud_list = [[]] * k
        for j in range(k):
            cluster_index_list[j] = np.where(labels == j)[0]
            # print("index")
            # print(cluster_index_flist[j])
            cluster_cloud_list[j] = pcd.select_by_index(cluster_index_list[j])
            cluster_cloud_list[j].paint_uniform_color(colormap[j])
            core[j] = np.reshape(np.mean(points[cluster_index_list[j], :], axis=0), (1, 3) )
            # print(np.shape(a))
    # o3d.visualization.draw_geometries([cluster_cloud_list[0], cluster_cloud_list[1],cluster_cloud_list[2]])
    # o3d.visualization.draw_geometries([cluster_cloud_list[0], cluster_cloud_list[1],cluster_cloud_list[2], cluster_cloud_list[3],cluster_cloud_list[4], cluster_cloud_list[5]])
    # o3d.visualization.draw_geometries([cluster_cloud_list[0], cluster_cloud_list[1],cluster_cloud_list[2], cluster_cloud_list[3],cluster_cloud_list[4], cluster_cloud_list[5],cluster_cloud_list[6],cluster_cloud_list[7]])
    # o3d.visualization.draw_geometries([cluster_cloud_list[0], cluster_cloud_list[1],cluster_cloud_list[2], cluster_cloud_list[3],cluster_cloud_list[4], cluster_cloud_list[5],cluster_cloud_list[6],cluster_cloud_list[7]])
    o3d.visualization.draw_geometries([cluster_cloud_list[0], cluster_cloud_list[1],cluster_cloud_list[2], cluster_cloud_list[3],cluster_cloud_list[4], cluster_cloud_list[5],cluster_cloud_list[6],cluster_cloud_list[7],cluster_cloud_list[8],cluster_cloud_list[9]])
    # o3d.visualization.draw_geometries([cluster_cloud_list])




