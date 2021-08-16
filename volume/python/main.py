import cv2
import math
import random
import open3d as o3d
import numpy as np


def get_pcd():
    img_id = 94
    rows = 480
    cols = 640
    channels =1
    depth_img = np.fromfile('images\\depth_%d.raw'%(img_id), dtype='uint16')
    depth_img = depth_img.reshape(rows, cols, channels)
    mask_img = cv2.imread('images\\Mask_%d.png'%(img_id))

    fx = 660
    fy = 660
    cx = 0
    cy = 0
    depth_scale = 1200

    points = np.zeros([rows * cols, 3])
    labels = np.zeros([rows * cols, 3])
    # points = np.random.rand(rows * cols, 3)
    k = 0

    colormap = np.array([[0, 0, 0],
                         [0, 0, 255],
                         [0, 255, 0],
                         [255, 0, 0],
                         [0, 128, 128]])

    area_num = np.zeros(5)
    index_list = [[], [], [], [], []]
    for i in range(0,  rows):
        for j in range(0,  cols):
            z = depth_img[i][j] / depth_scale
            x = (j - cx) * z / fx
            y = (i - cy) * z / fy
            points[k][0] = x
            points[k][1] = y
            points[k][2] = z
            for t in range(5):
                if (mask_img[i][j][:] == colormap[t]).all():
                    area_num[t] += 1
                    labels[k] = t
                    index_list[t].append(k)
                    break
            k += 1

    # print(np.shape(points))
    # print(area_num)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    # o3d.visualization.draw_geometries([pcd])
    # o3d.io.write_point_cloud("point94.ply", pcd)
    return pcd, index_list


def get_pcd2():
    pcd = o3d.io.read_point_cloud("pointcloud\\point95_new.ply")
    return pcd


if __name__ == "__main__":
    colormap = np.array([[0, 0, 0],
                         [0, 0, 255],
                         [0, 255, 0],
                         [255, 0, 0],
                         [0, 128, 128]])

    pcd, index_list = get_pcd()
    pcd2 = get_pcd2()
    plane_para, plane_index = pcd2.segment_plane(distance_threshold=0.01, ransac_n=10, num_iterations=1000)
    plane_cloud = pcd2.select_by_index(plane_index)
    plane_cloud.paint_uniform_color([0, 0, 0])
    o3d.visualization.draw_geometries([plane_cloud])
    # ret = [i for i in index_list[0] if i not in inliers]
    # index_list[0] = ret
    # outlier_cloud = pcd.select_by_index(inliers, invert=True)
    # outlier_cloud.paint_uniform_color([0, 1, 0])
    # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    list_cloud = []
    for t in range(5):
        list_cloud.append(pcd.select_by_index(index_list[t]))
    for t in range(5):
        list_cloud[t].paint_uniform_color(colormap[t] / 255)

    # o3d.visualization.draw_geometries([list_cloud[0], list_cloud[1], list_cloud[2], list_cloud[3], list_cloud[4]])
    # o3d.visualization.draw_geometries([plane_cloud, list_cloud[1], list_cloud[2], list_cloud[3], list_cloud[4]])


    volume_num = np.zeros(5)
    for t in range(1, 5):
        for i in index_list[t]:
            x = pcd.points[i][0]
            y = pcd.points[i][1]
            z = pcd.points[i][2]
            m = plane_para[0] * plane_para[0] + plane_para[1] * plane_para[1] + plane_para[2] * plane_para[2]
            s = (plane_para[0] * x + plane_para[1] * y + plane_para[2] * z + plane_para[3]) ** 2
            d = math.sqrt(s / m)
            volume_num[t] += d
            # print(x, y, z, d)

    p = (volume_num[1] / 260 + volume_num[2] / 260) / 2
    print(p)
    volume_num = np.divide(volume_num, p)

    # print(plane_para)
    # print(area_num)
    print("volume of objects : (milk_up, milk_down, object_left, object_right)")
    print(volume_num[1:])
