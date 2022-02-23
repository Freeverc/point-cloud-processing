from decimal import Decimal
import numpy as np
import open3d as o3d
import cv2 as cv
import csv

if __name__ == '__main__':
    print('Converting')
    point_cloud_file_name = 'pin-image2/replay_78067_2022-2-21 (5).csv'
    intensity_file_name = 'pin-image2/replay_78067_2022-2-21.bmp'

    intensity = cv.imread(intensity_file_name)
    intensity_color = cv.applyColorMap(intensity, cv.COLORMAP_JET)
    print(np.shape(intensity))

    f = csv.reader(open(point_cloud_file_name, 'r'))
    x_list = []
    y_list = []
    points = []
    colors = []
    found_data = False
    y_i = 0
    for line in f:
        if len(line) == 0:
            continue
        if line[0] == r'End':
            found_data = False

        if found_data:
            if len(line) != len(x_list) + 1:
                print("wrong data")
                print("line len : ", len(z_list))
                continue
            y = Decimal(line[0])
            y_list.append(y)
            z_list = [(Decimal(i) if i != '' else 0) for i in line[1:]]
            for x_i in range(len(z_list)):
                points.append([x_list[x_i], y, z_list[x_i]])
                colors.append([intensity_color[y_i, x_i, 0],
                               intensity_color[y_i, x_i, 1],
                               intensity_color[y_i, x_i, 2]])
            y_i += 1
            if y_i % 500 == 0:
                print(y_i)
        else:
            pass

        if line[0] == r'Y\X':
            x_list = [Decimal(i) for i in line[1:]]
            found_data = True
            print("x len : ", len(x_list))

    print(len(y_list))
    print(len(points))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    o3d.io.write_point_cloud(point_cloud_file_name.replace('csv', 'ply'), pcd)
    down_sampled_pcd = pcd.voxel_down_sample(voxel_size=0.05)
    o3d.visualization.draw_geometries([down_sampled_pcd], window_name='Point cloud')



