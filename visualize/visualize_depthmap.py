import open3d as o3d
import numpy as np
import argparse


def main():
    parser = argparse.ArgumentParser(description='Visualize point cloud :')
    parser.add_argument('--point_cloud_path',
                        type=str,
                        default='./data/lecturehall.ply',
                        help='file path for initial point cloud')
    parser.add_argument('--vis_point_cloud_path',
                        type=str,
                        default='./results/visualize.ply',
                        help='file path for visualized point cloud')
    parser.add_argument('--depth_map_path',
                        type=str,
                        default='./results/depth.png',
                        help=' file path for image visualized by distance')
    parser.add_argument('--inverse_front',
                        type=bool,
                        default=False,
                        help='inverse look direction or not')
    parser.add_argument('--show_plane',
                        type=bool,
                        default=True,
                        help='show plane or not')
    parser.add_argument('--use_plane_distance',
                        type=bool,
                        default=True,
                        help='Use plane distance or not')
    parser.add_argument('--camera_distance_factor',
                        type=float,
                        default=3,
                        help='camera distance factor: 0.5 - 3')
    args = parser.parse_args()

    # Load data
    print("Loading data : ")
    pcd = o3d.io.read_point_cloud(args.point_cloud_path)

    # Detecting planes
    print("Detecting planes : ")
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.1,
                                             ransac_n=10,
                                             num_iterations=200)
    print(plane_model)

    # Setting Front, up and look point
    sum_square = np.sqrt(
        np.square(plane_model[0]) + np.square(plane_model[0]) +
        np.square(plane_model[0]))
    front_vector = [
        plane_model[0] / sum_square, plane_model[1] / sum_square,
        plane_model[2] / sum_square
    ]
    if args.inverse_front:
        front_vector = [-front_vector[0], -front_vector[1], -front_vector[2]]
    up_vector = [front_vector[1], -front_vector[0], 0]

    min_bound = pcd.get_min_bound()
    max_bound = pcd.get_max_bound()
    range_x = max_bound[0] - min_bound[0]
    range_y = max_bound[1] - min_bound[1]
    range_z = max_bound[2] - min_bound[2]
    metric = max(range_x, range_y, range_z)
    print("metric : ", metric)
    look_at_vector = pcd.get_center()
    camera_distance = metric * args.camera_distance_factor
    camera_position = [
        look_at_vector[0] + camera_distance * front_vector[0],
        look_at_vector[1] + camera_distance * front_vector[1],
        look_at_vector[2] + camera_distance * front_vector[2]
    ]

    print("front : ", front_vector)
    print("up : ", up_vector)
    print("min_bound : ", min_bound)
    print("max_bound : ", max_bound)
    print("look_at_vector : ", look_at_vector)
    print("camera : ", camera_position)

    # Calculating distance.
    print("Calculating distance.")
    distance_list = np.zeros(len(pcd.points))
    i = 0
    for point in pcd.points:
        if args.use_plane_distance:
            d = (plane_model[0] * point[0] + plane_model[1] * point[1] +
                 plane_model[2] * point[2] + plane_model[3]) / sum_square
        else:
            d = np.sqrt(
                np.square(point[0] - camera_position[0]) +
                np.square(point[1] - camera_position[1]) +
                np.square(point[2] - camera_position[2]))
        distance_list[i] = d
        i += 1
        # print(point, d)
    min_distance = np.min(distance_list)
    max_distance = np.max(distance_list)
    print("Distance range : ", min_distance, max_distance)

    # Generating colors.
    print("Generating colors.")
    i = 0
    pcd.colors.clear()
    for point in pcd.points:
        c = (distance_list[i] - min_distance) / (max_distance - min_distance)
        color = [c, c, c]
        pcd.colors.append(color)
        i += 1
    print(pcd.colors)
    o3d.io.write_point_cloud(args.vis_point_cloud_path, pcd, True)

    # Show plane.
    if args.show_plane:
        inlier_cloud = pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        outlier_cloud.paint_uniform_color([0, 1, 0])
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                          zoom=1.0,
                                          front=front_vector,
                                          lookat=look_at_vector,
                                          up=up_vector)

    # Show result.
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    view_ctr = vis.get_view_control()
    view_ctr.set_front(front_vector)
    view_ctr.set_lookat(look_at_vector)
    view_ctr.set_up(up_vector)
    view_ctr.set_zoom(0.5)
    view_ctr.change_field_of_view(-20)
    render = vis.get_render_option()
    render.background_color = [0, 0, 0]
    vis.run()
    vis.capture_screen_image(args.depth_map_path)
    vis.destroy_window()


if __name__ == "__main__":
    main()
