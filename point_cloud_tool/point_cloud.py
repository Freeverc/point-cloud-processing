import open3d as o3d
import numpy as np

def downsample_point_cloud(point_cloud, voxel_size=0.05):
    """
    对点云进行降采样
    :param point_cloud: 输入的点云
    :param voxel_size: 体素大小
    :return: 降采样后的点云
    """
    return point_cloud.voxel_down_sample(voxel_size)

def icp_registration(source, target, threshold=1.0):
    """
    使用ICP算法对两个点云进行配准
    :param source: 源点云
    :param target: 目标点云
    :param threshold: 最大对应点对距离
    :return: 配准后的点云和变换矩阵
    """
    trans_init = np.eye(4)  # 初始化为单位矩阵
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    return reg_p2p.transformation

def preprocess_point_cloud(point_cloud, voxel_size):
    """
    预处理点云：降采样和估计法线
    """
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    point_cloud_down = point_cloud.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    point_cloud_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    point_cloud_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        point_cloud_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return point_cloud_down, point_cloud_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    """
    执行全局配准
    """
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4, 
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result

def merge_point_clouds(point_clouds, voxel_size=0.05, threshold=1.0):
    """
    合并多个点云
    :param point_clouds: 点云列表
    :param voxel_size: 体素大小，用于降采样
    :param threshold: ICP算法的最大对应点对距离
    :return: 合并后的点云
    """
    if not point_clouds:
        return None
    
    # 对第一个点云进行降采样
    merged = downsample_point_cloud(point_clouds[0], voxel_size)
    
    for i in range(1, len(point_clouds)):
        # 对后续的点云也进行降采样
        pc = downsample_point_cloud(point_clouds[i], voxel_size)
        # 使用ICP算法进行配准
        transformation = icp_registration(pc, merged, threshold)
        # 应用变换矩阵
        pc.transform(transformation)
        # 合并点云
        merged += pc
    
    # 对合并后的点云再次降采样以去除重叠部分
    merged = downsample_point_cloud(merged, voxel_size)
    return merged

# 示例：加载点云并合并
if __name__ == "__main__":
    # 假设有两个点云文件"cloud1.pcd"和"cloud2.pcd"
    cloud1 = o3d.io.read_point_cloud("data/points1_mini.ply")
    cloud2 = o3d.io.read_point_cloud("data/points2_mini.ply")
    
    # 合并点云
    merged_cloud = merge_point_clouds([cloud1, cloud2], voxel_size=0.05, threshold=1.0)
    
    # 可视化合并后的点云
    o3d.visualization.draw_geometries([merged_cloud])
