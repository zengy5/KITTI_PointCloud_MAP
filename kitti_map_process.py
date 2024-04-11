import open3d as o3d 
import cv2
import numpy as np
import os
import sys

ext = np.float64([
    [0.0, -1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0, -0.08],
    [0.0, 0.0, 0.0, 1.0]
]) # vel2cam

ext_inv = np.linalg.inv(ext) #cam2vel

test_therhold = 200

def get_pcd(bin_name):
    pointcloud = np.fromfile(str(bin_name), dtype=np.float32, count=-1).reshape([-1,4])
    x = pointcloud[:,0]
    y = pointcloud[:,1]
    z = pointcloud[:,2]
    
    pcd = o3d.geometry.PointCloud()
    pc = np.column_stack((x,y,z))
    pcd.points = o3d.utility.Vector3dVector(pc)
    return pcd

def write_pose(pose_txt, pose_result):
    poses = open(pose_txt,"r").readlines()
    for pose in poses:
        line = pose.strip(" ")
        line = pose.split()
        pose_matrix = np.float64([
            [line[0], line[1], line[2], line[3]],
            [line[4], line[5], line[6], line[7]],
            [line[8], line[9], line[10], line[11]],
            [0.0, 0.0, 0.0, 1.0]
        ])
        pose_matrix = ext_inv @ pose_matrix @ ext
        matrix_34 = pose_matrix[:3,:]
        
        with open(pose_result,'a') as f:
            # To save as the kitti form which can directly used to evo_traj
            matrix_str = ' '.join(['{:.6e}'.format(num) for num in matrix_34.flatten()]).rstrip()
            f.write(matrix_str + '\n')

def save_as_pcds(pcd_save_path, bin_stored_path, check_mode=False):
    if not os.path.exists(pcd_save_path):
        os.makedirs(pcd_save_path)
    if not os.path.exists(bin_stored_path):
        print("No bins in"+bin_stored_path)
        sys.exit()

    if check_mode:
        limit = test_therhold
    bins = os.listdir(bin_stored_path)
    bins.sort(key=lambda x:int(x[:-4]))
    i = 0
    for bin in bins:
        bin_name = bin_stored_path + "/" + bin
        pcd_name = pcd_save_path + "/" + str(i) + ".pcd"
        pcd = get_pcd(bin_name)
        i += 1
        o3d.io.write_point_cloud(pcd_name, pcd)
        if check_mode:
            if i > limit:
                break
            
def test_result(pose_result, pcd_save_path, save=False):
    poses_34 = open(pose_result,"r").readlines()
    poses = []
    for pose in poses_34:
        line = pose.strip(" ")
        line = pose.split()
        pose_matrix = np.float64([
            [line[0], line[1], line[2], line[3]],
            [line[4], line[5], line[6], line[7]],
            [line[8], line[9], line[10], line[11]],
            [0.0, 0.0, 0.0, 1.0]
        ])
        poses.append(pose_matrix)
    
    pcds = os.listdir(pcd_save_path)
    pcds.sort(key=lambda x:int(x[:-4]))
    merged_point_cloud = o3d.geometry.PointCloud()
    count = 0
    for i, pcd_file in enumerate(pcds):
        point_cloud = o3d.io.read_point_cloud(pcd_save_path + "/"+pcd_file)
        point_cloud = point_cloud.random_down_sample(0.3)
        
        pose = poses[i]
        
        point_cloud.transform(pose)
        merged_point_cloud += point_cloud
        count += 1
        if count == test_therhold:
            break
    if save:
        merge_pcd_save_path = '/home/imr/icp_data/keyframe/02/a.pcd'
        o3d.io.write_point_cloud(merge_pcd_save_path, merged_point_cloud)
    o3d.visualization.draw_geometries([merged_point_cloud])


if __name__ == "__main__":
    # where the kitti bins are stored
    bin_path_name = "/home/imr/dataset/official_data/KITTI/data_odometry_velodyne/dataset/sequences/02/velodyne"
    # where the kitti pose is stored
    pose_txt = "/home/imr/dataset/official_data/KITTI/data_odometry_poses/dataset/poses/02.txt"
    # where the pose of lidar you want to save in
    pose_result = "/home/imr/icp_data/keyframe/02/poses.txt"  
    # where the pcds transformed from raw bins you want to save in
    pcd_save_path = "/home/imr/icp_data/keyframe/02/kitti_pcds"

    
    write_pose(pose_txt, pose_result)
    save_as_pcds(pcd_save_path, bin_path_name)
    test_result(pose_result, pcd_save_path)

    