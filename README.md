To Start
===

Build
---

    mkdir -p ~/kitti_ws
    cd ~/kitti_ws
    git clone https://github.com/zengy5/KITTI_PointCloud_MAP.git
    cd KITTI_PointCloud_MAP
    mkdir build
    cd build
    cmake ..
    make

Use your Data
---

open the kitti.yaml in config, modefy the "bin_file" to yout kitti velodyne file path, and also the "trajectory" to your pose file name.
Vel2Cam_R and Vel2Cam_t can be found in kitti calibration files, or just keep the default value.

Run the merge program
---

    cd ~/kitti_ws/KITTI_PointCloud_MAP/build/bin
    ./kitti

The result .pcd can be generated at ~/kitti_ws/KITTI_PointCloud_MAP/0.pcd
