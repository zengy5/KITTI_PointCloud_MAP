#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

#include <Eigen/Dense>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>
#include <algorithm>

std::string yaml_path = "../../config/kitti.yaml";
std::string bin_file ;
std::string trajectory;
std::vector<double> Extrinc_R_temp;
std::vector<double> Extrinc_t_temp;
Eigen::Matrix3d Extrnic_R;
Eigen::Vector3d Extrnic_t;
Eigen::Matrix4d Extrnic = Eigen::Matrix4d::Identity();  // Vel2Cam

void loadConfig(std::string file_path)
{
    char* projectpath = NULL;
    projectpath = getcwd(NULL,0); 
    std::string path = projectpath;  
    file_path = path + "/" + file_path; 
    std::cout << "Load config form " << file_path << std::endl;

    cv::FileStorage read(file_path, cv::FileStorage::READ);
    std::string status;
    read["Status"] >> status;
    if(status.empty())
    {
        std::cout << "No File Path input, check the config.yaml" << std::endl;
        exit(0);
    }

    else std::cout << "Load success" << std::endl;
    read["bin_file"] >> bin_file;
    read["trajectory"] >> trajectory;
    read["Vel2Cam_R"] >> Extrinc_R_temp;
    read["Vel2Cam_t"] >> Extrinc_t_temp;

    std::cout << "PointCloud.bin files stored in " << bin_file << std::endl;
    std::cout << "Trajectory file path is " << trajectory << std::endl;
    std::cout << "Extrnic which Vel2Cam is" << std::endl;

    Extrnic_R << Extrinc_R_temp[0], Extrinc_R_temp[1], Extrinc_R_temp[2],
                Extrinc_R_temp[3], Extrinc_R_temp[4], Extrinc_R_temp[5],
                Extrinc_R_temp[6], Extrinc_R_temp[7], Extrinc_R_temp[8];

    Extrnic_t << Extrinc_t_temp[0], Extrinc_t_temp[1], Extrinc_t_temp[2];

    Extrnic.block<3,3>(0,0) = Extrnic_R;
    Extrnic.block<3,1>(0,3) = Extrnic_t;
    std::cout << Extrnic << std::endl;
    
}

int nameNum(const std::string& name)
{
    size_t start = name.find_first_of("0123456789");
    size_t end = name.find_last_of("0123456789");
    
    if (start != std::string::npos && end != std::string::npos)
    {
        std::string number = name.substr(start, end - start + 1);
        return std::stoi (number);
    }

    return 0;
}

void getFiles(std::string path, std::vector<std::string> & filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if (!(pDir = opendir(path.c_str())))
    {
        std::cout << "Folder path error, check if the / is right " << std::endl;
        return;
    }
    
    while ((ptr = readdir(pDir)) != 0)
    {
        if (strcmp(ptr->d_name,".") != 0 && strcmp(ptr->d_name, "..") != 0)
            filenames.push_back(ptr->d_name);
    }

    closedir(pDir);

    std::sort(filenames.begin(), filenames.end(), [](const std::string &a, const std::string &b)
    {
        return nameNum(a) < nameNum(b);
    });
    
}

pcl::PointCloud<pcl::PointXYZI> bin2pcd(std::string bin_path)
{
    pcl::PointCloud<pcl::PointXYZI> final_pcd;
    int32_t num = 1000000;
    float *data = (float*)malloc(num * sizeof(float));

    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;

    std::FILE *stream;
    
    stream = fopen(bin_path.c_str(), "r");
    num = fread(data,sizeof(float),num,stream)/4;
    pcl::PointXYZI point;
    for (int32_t i = 0; i < num; i++) {
        point.x = *px;
        point.y = *py;
        point.z = *pz;
        point.intensity = *pr;
        final_pcd.push_back(point);
        px+=4; py+=4; pz+=4; pr+=4;
    }
    fclose(stream);
    free(data);

    return final_pcd;
}


void show_progress(float progress,float all)
{
    if (progress > 1) progress = 1;

    int pa = progress * 50;
    std::cout << "\33[1A";
    std::cout << "[" + std::string(pa,'=') + ">" + std::string(50 - pa, ' ') << "]" << std::to_string(int16_t( progress*100.0))
                << '%' <<std::endl;
    fflush(stdout);
}


int main(int argc, char ** argv)
{
    loadConfig(yaml_path);
    std::vector<std::string> bins;
    Eigen::Matrix4d Extrnic_Inv = Extrnic.inverse().eval(); // Cam2Vel
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>());

    std::ifstream input_file(trajectory);
    if(!input_file.is_open())
    {
        std::cerr << "Trajectory file open failed at " << trajectory << std::endl;
        return 1;
    }

    std::vector<std::vector<double>> poses12; // 3*4 matrix

    std::string line;
    while (std::getline(input_file, line))
    {
        std::vector<double> row;
        std::istringstream ss(line);
        std::string token;

        while (std::getline(ss, token, ' '))
        {
            row.push_back(std::stod(token));
        }

        poses12.push_back(row);
        
    }
    getFiles(bin_file, bins);

    if (bins.size() != poses12.size()) 
    {
        std::cout << "Pose num doesn't equal to bins \n"; 
    }
    
    else std::cout << "Load " << bins.size() << " bins and poses\n";

    int count = 0;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    for (size_t i = 0; i < bins.size(); i++ )
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        std::vector<double> pose_now = poses12[i];
        std::string bin_name = bin_file + "/" + bins[i];

        *temp_cloud = bin2pcd(bin_name);
        voxel_grid.setInputCloud(temp_cloud);
        float leafsize = 0.8f;
        voxel_grid.setLeafSize(leafsize,leafsize,leafsize);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        voxel_grid.filter(*filtered);

        pose << pose_now[0], pose_now[1], pose_now[2], pose_now[3],
                pose_now[4], pose_now[5], pose_now[6], pose_now[7],
                pose_now[8], pose_now[9], pose_now[10], pose_now[11],
                0.0,         0.0,         0.0,          1.0;

        pose = (Extrnic_Inv * pose * Extrnic).eval();
        pcl::transformPointCloud(*filtered, *filtered, pose);

        *global_map += *filtered;

        temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        filtered.reset(new pcl::PointCloud<pcl::PointXYZI>());
        float pro = (i+1) * 1.0/bins.size();
        show_progress(pro,bins.size());

    }
    std::string pcd_save_path = "../../" + std::to_string(count) + ".pcd";
    pcl::io::savePCDFile(pcd_save_path, *global_map);
    return 0;
}
