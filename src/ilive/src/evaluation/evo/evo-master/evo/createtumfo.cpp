#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_types.h>                 //pcl点云格式头文件
#include <pcl_conversions/pcl_conversions.h> //转换
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "Eigen/Core"
#include <boost/foreach.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <stdlib.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

#define foreach BOOST_FOREACH
using namespace std;


std::string  txt_path;
struct TRAJECTORY
{
    double time;
    double position_x;
    double position_y;
    double position_z;
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
};

void read_trajectory_file(std::string filepathIn)
{
    std::ifstream inFile(filepathIn);
    std::ofstream traj_ofs;
    traj_ofs.open( txt_path.c_str(), std::ios::app);
    if (!inFile)
    {
        cout << "open write file fail!" << endl;
    }
    // temp v
    char content[2000];
    std::string content_str = "";

    while (!inFile.eof())
    {
        content_str = "";
        inFile.getline(content, 2000);
        if (strlen(content) < 2)
            break;

        std::istringstream _Readstr(content);
        std::string partOfstr;
        double data[9];

        for (int i = 0; i < 9; i++)
        {
            getline(_Readstr, partOfstr, ' ');
            data[i] = strtold(partOfstr.c_str(), NULL);
        }

        TRAJECTORY trajectory;
        trajectory.time = data[0];
        trajectory.position_x = data[1];
        trajectory.position_y = data[2];
        trajectory.position_z = data[3];
        trajectory.orientation_w = data[4];
        trajectory.orientation_x = data[5];
        trajectory.orientation_y = data[6];
        trajectory.orientation_z = data[7];

        traj_ofs << std::setprecision(16) <<  trajectory.time << "," << trajectory.position_x << "," <<  trajectory.position_y<< "," <<  trajectory.position_z << "," << trajectory.orientation_w << "," << trajectory.orientation_x << "," << trajectory.orientation_y << "," << trajectory.orientation_z<<std::endl;
    }
    inFile.close();
    traj_ofs.close();
    cout << "read trajectory file end!" << endl;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "predict_odom_real2.txt");
    ros::NodeHandle nh;

    string traj_path = "";
    nh.param<string>("input_trajectory_path", traj_path, "/home/slam/workspace/ws_ILIVE/Evaluation_Data/evo/evo-master/test/data/");
    nh.param<string>("output_txt_path", txt_path, "/home/slam/workspace/ws_ILIVE/Evaluation_Data/evo/evo-master/test/data/");
    read_trajectory_file(traj_path);

    return 0;
}