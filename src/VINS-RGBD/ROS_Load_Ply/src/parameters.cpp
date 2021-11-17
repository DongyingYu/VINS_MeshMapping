#include "parameters.h"



std::string addressfile;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    // config_file = readParam<std::string>(n, "config_file");
    config_file = "/home/ipsg/1295/VINS_MeshMapping/src/VINS-RGBD/config/realsense/realsense_live_config.yaml";
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    // std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["addressFile_path"] >> addressfile;

    
    std::ifstream ifs(addressfile.c_str());
    std::string line;
    //for (int i = 0; i < classes.size(); i++)
    //{
    //	cout << classes[i] << endl;
    //}

    fsSettings.release();
}
