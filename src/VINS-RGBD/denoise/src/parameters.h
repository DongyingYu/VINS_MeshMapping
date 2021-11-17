#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>


extern std::string IMAGE_TOPIC;
extern std::vector<std::string> classes;


void readParameters(ros::NodeHandle &n);




