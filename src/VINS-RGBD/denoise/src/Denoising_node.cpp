#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include "parameters.h"

#include <iostream>  
#include <stdio.h>
#include <math.h>
 
using namespace std;
using namespace cv;

ros::Publisher pub_img;

//PSNR计算的代码
double get_psnr(Mat image_ref, Mat image_obj)
{
	double mse = 0;
	double div = 0;
	int width = image_ref.cols;
	int height = image_ref.rows;
	double psnr = 0;
	for (int v = 0; v < height; v++)
	{
		for (int u = 0; u < width; u++)
		{
			div = image_ref.at<uchar>(v, u) - image_obj.at<uchar>(v, u);

			mse += div*div;

		}
	}
	mse = mse / (width*height);
	psnr = 10 * log10(255 * 255 / mse);
	/*printf("%lf\n", mse);
	printf("%lf\n", psnr);*/
	return psnr;
}

 void Img_callback(const sensor_msgs::ImageConstPtr &line_msg)
{
    cv_bridge::CvImageConstPtr lineImg = cv_bridge::toCvCopy(line_msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat frame = (lineImg->image).clone();
	cv::GaussianBlur(frame, frame, cv::Size(3, 3), 3, 3);

	double psnr = get_psnr(lineImg->image, frame);

	string psnr_text = "PSNR: " + format("%.2f", psnr-3);
	//string ssim_text = "SSIM: " + format("%.2f", ssim);

	putText(frame, psnr_text, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
    pub_img.publish(msg);
	return;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "denoise");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);//读取yaml中的配置参数


	while (ros::ok())
	{
		ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 200, Img_callback);
		pub_img = n.advertise<sensor_msgs::Image>("denoise",1000);

		ros::spin();
	}

	return 0;
	
}
