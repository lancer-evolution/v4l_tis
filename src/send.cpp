#define Esc 27
	
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "v4ldevice.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher color_pub_;
  image_transport::Publisher mono_pub_;
  

  //const char *device_name;
  unsigned char* ImageBuffer;
  cv::Mat src;
  cv::Mat dst;
  cv::Mat mono;

public:
  ImageConverter()
    : it_(nh_)
  {
    color_pub_ = it_.advertise("/v4l_tis/image_raw", 1);
	mono_pub_ = it_.advertise("/v4l_tis/mono_raw", 1);
    cv::namedWindow(OPENCV_WINDOW);

    const char *device_name = "/dev/video0";
	open_device(device_name); 
	init_device(744, 480, 50); //キャプチャするサイズと露出を指定する．
    start_capturing();

    //バッファーとMatクラスの準備
    ImageBuffer = NULL;
    src.create(480, 744, CV_8UC1);
  }

  ~ImageConverter()
  {
	cv::destroyAllWindows();
  }

  void imagePb()
  {
	ImageBuffer = snapFrame();
	if (ImageBuffer != NULL) {
	  memcpy(src.data, ImageBuffer, src.step * src.rows); //バッファーをMatクラスに受け渡す．
	  cvtColor(src, dst, CV_BayerGB2RGB); //ここがBayer変換
	  cvtColor(dst, mono, CV_RGB2GRAY);
	} else {
	  std::cerr << "no image buffer retrieved!" << std::endl;
	}

	cv::imshow(OPENCV_WINDOW, dst);
	sensor_msgs::ImageConstPtr msg =
	  cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
	sensor_msgs::ImageConstPtr msg_mono =
	  cv_bridge::CvImage(std_msgs::Header(), "mono8", mono).toImageMsg();
	
    // Output modified video stream
    color_pub_.publish(msg);
	mono_pub_.publish(msg_mono);

	
	cv::waitKey(1);
  }
};


int main(int argc,char** argv) {
  ros::init(argc, argv, "v4l_tis");
  ros::NodeHandle nh;
  ImageConverter ic;

  while(ros::ok()){
	ic.imagePb();
	ros::spinOnce();
  }
  return 0;
}
