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
#include <camera_info_manager/camera_info_manager.h>
#include <sys/time.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
public:
  
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher color_pub_;
  image_transport::Publisher mono_pub_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  image_transport::CameraPublisher info_pub_;

  std::string camera_info_url, camera_name;
  std::string link_name;

  //const char *device_name;
  unsigned char* ImageBuffer;
  cv::Mat src;
  cv::Mat dst;
  cv::Mat mono;

  ImageConverter()
	:nh_("~"),
	 it_(nh_),
	 cinfo_(new camera_info_manager::CameraInfoManager(nh_))
  {
    //color_pub_ = it_.advertise("/v4l_tis/image_raw", 1);
	//mono_pub_ = it_.advertise("/v4l_tis/mono_raw", 1);
	info_pub_ = it_.advertiseCamera("/v4l_tis/image_raw", 1);
    //cv::namedWindow(OPENCV_WINDOW);

	
	nh_.param("camera_info_url", camera_info_url, std::string("file:///home/kazuhiro/ROS/svo_ws/src/v4l_tis/ost.yaml"));
	nh_.param("camera_name", camera_name, std::string("narrow_stereo"));
	nh_.param("link_name", link_name, std::string("camera"));
	//cinfo_.reset(new camera_info_manager::CameraInfoManager(nh_, "v4l_tis", camera_info_url));
	//cinfo_.reset(new camera_info_manager::CameraInfoManager(nh_));
	cinfo_->setCameraName(camera_name);
	cinfo_->loadCameraInfo(camera_info_url);
	

    const char *device_name = "/dev/video0";
	open_device(device_name); 
	init_device(744, 480, 50, 63, 60); //width,height,exposure,gain,fps
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
	  //cvtColor(dst, mono, CV_RGB2GRAY);
	} else {
	  std::cerr << "no image buffer retrieved!" << std::endl;
	}
	
	//cv::imshow(OPENCV_WINDOW, dst);
	sensor_msgs::ImagePtr msg =
	  cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
	// sensor_msgs::ImagePtr msg_mono =
	//   cv_bridge::CvImage(std_msgs::Header(), "mono8", mono).toImageMsg();

	sensor_msgs::CameraInfoPtr ci_(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
	
	ci_->header.frame_id = link_name;
	msg->header.frame_id = link_name;
	
	
    // Output modified video stream
    //color_pub_.publish(msg);
	//mono_pub_.publish(msg_mono);
	info_pub_.publish(msg, ci_);

	//cv::waitKey(1);
  }
};


int main(int argc,char** argv) {
  ros::init(argc, argv, "v4l_tis");
  //ros::NodeHandle nh;
  ImageConverter ic;
  ros::Rate loop(1);				// Hz
  struct timeval s, e;

  while(ros::ok()){
    gettimeofday(&s, NULL);
	ic.imagePb();
	ros::spinOnce();
	//loop.sleep();

	gettimeofday(&e, NULL);
    //cout << 1.0/((e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6) << "Hz." << endl;
    
  }
  return 0;
}
