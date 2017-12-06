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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
// エラー処理関係
#include <errno.h>
#include <sys/ioctl.h>

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
	:nh_("~"),
	it_(nh_)
  {
	mono_pub_ = it_.advertise("/mono_raw", 1);
  }

  ~ImageConverter()
  {
	cv::destroyAllWindows();
  }

  void imagePb(cv::Mat &image)
  {
	sensor_msgs::ImageConstPtr msg_mono =
	  cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
	
    // Output modified video stream
	mono_pub_.publish(msg_mono);
  }
};




int main(int argc,char** argv) {
  ros::init(argc, argv, "udp_reciever");
  //ros::NodeHandle nh;

  int sock, numrcv, val;
  struct sockaddr_in addr;
  
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  
  addr.sin_family = AF_INET;
  addr.sin_port = htons(12345);
  addr.sin_addr.s_addr = /*inet_addr("10.42.0.1");*/htonl(INADDR_ANY);

  // ソケットの生成
  bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  val = 1;
  ioctl(sock, FIONBIO, &val); // ノンブロッキング

  cvNamedWindow("Receive", CV_WINDOW_AUTOSIZE);
  cv::Mat image;// = cv::Mat(480,744,CV_8UC3);
  
  static const int receiveSize = 65500;
  static char buff[receiveSize];
  vector<uchar> ibuff;


  while(numrcv < 1){
	numrcv = recv(sock, buff, receiveSize, 0);
	if (errno == EAGAIN) {
	  cout << "Waiting for data." << endl;
	  sleep(1);
	} else {
	  cerr << "recv" << endl;
	  break;
	}
  }

  ImageConverter ic;
  
  while(ros::ok()){
	numrcv = recv(sock, buff, receiveSize, 0);

	for(int i=0; i<sizeof(buff) ; i++){
	  ibuff.push_back((uchar)buff[i]);
	}
	image = imdecode(Mat(ibuff), CV_LOAD_IMAGE_COLOR);//CV_LOAD_IMAGE_GRAYSCALE
	ibuff.clear();
	ic.imagePb(image);
  	
	cv::imshow("Receive", image);
	cv::waitKey(1);

	// escで終了
	if ((const char)cv::waitKey(1) == Esc) {
	  cv::destroyAllWindows();
	  break;
	}
	ros::spinOnce();
  }

  ibuff.clear();
  close(sock);
  cout << "Reciever Stoped" << endl;
  return 0;
}
