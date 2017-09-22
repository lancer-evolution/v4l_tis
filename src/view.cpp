#define Esc 27
	
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "v4ldevice.h"

using namespace std;

int main(int argc,char** args) {
    //カメラの初期化
  const char *device_name = "/dev/video0";
  open_device(device_name); //ここは，PCに接続したカメラの場所を指定する．１台しか接続しない場合は，通常は0番にカメラは接続されるので，このパスとなる．
  init_device(744, 480, 50); //キャプチャするサイズと露出を指定する．

  
    //キャプチャの開始
    start_capturing();

    //バッファーとMatクラスの準備
    unsigned char* ImageBuffer = NULL;
    cv::Mat src(480, 744, CV_8UC1); //8-bit, 3チャンネルのMatクラスを用意する．配列のサイズはカメラ初期化で指定したものと同じサイズにするが，widthとheightが逆になるので，注意する．もしも逆にすると，表示される映像は縦に幾つか割れ目の入ったような映像となる．
	cv::Mat dst; //bayer変換後のデータを格納するMatクラスを用意する．

    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    while (true) {

        //キャプチャ
        ImageBuffer = snapFrame();
        if (ImageBuffer != NULL) {
            memcpy(src.data, ImageBuffer, src.step * src.rows); //バッファーをMatクラスに受け渡す．
            cvtColor(src, dst, CV_BayerGB2RGB); //ここがBayer変換
        } else {
            std::cerr << "no image buffer retrieved!" << std::endl;
        }
        cv::imshow("image", src);
        if ((const char)cv::waitKey(1) == Esc) {
            cv::destroyAllWindows();
            break;
        }
		//cout << src << endl;
		//cv::waitKey(0);
    }

}
