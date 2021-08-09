#include <stdio.h>
#include <opencv2/opencv.hpp>

int main(void){
  cv::Mat img, yuv_img;
  
  img = cv::imread("images/0000.jpg");
  
  cv::cvtColor(img, yuv_img, cv::COLOR_BGR2YUV);
  
  cv::imshow("original", img);
  cv::imshow("YUV", yuv_img);
  
  cv::waitKey(0);
  cv::destroyWindow("original");
  return 0;
}
