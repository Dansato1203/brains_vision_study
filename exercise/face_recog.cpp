#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

int main()
{
    cv::CascadeClassifier cascade;
    cascade.load("xml/haarcascade_frontalface_alt.xml");
    cv::Mat img = cv::imread("images/0001.png");
    std::vector<cv::Rect> faces; 
    cascade.detectMultiScale(img, faces, 1.1, 3, 0, cv::Size(20, 20));
    for(int i = 0; i<faces.size(); i++)
    {
        cv::rectangle(img, cv::Point(faces[i].x, faces[i].y), cv::Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), cv::Scalar(0, 0, 255), 3, CV_AA);
        }

    cv::imshow("detect face", img);
    cv::waitKey(0);
}
