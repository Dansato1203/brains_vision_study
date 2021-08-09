#include <iostream>
#include <fstream>
#include <algorithm>
#include <fcntl.h>
#include <cstdlib>

#include "image_grabber_v4l.h"
#include "v4l/video_adjust.h"
#include "v4l/video_adjust_KBCRM05VU.h"
#include "v4l/video_adjust_KBCRS02MU.h"
#include "v4l/v4l2capture.h"

static camera_parameter *cam_para = NULL;

static v4l2capture v4l2cap;

// Camera ID
constexpr int CAMERA_KBCRM05VU = 3; // 640 x 480
constexpr int CAMERA_KBCRS02MU = 4; // 1280 x 1024

ImageGrabberV4l::ImageGrabberV4l(int in_id, const int w, const int h) : ImageGrabber(in_id, w, h), cap_width(0), cap_height(0), capture_failed_count(0), iauto(0), camera_bgr_filename("camera_bgr.txt"), bgr_param_num(5 /* shutter speed, gain, red, green and blue */)
{
	initDefaultBGR();
	setLensType(CAMERA_KBCRS02MU); // default camera type
	readCameraSettingFile();
	v4l2cap.open_device(cap_width, cap_height);
	IplImage *image = NULL;
	v4l2cap.read_frame(&image);
}

ImageGrabberV4l::~ImageGrabberV4l()
{
	v4l2cap.close_device();
}

void ImageGrabberV4l::initDefaultBGR(void)
{
	shutter = 98;
	gain = 60;
	wb_r = 70;
	wb_g = 50;
	wb_b = 100;
}


int ImageGrabberV4l::setLensType(int lens_type)
{
	switch(lens_type) {
	case CAMERA_KBCRM05VU:
		cam_para = new camera_parameter_KBCRM05VU;
		cap_width = 640;
		cap_height = 480;
		break;
	case CAMERA_KBCRS02MU:
		cam_para = new camera_parameter_KBCRS02MU;
		// FPS reference: https://www.shikino.co.jp/products/product-kbcr-s02mu.html
		// 7.5 FPS(SXGA: 1280x1024)
		//cap_width = 1280;
		//cap_height = 1024;
		// 30 FPS(VGA: 640x480)
		cap_width = 640;
		cap_height = 480;
		break;
	default:
		std::cerr << "Error: unexpected camera type [ " << lens_type << " ]" << std::endl;
		cap_width = 0;
		cap_height = 0;
		break;
	}
	return 0;
}


int ImageGrabberV4l::grabFrame(void)
{
	constexpr int failed_limit = 60;
	auto success = v4l2cap.grab_frame();
	if(success)
		capture_failed_count = 0;
	capture_failed_count++;
	if(capture_failed_count == failed_limit) {
		std::cerr << "set camera settings" << std::endl;
		writeCameraSettings();
	}
	return 0;
}


void ImageGrabberV4l::writeCameraSettings(void)
{
	int success_auto_mode = 0, success_write_parameters = 0;
	int fd = v4l2cap.getFd();
	constexpr int max_try_count = 10;

	for(int i = 0; i < max_try_count; i ++) {
		int failed = cam_para->setAutoMode(fd, iauto);
		if(failed) {
			std::cerr << "failed set camera auto mode" << std::endl;
		} else {
			success_auto_mode = 1;
			break;
		}
	}

	if(iauto == 0) {
		for(int i = 0; i < max_try_count; i ++) {
			int failed = 0;
			failed |= cam_para->setWhiteBalance(fd, wb_r, wb_g, wb_b);
			failed |= cam_para->setGain(fd, gain);
			failed |= cam_para->setShutterSpeed(fd, shutter);
			if(failed) {
				std::cerr << "failed set camera parameter" << std::endl;
			} else {
				success_write_parameters = 1;
				break;
			}
		}
	} else {
		success_write_parameters = 1;
	}

	if((!success_auto_mode) || (!success_write_parameters)) {
		int result = system("aplay sound/settingfailed.wav &");
	}
	return;
}


void ImageGrabberV4l::storeCameraSettings(bool bauto, int in_shutter, int in_gain, int whitebalance)
{
	iauto = bauto ? 1 : 0;
	wb_r = (whitebalance >> 16) & 0xff;
	wb_g = (whitebalance >>  8) & 0xff;
	wb_b = (whitebalance) & 0xff;
	shutter = in_shutter;
	gain = in_gain;
}

void ImageGrabberV4l::setCameraSettings(bool bauto, int in_shutter, int in_gain, int whitebalance)
{
	storeCameraSettings(bauto, in_shutter, in_gain, whitebalance);
	writeCameraSettings();
}


void ImageGrabberV4l::getCameraSettings(bool &bauto, int &out_shutter, int &out_gain, int &whitebalance)
{
	int fd = v4l2cap.getFd();
	bauto = false;
	constexpr int max_try_count = 10;
	for(int i = 0; i < max_try_count; i ++) {
		int r, g, b, gaintmp, shuttertmp;
		int failed = cam_para->getParameter(fd, &r, &g, &b, &gaintmp, &shuttertmp);
		if(failed) {
			std::cerr << "failed get camera parameter" << std::endl;
			out_shutter = 0;
			out_gain = 0;
			whitebalance = 0;
		} else {
			out_shutter = shuttertmp;
			out_gain = gaintmp;
			whitebalance = (r << 16) | (g << 8) | b;
			break;
		}
	}
}


void ImageGrabberV4l::writeCameraSettingFile(void)
{
	std::ofstream ofs(camera_bgr_filename);
	if(!ofs) {
		std::cerr << "Failed to save camera parameter to \"" << camera_bgr_filename << "\"." << std::endl;
		return;
	}
	ofs
		<< "shutter " << shutter << std::endl
		<< "gain " << gain << std::endl
		<< "whitebalance_r " << wb_r << std::endl
		<< "whitebalance_g " << wb_g << std::endl
		<< "whitebalance_b " << wb_b << std::endl;
}

void ImageGrabberV4l::readCameraSettingFile(void)
{
	std::vector<int> gains(bgr_param_num, 0);
	getCameraBGR(camera_bgr_filename, gains);
	shutter = gains[0];
	gain = gains[1];
	wb_r = gains[2];
	wb_g = gains[3];
	wb_b = gains[4];
}

void ImageGrabberV4l::getCameraBGR(std::string filename, std::vector<int> &param)
{
	std::vector<int> default_bgr(bgr_param_num, 0);
	getDefaultBGR(default_bgr);

	if(param.size() != bgr_param_num) {
		param.resize(bgr_param_num);
	}
	std::ifstream ifs(filename);
	if(!ifs) {
		param = default_bgr;
		return;
	}
	for(int i = 0; i < bgr_param_num; i++) {
		std::string param_line;
		std::getline(ifs, param_line);
		param_line.erase(0, param_line.find(' '));
		try {
			param[i] = std::stof(param_line);
		} catch(std::invalid_argument e) {
			param[i] = default_bgr[i];
		}
	}
}

void ImageGrabberV4l::getDefaultBGR(std::vector<int> &param)
{
	initDefaultBGR();
	std::vector<int> default_param = {
		shutter,
		gain,
		wb_r,
		wb_g,
		wb_b
	};
	param = default_param;
}

int ImageGrabberV4l::getImage(cv::Mat &img)
{
	IplImage *image = NULL;
	v4l2cap.retreve_frame(&image);
	if(image == NULL)
		return -1;
	cv::Mat tmp_img = cv::cvarrToMat(image);
	cv::resize(tmp_img, img, cv::Size(width, height));
	cvReleaseImage(&image);
	return 0;
}

#include <asm/types.h>
#include <linux/videodev2.h>

#include <libv4l1.h>
#include <libv4l2.h>

struct buffer
{
	void *  start;
	size_t  length;
};

typedef struct v4l2_ctrl_range {
	__u32 ctrl_id;
	__s32 initial_value;
	__s32 current_value;
	__s32 minimum;
	__s32 maximum;
	__s32 default_value;
} v4l2_ctrl_range;

#define MAX_V4L_BUFFERS 10

typedef struct CvCaptureCAM_V4L
{
	char* deviceName;
	int deviceHandle;
	int bufferIndex;
	int FirstCapture;

	int width; int height;

	struct video_capability capability;
	struct video_window     captureWindow;
	struct video_picture    imageProperties;
	struct video_mbuf       memoryBuffer;
	struct video_mmap       *mmaps;
	char *memoryMap;
	IplImage frame;

	/* V4L2 variables */
	buffer buffers[MAX_V4L_BUFFERS + 1];
	struct v4l2_capability cap;
	struct v4l2_input inp;
	struct v4l2_format form;
	struct v4l2_crop crop;
	struct v4l2_cropcap cropcap;
	struct v4l2_requestbuffers req;
	struct v4l2_jpegcompression compr;
	struct v4l2_control control;
	enum v4l2_buf_type type;
	struct v4l2_queryctrl queryctrl;
	struct v4l2_querymenu querymenu;

	/* V4L2 control variables */
	v4l2_ctrl_range** v4l2_ctrl_ranges;
	int v4l2_ctrl_count;

	int is_v4l2_device;
}
CvCaptureCAM_V4L;

class CvCaptureCAM_V4L_CPP
{
	public:
		CvCaptureCAM_V4L_CPP() { captureV4L = 0; }
		virtual ~CvCaptureCAM_V4L_CPP() { close(); }

		virtual bool open( int index );
		virtual void close();

		virtual double getProperty(int);
		virtual bool setProperty(int, double);
		virtual bool grabFrame();
		virtual IplImage* retrieveFrame(int);

		CvCaptureCAM_V4L* captureV4L;
};

int main(){
  IplImage *image = NULL;

  /*
  cv::CascadeClassifier cascade;
  cascade.load("xml/haarcascade_frontalface_alt.xml");
  std::vector<cv::Rect> faces;
  */

	v4l2capture v4l2cap;
	v4l2cap.open_device(V4L2_WIDTH, V4L2_HEIGHT);
	while(1){
		v4l2cap.read_frame(&image);
		cvShowImage("image",image);
    cv::Mat tmp_img = cv::cvarrToMat(image);
    cv::Mat bgr_img;
    cv::cvtColor(tmp_img, bgr_img, cv::COLOR_YCrCb2BGR);

    /*
    cascade.detectMultiScale(bgr_img, faces, 1.1, 3, 0, cv::Size(20,20));
    for(int i = 0;i<faces.size();i++){
      cv::rectangle(bgr_img, cv::Point(faces[i].x, faces[i].y), cv::Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), cv::Scalar(0, 0, 255), 3, CV_AA);
    }
    */
    cv::imshow("bgr_img", bgr_img);

		if (cv::waitKey(1) == 27) break;
	}
	v4l2cap.close_device();
	cvReleaseImage(&image);
  cv::destroyAllWindows();

	return 0;
}
