#include <chrono>
#include <mutex>

#include "image_grabber.h"

static std::mutex terminate_lock_obj;

ImageGrabber::ImageGrabber(int in_id, const int w, const int h) : terminated(false), bpp(3), id(in_id), width(w), height(h)
{
	img = cv::Mat(height, width, CV_8UC3);
}

ImageGrabber::~ImageGrabber()
{
}

int ImageGrabber::setLensType(int lensType)
{
	return 0;
}

int ImageGrabber::startCamera(void)
{
	terminated = false;
	capture_thread = std::thread(&ImageGrabber::threadRun, this);
	return 0;
}

int ImageGrabber::getBufferSize(void)
{
	return width * height * bpp;
}

void ImageGrabber::stopCamera(void)
{
	std::unique_lock<std::mutex> lock_terminate(terminate_lock_obj);
	terminated = true;
	lock_terminate.unlock();
	capture_thread.join();
}

void ImageGrabber::setCameraSettings(bool in_bauto, int in_shutter, int in_gain, int in_white_balance)
{
}

void ImageGrabber::getCameraSettings(bool &bauto, int &shutter, int &gain, int &white_balance)
{
	bauto = false;
	shutter = 0;
	gain = 0;
	white_balance = 0;
	return;
}

void ImageGrabber::threadRun(void)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	while(true) {
		grabFrame();
		std::unique_lock<std::mutex> lock_terminate(terminate_lock_obj);
		if(terminated)
			break;
		lock_terminate.unlock();
	}
}

void ImageGrabber::readCameraSettingFile(void)
{
}

void ImageGrabber::writeCameraSettingFile(void)
{
}

