#ifndef IMAGE_GRABBER_H
#define IMAGE_GRABBER_H

#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageGrabber
{
public:
	ImageGrabber(int id, const int w, const int h);
	virtual ~ImageGrabber();
	virtual int setLensType(int lensType);
	int startCamera(void);
	void stopCamera(void);
	virtual int getImage(cv::Mat &) = 0;
	virtual int grabFrame(void) = 0;
	int getBufferSize(void);
	virtual void getCameraSettings(bool &bauto, int &shutter, int &gain, int &whitebalance);
	virtual void setCameraSettings(bool bauto, int shutter, int gain, int whitebalance);
	virtual void readCameraSettingFile(void);
	virtual void writeCameraSettingFile(void);

private:
	void threadRun(void);

protected:
	std::thread capture_thread;
	cv::Mat img;
	bool terminated;
	const int bpp;
	int id;
	int width;
	int height;
};

#endif // IMAGE_GRABBER_H

