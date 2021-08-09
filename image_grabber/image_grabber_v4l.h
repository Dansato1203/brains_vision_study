#ifndef IMAGE_GRABBER_V4L_H
#define IMAGE_GRABBER_V4L_H

#include <string>
#include <vector>
#include "image_grabber.h"

class ImageGrabberV4l : public ImageGrabber
{
public:
	ImageGrabberV4l(int id, const int w, const int h);
	~ImageGrabberV4l();
	int getImage(cv::Mat &) override;
	int grabFrame(void) override;
	int setLensType(int lensType) override;
	void readCameraSettingFile(void);
	void writeCameraSettingFile(void);
	void getCameraSettings(bool &bauto, int &shutter, int &gain, int &whitebalance) override;
	void setCameraSettings(bool bauto, int shutter, int gain, int whitebalance) override;

private:
	void getDefaultBGR(std::vector<int> &param);
	void getCameraBGR(std::string filename, std::vector<int> &param);
	void writeCameraSettings(void);
	void storeCameraSettings(bool bauto, int shutter, int gain, int whitebalance);
	void initDefaultBGR(void);
	int cap_width;
	int cap_height;
	int capture_failed_count;
	int iauto;
	int shutter;
	int gain;
	int wb_r;
	int wb_g;
	int wb_b;
	std::string camera_bgr_filename;
	const int bgr_param_num;
};

#endif // IMAGE_GRABBER_V4L_H

