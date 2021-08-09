#ifndef V4LCAPTURE_H
#define V4LCAPTURE_H

#include <opencv2/core/core.hpp>

constexpr int V4L2_N_IMAGES = 2;
constexpr int V4L2_WIDTH = 640;
constexpr int V4L2_HEIGHT = 480;

class v4l2capture
{
public:
	int read_frame(IplImage **image);
	int grab_frame(void);
	int retreve_frame(IplImage **image);
	void open_device(int width, int height);
	void close_device(void);
	int getFd(void)
	{
		return fd;
	}
private:
	const static char *dev_name0;					//! デバイス名0
	const static char *dev_name1;					//! デバイス名1
	static int fd;									//! ファイルディスクリプタ
	char *images[V4L2_N_IMAGES];					//! ビデオフレームメモリと共有するYUYV画像データ
	int image_size;								//! ビデオフレームメモリと共有するYUYV画像データのサイズ
	char *capture_image;							//! キャプチャしたYUYV画像データ
	int cap_width;									//! キャプチャの幅
	int cap_height;									//! キャプチャの高さ

	static void YUYV2YCrCb(char *yuyv_img, IplImage *img);
};

#endif // V4LCAPTURE_H

