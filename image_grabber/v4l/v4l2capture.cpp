#include <iostream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <linux/videodev2.h>

#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <vector>

#include "v4l2capture.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))				// 変数のクリアのマクロ

int v4l2capture::fd = -1;								// ファイルディスクリプタ 
const char *v4l2capture::dev_name0 = "/dev/video0";	// デバイスのアドレス
const char *v4l2capture::dev_name1 = "/dev/video1";	// デバイスのアドレス

static std::mutex lock_obj;

/*!
 * @brief エラーを表示して終了する．
 */
static void errno_exit(const char *s){
	std::cerr << s << " error " << errno << ", " << strerror(errno) << std::endl;
	exit (EXIT_FAILURE);
}

/*!
 * @brief デバイスの制御を行う関数
 *
 * @param[in] fd ファイルディスクリプタ
 * @param[in] request デバイス依存のリクエストコード
 * @param[in] arg パラメータ（型を指定しないポインタ）
 *
 * @return ioctlの戻り値
 */
static int xioctl(int fd, int request, void *arg)
{
	int r;
	do {
		r = ioctl(fd, request, arg);
	} while(r == -1 && errno == EINTR);
	return r;
}

/*!
 * @brief YUYVからIplImage形式に変換
 * @param[in] yuyv_img ソースのYUYV画像
 * @param[out] img ソースのBGR画像
 */
void v4l2capture::YUYV2YCrCb(char *yuyv_img, IplImage *img)
{
	unsigned char *yuyv = (unsigned char *)yuyv_img;
	unsigned char *ycrcb = (unsigned char *)img->imageData;
	int num = img->width * img->height / 2;
	int y0, y1, u, v;

	for (int n=0; n<num ; n++){
		y0 = (*yuyv++);
		u  = (*yuyv++);
		y1 = (*yuyv++);
		v  = (*yuyv++);

		*(ycrcb++) = y0;
		*(ycrcb++) = v;
		*(ycrcb++) = u;
		*(ycrcb++) = y1;
		*(ycrcb++) = v;
		*(ycrcb++) = u;
	}
}

/*!
 * @brief カメラ画像をキャプチャする。
 * キャプチャのみ行う。読み出しはretreve_frameで行う。
 *
 * @return 0:正常にキャプチャ、-1:エラー
 */
int v4l2capture::grab_frame()
{
	struct v4l2_buffer buf;
	fd_set fds;
	struct timeval tv = {2, 0};							// タイムアウトを2秒間とする

	while(true){
		FD_ZERO (&fds);									// ファイルディスクリプタの登録
		FD_SET (fd, &fds);
		int r = select(fd + 1, &fds, NULL, NULL, &tv);	// キャプチャが完了するまで待つ
		if (r == -1) {									// エラー
			std::cerr << "select" << std::endl;
			break;										// 再接続を試みる
		}
		if (!r){										// タイムアウトの場合
			std::cerr << "time out" << std::endl;
			break;										// 再接続を試みる
		}
		CLEAR (buf);									// バッファをクリア
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;			// ビデオキャプチャのバッファ
		buf.memory = V4L2_MEMORY_MMAP;					// メモリーマップI/Oのバッファ
		std::unique_lock<std::mutex> lock_camera(lock_obj);
		if (xioctl (fd, VIDIOC_DQBUF, &buf) == -1){		// キューから1フレーム分のバッファを取り出す
			std::cerr << "VIDIOC_DQBUF" << std::endl;
			break;										// 再接続を試みる
		}
		capture_image = images[buf.index];				// 引数のポインタの画像データにコピーする
		if (xioctl (fd, VIDIOC_QBUF, &buf) == -1){		// バッファをエンキューする
			std::cerr << "VIDIOC_QBUF" << std::endl;
			break;										// 再接続を試みる
		}
		return 0;
	}
	capture_image = NULL;
	close_device();
	open_device(cap_width, cap_height);
	std::cerr << "reboot capture device" << std::endl;

	return -1;
}

/*!
 * @brief キャプチャした画像を読み出す
 * 必ずgrab_frameを行ってから実行する
 *
 * @param[out] image 読み出した画像
 *
 * @return 0:正常にキャプチャ、-1:エラー
 */
int v4l2capture::retreve_frame(IplImage **image)
{
	if (capture_image == NULL) return -1;
	if (*image == NULL){
		*image = cvCreateImage(cvSize(cap_width, cap_height), IPL_DEPTH_8U, 3);
	}
	if (((*image)->width != cap_width)||((*image)->height != cap_height)){
		cvReleaseImage(image);
		*image = cvCreateImage(cvSize(cap_width, cap_height), IPL_DEPTH_8U, 3);		
	}
	std::lock_guard<std::mutex> lock_camera(lock_obj);
	YUYV2YCrCb(capture_image, *image);				//YUYVからIplImageへ変換
	return 0;
}

/*!
 * @brief キャプチャした画像を読み出す
 * キャプチャは30fpsで行うため、そのキャプチャしたタイミングを待って読み出す。
 *
 * @param[out] image 読み出した画像
 *
 * @return 0:正常にキャプチャ、-1:エラー
 */
int v4l2capture::read_frame(IplImage **image)
{
	if (grab_frame()) return -1;
	retreve_frame(image);
	return 0;
}


/*!
 * @brief デバイスをオープンする
 *
 * @param[in] width 画像の幅
 * @param[in] height 画像の高さ
 *
 * @return 0:正常, -1:異常終了
 */
void v4l2capture::open_device(int width, int height){
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	unsigned int i;
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cap_width = width, cap_height = height;

	while(1){
		if (-1 != (fd = open (dev_name0, O_RDWR | O_NONBLOCK, 0))) break;
		if (-1 != (fd = open (dev_name1, O_RDWR | O_NONBLOCK, 0))) break;
		std::cerr << "Cannot open camera device" << std::endl;;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) errno_exit("no V4L2 device");
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) errno_exit("can't dev");
	if (!(cap.capabilities & V4L2_CAP_STREAMING)) errno_exit("err stream IO");

	CLEAR (fmt);
	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = width;
	fmt.fmt.pix.height      = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field       = V4L2_FIELD_ANY;
	if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt)) errno_exit("VIDIOC_S_FMT");
	if ((fmt.fmt.pix.width != width) || (fmt.fmt.pix.height != height))
		errno_exit("unavailable size");

	//init_mmap
	struct v4l2_requestbuffers req;
	CLEAR (req);
	req.count  = 2;
	req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) errno_exit("err mem mapping");
	if (2 > req.count) errno_exit("Insufficient buffer memory on %s\n");

	for (i = 0; i < V4L2_N_IMAGES; i++) {
		struct v4l2_buffer buf;
		CLEAR (buf);
		buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index  = i;
		if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)) errno_exit("QUERYBUF");
		image_size = buf.length;
		images[i] = (char *)mmap (
			NULL /* start anywhere */,
			buf.length,
			PROT_READ | PROT_WRITE /* required */,
			MAP_SHARED /* recommended */,
			fd, buf.m.offset);

		if (MAP_FAILED == images[i]) errno_exit ("mmap");
		if (-1 == xioctl (fd, VIDIOC_QBUF, &buf)) errno_exit ("VIDIOC_QBUF");
	}
	capture_image = NULL;

	if (-1 == xioctl (fd, VIDIOC_STREAMON, &type)) errno_exit ("STREAMON");
}

/*!
 * @brief カメラデバイスを閉じる
 *
 */
void v4l2capture::close_device(void){
	unsigned int i;
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
		std::cerr << "STREAMOFF error" << std::endl;

	for (i = 0; i < V4L2_N_IMAGES; i++) {
		munmap (images[i], image_size);
	}
	if (-1 == close(fd))
		std::cerr << "camera close error" << std::endl;
	fd = -1;
}


#if 0
// 動作チェックのための関数
// コンパイル方法
// gcc -o v4l2capture v4l2capture.cpp `pkg-config --libs opencv` -lboost_thread
// gcc -o v4l2capture v4l2capture.cpp `pkg-config --libs opencv` -lpthread
//
int main(){
	IplImage *image = NULL;

  cv::CascadeClassifier cascade;
  cascade.load("xml/haarcascade_frontalface_alt.xml");
  std::vector<cv::Rect> faces;

	v4l2capture v4l2cap;
	v4l2cap.open_device(V4L2_WIDTH, V4L2_HEIGHT);
	while(1){
		v4l2cap.read_frame(&image);
		cvShowImage("image",image);
    cv::Mat tmp_img = cv::cvarrToMat(image);
    cv::Mat bgr_img;
    cv::cvtColor(tmp_img, bgr_img, cv::COLOR_YCrCb2BGR);

    cascade.detectMultiScale(bgr_img, faces, 1.1, 3, 0, cv::Size(20,20));
    for(int i = 0;i<faces.size();i++){
      cv::rectangle(bgr_img, cv::Point(faces[i].x, faces[i].y), cv::Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), cv::Scalar(0, 0, 255), 3, CV_AA);
    }
    cv::imshow("bgr_img", bgr_img);

		if (cv::waitKey(1) == 27) break;
	}
	v4l2cap.close_device();
	cvReleaseImage(&image);
  cv::destroyAllWindows();

	return 0;
}
#endif

