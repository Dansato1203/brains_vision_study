/*
 * ロボカップ用ビデオ調整ツール
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>

#include "video_adjust_KBCRM05VU.h"

/* Shikino 2011.08.25 */
/*--------------------------------------------------------------------------*/
int m05vu_reg_write(int, unsigned char, unsigned char );
int m05vu_reg_read(int, unsigned char, unsigned char*);
/*--------------------------------------------------------------------------*/

#define CLEAR(x) memset (&(x), 0, sizeof (x))

/*
   typedef enum {
   IO_METHOD_READ,
   IO_METHOD_MMAP,
   IO_METHOD_USERPTR,
   } io_method;
   */

struct buffer {
	void *                  start;
	size_t                  length;
};

static char *           dev_name        = NULL;
static int              fd              = -1;
struct buffer *         buffers         = NULL;
static unsigned int     n_buffers       = 0;

	static void
errno_exit                      (const char *           s)
{
	fprintf (stderr, "%s error %d, %s\n",
			s, errno, strerror (errno));

	exit (EXIT_FAILURE);
}

	static int
xioctl                          (int                    fd,
		int                    request,
		void *                 arg)
{
	int r;

	do r = ioctl (fd, request, arg);
	while (-1 == r && EINTR == errno);

	return r;
}

	static void
uninit_device                   (void)
{
	free (buffers);
}

	static void
init_device                     (void)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;

	if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf (stderr, "%s is no V4L2 device\n",
					dev_name);
			exit (EXIT_FAILURE);
		} else {
			errno_exit ("VIDIOC_QUERYCAP");
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf (stderr, "%s is no video capture device\n",
				dev_name);
		exit (EXIT_FAILURE);
	}

	/* Select video input, video standard and tune here. */

	CLEAR (cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
				case EINVAL:
					/* Cropping not supported. */
					break;
				default:
					/* Errors ignored. */
					break;
			}
		}
	} else {        
		/* Errors ignored. */
	}


	CLEAR (fmt);

	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = 640; 
	fmt.fmt.pix.height      = 480;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

	if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
		errno_exit ("VIDIOC_S_FMT");

	/* Note VIDIOC_S_FMT may change width and height. */

	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;
}

	static void
close_device                    (void)
{
	if (-1 == close (fd))
		errno_exit ("close");

	fd = -1;
}

	static void
open_device                     (void)
{
	struct stat st; 

	if (-1 == stat (dev_name, &st)) {
		fprintf (stderr, "Cannot identify '%s': %d, %s\n",
				dev_name, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}

	if (!S_ISCHR (st.st_mode)) {
		fprintf (stderr, "%s is no device\n", dev_name);
		exit (EXIT_FAILURE);
	}

	fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

	if (-1 == fd) {
		fprintf (stderr, "Cannot open '%s': %d, %s\n",
				dev_name, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}
}

	static void
usage                           (FILE *                 fp,
		int                    argc,
		char **                argv)
{
	fprintf (fp,
			"Usage: %s [options]\n\n"
			"Options:\n"
			"-d | --device name   Video device name [/dev/video]\n"
			"-h | --help          Print this message\n"
			"",
			argv[0]);
}

static const char short_options [] = "d:h";

static const struct option
long_options [] = {
	{ "device",     required_argument,      NULL,           'd' },
	{ "help",       no_argument,            NULL,           'h' },
	{ 0, 0, 0, 0 }
};


/*
 * @brief ホワイトバランスなどのオートとマニュアルを切り替える
 * @param[in] hcam handle obtained from V4L2 open()
 * @param[in] is_auto オートモードのフラグ(1:auto, 0:manual)
 * @return 0:成功 -1:失敗
 */

int camera_parameter_KBCRM05VU::setAutoMode(int hcam, int is_auto)
{
	int ret = 0;
	if (is_auto){
		ret |= m05vu_reg_write(hcam, 0x13, 0xEF);		// ホワイトバランスなどをオートに設定
	} else {
		ret |= m05vu_reg_write(hcam, 0x13, 0x00);		// ホワイトバランスなどをマニュアルに設定
	}
	ret |= m05vu_reg_write(hcam, 0x3E, 0x02);			// Automatic Black Level Calibration
	ret |= m05vu_reg_write(hcam, 0x3E, 0x02);			// Automatic Black Level Calibration
	ret |= m05vu_reg_write(hcam, 0x64, 0x1F);			// UV Adjust Control
	ret |= m05vu_reg_write(hcam, 0x0E, 0x00);			// Auto Frame Rate Adjust In Low Light
	ret |= m05vu_reg_write(hcam, 0x2D, 0x00);			// 露光時間
	ret |= m05vu_reg_write(hcam, 0x2E, 0x00);			// 露光時間
	return ret;
}

/*
 * @brief ホワイトバランスの設定
 * @param[in] hcam handle obtained from V4L2 open()
 * @param[in] red   ホワイトバランスの赤の値(0-255)
 * @param[in] green ホワイトバランスの緑の値(0-255)
 * @param[in] blue  ホワイトバランスの青の値(0-255)
 * @return 0:成功 -1:失敗
 */
int camera_parameter_KBCRM05VU::setWhiteBalance(int hcam, int red, int green, int blue)
{
	int ret = 0;
	ret |= m05vu_reg_write(hcam, 0x1, blue );			// ホワイトバランスの青を設定する
	ret |= m05vu_reg_write(hcam, 0x2, red  );			// ホワイトバランスの赤を設定する
	ret |= m05vu_reg_write(hcam, 0x3, green);			// ホワイトバランスの緑を設定する
	return ret;
}

/*
 * @brief ゲインの設定
 * @param[in] hcam handle obtained from V4L2 open()
 * @param[in] gain ゲインの値(0-255:非線形)
 * @return 0:成功 -1:失敗
 */
int camera_parameter_KBCRM05VU::setGain(int hcam, int gain)
{
	return m05vu_reg_write(hcam, 0x0, gain);				// ゲインを設定する		
}

/*
 * @brief シャッタースピードの設定
 * @param[in] hcam handle obtained from V4L2 open()
 * @param[in] shutter_speed 16ビット
 * @return 0:成功 -1:失敗
 */
int camera_parameter_KBCRM05VU::setShutterSpeed(int hcam, int shutter_speed)
{
	int ret = 0;
	int high = shutter_speed / 0x100;
	int low  = shutter_speed & 0xff;
	ret |= m05vu_reg_write(hcam, 0x08, high);
	ret |= m05vu_reg_write(hcam, 0x10, low );
	return ret;
}

/*
 * @brief パラメータの取得
 * @param[in] hcam handle obtained from V4L2 open()
 * @param[out] red   ホワイトバランスの赤の値
 * @param[out] green ホワイトバランスの緑の値
 * @param[out] blue  ホワイトバランスの青の値
 * @param[out] gain  ゲインの値
 * @param[out] shutter_speed シャッタースピードの値
 * @return 0:成功 -1:失敗
 */
int camera_parameter_KBCRM05VU::getParameter(int hcam, int *red, int *green, int *blue, int *gain, int *shutter_speed)
{
	int ret = 0;
	unsigned char data;
	unsigned char data_high, data_low;

	ret |= m05vu_reg_read(hcam, 0x00, &data);
	*gain  = data;
	ret |= m05vu_reg_read(hcam, 0x01, &data);
	*blue  = data;
	ret |= m05vu_reg_read(hcam, 0x02, &data);
	*red   = data;
	ret |= m05vu_reg_read(hcam, 0x03, &data);
	*green = data;
	ret |= m05vu_reg_read(hcam, 0x08, &data_high);
	ret |= m05vu_reg_read(hcam, 0x10, &data_low );
	*shutter_speed = data_high * 0x0100 + data_low;

	return ret;
}

/* Shikino Add 2011.08.25 */

/*
 * Command ID
 */
#define COM_TYPE_COM_STT		(0xFF)           /* Command ID [START]      */
#define COM_TYPE_COM_END		(0xFE)           /* Command ID [END]        */
#define COM_TYPE_REG_WR		    (0xFA)           /* Command ID [REG WRITE]  */
#define COM_TYPE_REG_RD		    (0xF9)           /* Command ID [REG READ ]  */

/*
 * (Sub Routine) IRIS contorol
 */
int myIrisGetControl(int);                       /* IRIS property Get       */
int myIrisSetControl(int, unsigned char);        /* IRIS property Set       */


/* --------------------------------------------------------------------------
 *  Camera(KBCR-M05VU) register write 
 *
 * @param hcam		    handle obtained from V4L2 open()
 * @param addr 	        register address
 * @param data			register data
 *
 * @return
 * 		     0 (on success)
 * 		    -1 (if an error has occurred)
 ---------------------------------------------------------------------------- */

int m05vu_reg_write(int hcam, unsigned char addr, unsigned char data )
{
	int rtn;

	/* Command Start */
	if ((rtn = myIrisGetControl(hcam                     )) < 0) { goto error_end;}
	if ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_STT) )) < 0) { goto error_end;}

	/* Regiter Write */
	if ((rtn = myIrisGetControl(hcam                     )) < 0) { goto error_end;}
	if ((rtn = myIrisSetControl(hcam, (COM_TYPE_REG_WR)  )) < 0) { goto error_end;}
	if ((rtn = myIrisSetControl(hcam, (addr)             )) < 0) { goto error_end;}
	if ((rtn = myIrisSetControl(hcam, (data)             )) < 0) { goto error_end;}

	/* Command End */
	if ((rtn = myIrisGetControl(hcam                     )) < 0) { goto error_end;}
	if ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_END) )) < 0) { goto error_end;}

	return 0;   /* Success */

error_end:
	return rtn; /* Error */
}


/* --------------------------------------------------------------------------
 *  Camera(KBCR-M05VU) register read 
 *
 * @param hcam		    handle obtained from V4L2 open()
 * @param addr 	        register address
 * @param data			register data (*pointer)
 *
 * @return
 * 		     0 (on success)
 * 		    -1 (if an error has occurred)
 ---------------------------------------------------------------------------- */
int m05vu_reg_read(int hcam, unsigned char addr, unsigned char* data)
{

	int rtn;
	int reg_data;

	/* Command Start */
	if ((rtn = myIrisGetControl(hcam                     )) < 0){ goto error_end;}
	if ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_STT) )) < 0){ goto error_end;}

	/* Regiter Read */
	if ((rtn = myIrisGetControl(hcam                     )) < 0){ goto error_end;}
	if ((rtn = myIrisSetControl(hcam, (COM_TYPE_REG_RD)  )) < 0){ goto error_end;}
	if ((rtn = myIrisSetControl(hcam, (addr)             )) < 0){ goto error_end;}

	if ((reg_data = myIrisGetControl(hcam            )) < 0){ goto error_end;}
	else{
		*data = (unsigned char)reg_data; /* register value */
	}

	/* Command End */
	if ((rtn = myIrisGetControl(hcam                     )) < 0){ goto error_end;}
	if ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_END) )) < 0){ goto error_end;}

	return 0;   /* Success */

error_end:
	return rtn; /* Error */

}


/*
 * (Sub Routine) IRIS contorol
 */

int myIrisGetControl(int hcam )
{
	struct v4l2_control control_s;
	int err;

	control_s.id = V4L2_CID_IRIS_ABSOLUTE;

	if ((err = ioctl( hcam, VIDIOC_G_CTRL, &control_s)) < 0) {
		return -1;
	}

	return control_s.value;
}

int myIrisSetControl(int hcam, unsigned char value)
{
	struct v4l2_control control_s;
	int err;

	control_s.id = V4L2_CID_IRIS_ABSOLUTE;
	control_s.value = value;

	if ((err = ioctl( hcam, VIDIOC_S_CTRL, &control_s)) < 0) {
		return -1;
	}

	return 0;
}

