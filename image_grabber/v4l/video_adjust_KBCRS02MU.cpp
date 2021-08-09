/*
 * ロボカップ用ビデオ調整ツール
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>

#include "video_adjust_KBCRS02MU.h"

//--- Define White Blance Register Address ---
#define R_DATA_REG_H  (0x3D0)    //R Hi(2bit)
#define R_DATA_REG_L  (0x3D1)    //R Lo(8bit)

#define G_DATA_REG_H  (0x3D2)    //G Hi(2bit)
#define G_DATA_REG_L  (0x3D3)    //G Lo(8bit)

#define B_DATA_REG_H  (0x3D4)    //B Hi(2bit)
#define B_DATA_REG_L  (0x3D5)    //B Lo(8bit)

/*------------------------------------------------------------------------*/
/* Read Register                                                          */
/*------------------------------------------------------------------------*/
int get_dsp_reg(int cam, unsigned short addr, unsigned char* data)
{
	int	rtn_cd;
	int reg_addr;

	unsigned char	mData[11];
	struct uvc_xu_control_query	sCQ;

	mData[0] = 0x01;
	mData[1] = 0x02;
	mData[2] = 0x00;
	mData[3] = 0x00;
	mData[4] = 0x00;
	mData[5] = 0x00;
	mData[6] = 0x00;
	mData[7] = 0x00;
	mData[8] = 0x00;
	mData[9] = 0x00;
	mData[10] = 0x00;

	sCQ.unit		= 0x03;
	sCQ.selector	= 0x02;
	sCQ.query		= UVC_SET_CUR;
	sCQ.size		= 11;
	sCQ.data		= mData;

	rtn_cd	= ioctl(cam, UVCIOC_CTRL_QUERY, &sCQ);
	if( rtn_cd ) { return (rtn_cd); }

	reg_addr = addr;

	mData[0] = 0x03;
	mData[1] = (unsigned char)(reg_addr&0xff);
	mData[2] = (unsigned char)((reg_addr>>8)&0xff);
	mData[3] = 0x01;
	mData[4] = 0x00;
	mData[5] = 0x00;
	mData[6] = 0x00;
	mData[7] = 0x00;
	mData[8] = 0x00;
	mData[9] = 0x00;
	mData[10] = 0x00;

	sCQ.unit		= 0x03;
	sCQ.selector	= 0x02;
	sCQ.query		= UVC_SET_CUR;
	sCQ.size		= 11;
	sCQ.data		= mData;

	rtn_cd	= ioctl(cam, UVCIOC_CTRL_QUERY, &sCQ);
	if( rtn_cd ) { return (rtn_cd); }

	sCQ.unit		= 0x03;
	sCQ.selector	= 0x02;
	sCQ.query		= UVC_GET_CUR;
	sCQ.size		= 11;
	sCQ.data		= mData;

	rtn_cd	= ioctl(cam, UVCIOC_CTRL_QUERY, &sCQ);
	if( rtn_cd ) { return (rtn_cd); }

	*data = mData[4];

	mData[0] = 0x01;
	mData[1] = 0x00;
	mData[2] = 0x00;
	mData[3] = 0x00;
	mData[4] = 0x00;
	mData[5] = 0x00;
	mData[6] = 0x00;
	mData[7] = 0x00;
	mData[8] = 0x00;
	mData[9] = 0x00;
	mData[10] = 0x00;

	sCQ.unit		= 0x03;
	sCQ.selector	= 0x02;
	sCQ.query		= UVC_SET_CUR;
	sCQ.size		= 11;
	sCQ.data		= mData;

	rtn_cd	= ioctl(cam, UVCIOC_CTRL_QUERY, &sCQ);

	return (rtn_cd);
}


/*------------------------------------------------------------------------*/
/* Write Register                                                         */
/*------------------------------------------------------------------------*/
int set_dsp_reg(int cam, unsigned short addr, unsigned char data)
{
	int	rtn_cd;

	unsigned char	mData[11];
	struct uvc_xu_control_query	sCQ;

	int reg_addr;

	mData[0] = 0x01;
	mData[1] = 0x02;
	mData[2] = 0x00;
	mData[3] = 0x00;
	mData[4] = 0x00;
	mData[5] = 0x00;
	mData[6] = 0x00;
	mData[7] = 0x00;
	mData[8] = 0x00;
	mData[9] = 0x00;
	mData[10] = 0x00;

	sCQ.unit		= 0x03;
	sCQ.selector	= 0x02;
	sCQ.query		= UVC_SET_CUR;
	sCQ.size		= 11;
	sCQ.data		= mData;

	rtn_cd	= ioctl(cam, UVCIOC_CTRL_QUERY, &sCQ);
	if( rtn_cd ) { return (rtn_cd); }

	reg_addr = addr;

	mData[0] = 0x02;
	mData[1] = (unsigned char)(reg_addr&0xff);
	mData[2] = (unsigned char)((reg_addr>>8)&0xff);
	mData[3] = 0x01;
	mData[4] = data;
	mData[5] = 0x00;
	mData[6] = 0x00;
	mData[7] = 0x00;
	mData[8] = 0x00;
	mData[9] = 0x00;
	mData[10] = 0x00;

	sCQ.unit		= 0x03;
	sCQ.selector	= 0x02;
	sCQ.query		= UVC_SET_CUR;
	sCQ.size		= 11;
	sCQ.data		= mData;

	rtn_cd	= ioctl(cam, UVCIOC_CTRL_QUERY, &sCQ);
	if( rtn_cd ) { return (rtn_cd); }

	mData[0] = 0x01;
	mData[1] = 0x00;
	mData[2] = 0x00;
	mData[3] = 0x00;
	mData[4] = 0x00;
	mData[5] = 0x00;
	mData[6] = 0x00;
	mData[7] = 0x00;
	mData[8] = 0x00;
	mData[9] = 0x00;
	mData[10] = 0x00;

	sCQ.unit		= 0x03;
	sCQ.selector	= 0x02;
	sCQ.query		= UVC_SET_CUR;
	sCQ.size		= 11;
	sCQ.data		= mData;

	rtn_cd	= ioctl(cam, UVCIOC_CTRL_QUERY, &sCQ);

	return (rtn_cd);
}

/*
 * @brief ホワイトバランスなどのオートとマニュアルを切り替える
 * @param[in] hcam handle obtained from V4L2 open()
 * @param[in] is_auto オートモードのフラグ(1:auto, 0:manual)
 * @return 0:成功 -1:失敗
 */

int camera_parameter_KBCRS02MU::setAutoMode(int hcam, int is_auto)
{
	int ret = 0;
	struct v4l2_control control_s;

	control_s.id    = V4L2_CID_AUTO_WHITE_BALANCE;
	control_s.value = is_auto ? 1 : 0;
	ret |= (ioctl(hcam, VIDIOC_S_CTRL, &control_s) < 0);

	control_s.id    = V4L2_CID_EXPOSURE_AUTO;
	control_s.value = is_auto ? 0 : 1;
	ret |= (ioctl(hcam, VIDIOC_S_CTRL, &control_s) < 0);

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
int camera_parameter_KBCRS02MU::setWhiteBalance(int hcam, int red, int green, int blue)
{
	int ret = 0;

	ret |= set_dsp_reg(hcam, R_DATA_REG_H,  red   >> 6        ); //R Hi(2bit)
	ret |= set_dsp_reg(hcam, R_DATA_REG_L, (red   << 2) & 0xff); //R Lo(8bit)

	ret |= set_dsp_reg(hcam, G_DATA_REG_H,  green >> 6        ); //G Hi(2bit)
	ret |= set_dsp_reg(hcam, G_DATA_REG_L, (green << 2) & 0xff); //G Lo(8bit)

	ret |= set_dsp_reg(hcam, B_DATA_REG_H,  blue  >> 6        ); //B Hi(2bit)
	ret |= set_dsp_reg(hcam, B_DATA_REG_L, (blue  << 2) & 0xff); //B Lo(8bit)

	if (ret) printf("error : white balance\r\n");

	return ret;
}

/*
 * @brief ゲインの設定
 * @param[in] hcam handle obtained from V4L2 open()
 * @param[in] gain ゲインの値(0-255:非線形)
 * @return 0:成功 -1:失敗
 */
int camera_parameter_KBCRS02MU::setGain(int hcam, int gain)
{
	int ret = 0;
	struct v4l2_control control_s;

	control_s.id    = V4L2_CID_GAIN;
	control_s.value = gain;
	ret |= (ioctl(hcam, VIDIOC_S_CTRL, &control_s) < 0);

	if (ret) printf("error : gain\r\n");

	return ret;
}

/*
 * @brief シャッタースピードの設定
 * @param[in] hcam handle obtained from V4L2 open()
 * @param[in] shutter_speed 16ビット
 * @return 0:成功 -1:失敗
 */
int camera_parameter_KBCRS02MU::setShutterSpeed(int hcam, int shutter_speed)
{
	int ret = 0;
	struct v4l2_control control_s;

	control_s.id    = V4L2_CID_EXPOSURE_ABSOLUTE;
	control_s.value = shutter_speed;
	ret |= (ioctl(hcam, VIDIOC_S_CTRL, &control_s) < 0);

	if (ret) printf("error : shutter speed\r\n");

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
int camera_parameter_KBCRS02MU::getParameter(int hcam, int *red, int *green, int *blue, int *gain, int *shutter_speed)
{
	int ret = 0;
	unsigned char data;
	unsigned char data_high, data_low;
	struct v4l2_control control_s;

	ret |= (get_dsp_reg(hcam, R_DATA_REG_H, &data_high) < 0);
	ret |= (get_dsp_reg(hcam, R_DATA_REG_L, &data_low ) < 0);
	*red   = (data_high << 6) + (data_low >> 2);

	ret |= (get_dsp_reg(hcam, G_DATA_REG_H, &data_high) < 0);
	ret |= (get_dsp_reg(hcam, G_DATA_REG_L, &data_low ) < 0);
	*green = (data_high << 6) + (data_low >> 2);

	ret |= (get_dsp_reg(hcam, B_DATA_REG_H, &data_high) < 0);
	ret |= (get_dsp_reg(hcam, B_DATA_REG_L, &data_low ) < 0);
	*blue  = (data_high << 6) + (data_low >> 2);

	control_s.id = V4L2_CID_GAIN;
	ret |= (ioctl( hcam, VIDIOC_G_CTRL, &control_s) < 0);
	*gain = control_s.value;

	control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
	ret |= (ioctl( hcam, VIDIOC_G_CTRL, &control_s) < 0);
	*shutter_speed = control_s.value;

	return ret;
}

