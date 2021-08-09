#ifndef VIDEO_ADJUST_KBCRS02MU_H
#define VIDEO_ADJUST_KBCRS02MU_H

#include "video_adjust.h"

/*
 * ロボカップ用ビデオ調整ツール
 * KBCRS02MU用(1280x1024)
 */

class camera_parameter_KBCRS02MU : public camera_parameter {

public:
	/*
	 * @brief ホワイトバランスなどのオートとマニュアルを切り替える
	 * @param[in] hcam handle obtained from V4L2 open()
	 * @param[in] is_auto オートモードのフラグ(1:auto, 0:manual)
	 * @return 0:成功 -1:失敗
	 */
	int setAutoMode(int hcam, int is_auto);

	/*
	 * @brief ホワイトバランスの設定
	 * @param[in] hcam handle obtained from V4L2 open()
	 * @param[in] red   ホワイトバランスの赤の値(0-255)
	 * @param[in] green ホワイトバランスの緑の値(0-255)
	 * @param[in] blue  ホワイトバランスの青の値(0-255)
	 * @return 0:成功 -1:失敗
	 */
	int setWhiteBalance(int hcam, int red, int green, int blue);

	/*
	 * @brief ゲインの設定
	 * @param[in] hcam handle obtained from V4L2 open()
	 * @param[in] gain ゲインの値(0-255:非線形)
	 * @return 0:成功 -1:失敗
	 */
	int setGain(int hcam, int gain);

	/*
	 * @brief シャッタースピードの設定
	 * @param[in] hcam handle obtained from V4L2 open()
	 * @param[in] shutter_speed 16ビット
	 * @return 0:成功 -1:失敗
	 */
	int setShutterSpeed(int hcam, int shutter_speed);

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
	int getParameter(int hcam, int *red, int *green, int *blue, int *gain, int *shutter_speed);
};

#endif // VIDEO_ADJUST_KBCRS02MU_H

