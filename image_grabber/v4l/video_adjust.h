#ifndef VIDEO_ADJUST_H
#define VIDEO_ADJUST_H

/*
 * ロボカップ用ビデオ調整ツールの雛形
 */

class camera_parameter {

public:
	/*
	 * @brief ホワイトバランスなどのオートとマニュアルを切り替える
	 * @param[in] hcam handle obtained from V4L2 open()
	 * @param[in] is_auto オートモードのフラグ(1:auto, 0:manual)
	 * @return 0:成功 -1:失敗
	 */
	virtual int setAutoMode(int hcam, int is_auto) = 0;

	/*
	 * @brief ホワイトバランスの設定
	 * @param[in] hcam handle obtained from V4L2 open()
	 * @param[in] red   ホワイトバランスの赤の値(0-255)
	 * @param[in] green ホワイトバランスの緑の値(0-255)
	 * @param[in] blue  ホワイトバランスの青の値(0-255)
	 * @return 0:成功 -1:失敗
	 */
	virtual int setWhiteBalance(int hcam, int red, int green, int blue) = 0;

	/*
	 * @brief ゲインの設定
	 * @param[in] hcam handle obtained from V4L2 open()
	 * @param[in] gain ゲインの値(0-255:非線形)
	 * @return 0:成功 -1:失敗
	 */
	virtual int setGain(int hcam, int gain) = 0;

	/*
	 * @brief シャッタースピードの設定
	 * @param[in] hcam handle obtained from V4L2 open()
	 * @param[in] shutter_speed 16ビット
	 * @return 0:成功 -1:失敗
	 */
	virtual int setShutterSpeed(int hcam, int shutter_speed) = 0;

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
	virtual int getParameter(int hcam, int *red, int *green, int *blue, int *gain, int *shutter_speed) = 0;
};

#endif // VIDEO_ADJUST_H

