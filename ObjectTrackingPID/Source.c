#include "ObjectTrackingPID.h"
#include <stdlib.h>

int main(void)
{
	/*--- 區塊一：宣告 ---*/
	struct CAM_TARGET cam_target;							//特徵值：輸入
	struct TARGET_CAM_PID target_cam_pid;					//控制系統的參數
	struct WHEEL wheel;										//輸出

	/*--- 區塊二：初始化 ---*/
	Load_TARGET_CAM_PID(&target_cam_pid);					//從 Init\target_webcam_pid.ini 讀取"控制方法的參數"

	/*--- 區塊三：計算 ---*/
	Target_Cam_PID(&target_cam_pid, &cam_target, &wheel);	//特徵值+參數 -> 輸出

	return 0;
}