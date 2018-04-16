#pragma once
#include <time.h>
#include <stdbool.h>

struct CAM_TARGET // 輸入特徵("與目的"及"感測器"相關)
{
	int x;
	int y;
};

typedef struct
{
	double Kp, Ki, Kd;
	double set_point;
	double delta_time;
	double integral;
	double now_error, previous_error;
	clock_t now_clock, previous_clock;
}PID_Parameter;

struct TARGET_CAM_PID // 主要跟"控制方法"的參數有關
{
	//xxx; //內容請視需求自行增減，主要與控制方法的參數有關
	int frame_width, frame_height;
	int speed_min, speed_max;
	double speed, difference;
	PID_Parameter Straight, Rotation;
};

struct WHEEL // 輸出
{
	int sal;
	int sar;
};

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

	bool Load_TARGET_CAM_PID(struct TARGET_CAM_PID *target_cam_pid);
	bool Target_Cam_PID(struct TARGET_CAM_PID *target_cam_pid, struct CAM_TARGET *cam_target, struct WHEEL *wheel);

#ifdef __cplusplus
}
#endif // __cplusplus
