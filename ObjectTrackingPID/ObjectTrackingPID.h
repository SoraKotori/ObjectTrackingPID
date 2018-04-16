#pragma once
#include <time.h>
#include <stdbool.h>

struct CAM_TARGET // ��J�S�x("�P�ت�"��"�P����"����)
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

struct TARGET_CAM_PID // �D�n��"�����k"���ѼƦ���
{
	//xxx; //���e�е��ݨD�ۦ�W��A�D�n�P�����k���ѼƦ���
	int frame_width, frame_height;
	int speed_min, speed_max;
	double speed, difference;
	PID_Parameter Straight, Rotation;
};

struct WHEEL // ��X
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
