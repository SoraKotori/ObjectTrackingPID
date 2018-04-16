#include "ObjectTrackingPID.h"
#include <stdlib.h>

int main(void)
{
	/*--- �϶��@�G�ŧi ---*/
	struct CAM_TARGET cam_target;							//�S�x�ȡG��J
	struct TARGET_CAM_PID target_cam_pid;					//����t�Ϊ��Ѽ�
	struct WHEEL wheel;										//��X

	/*--- �϶��G�G��l�� ---*/
	Load_TARGET_CAM_PID(&target_cam_pid);					//�q Init\target_webcam_pid.ini Ū��"�����k���Ѽ�"

	/*--- �϶��T�G�p�� ---*/
	Target_Cam_PID(&target_cam_pid, &cam_target, &wheel);	//�S�x��+�Ѽ� -> ��X

	return 0;
}