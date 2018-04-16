#include "ObjectTrackingPID.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define Stringizing(s) #s
#define path_txt "Init\\target_webcam_pid.ini"

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

#define SPEED_MIN 0
#define SPEED_MAX 510

#define Kp_fixed 1.500000
#define Ki_fixed 0.000000
#define Kd_fixed 0.190000

#define Straight_set_point_fixed 345.000000
#define Rotation_set_point_fixed 320.000000

bool PID_Init_fixed(struct TARGET_CAM_PID *const PID);
bool PID_Init_file_read(struct TARGET_CAM_PID *const PID);
bool PID_Init_file_read_One(FILE *file, char *PID_string, PID_Parameter *PID_parameter);
bool PID_Init_command(struct TARGET_CAM_PID *const PID);
bool PID_Init_command_One(char *PID_string, PID_Parameter *PID_parameter);
bool PID_Init_file_write(struct TARGET_CAM_PID *const PID);
bool PID_Init_file_write_One(FILE *file, char *PID_string, PID_Parameter *PID_parameter);

bool PID_execute_single(const double input, PID_Parameter *const parameter, double *output);
bool PID_execute_speed_check_One(int *const speed, const struct TARGET_CAM_PID *const PID);

bool Load_TARGET_CAM_PID(struct TARGET_CAM_PID *target_cam_pid)
{
	if (NULL == target_cam_pid)
	{
		assert(!(NULL == target_cam_pid));
		return false;
	}

	PID_Init_fixed(target_cam_pid);
	if (true == PID_Init_file_read(target_cam_pid))
	{
		return true;
	}
	PID_Init_command(target_cam_pid);
	if (true == PID_Init_file_write(target_cam_pid))
	{
		return true;
	}
	return false;
}

bool PID_Init_fixed(struct TARGET_CAM_PID *const PID)
{
	PID->frame_height = FRAME_HEIGHT;
	PID->frame_width = FRAME_WIDTH;
	PID->speed_max = SPEED_MAX;
	PID->speed_min = SPEED_MIN;
	PID->Straight.Kp = Kp_fixed;
	PID->Straight.Ki = Ki_fixed;
	PID->Straight.Kd = Kd_fixed;
	PID->Straight.set_point = Straight_set_point_fixed;
	PID->Rotation.Kp = Kp_fixed;
	PID->Rotation.Ki = Ki_fixed;
	PID->Rotation.Kd = Kd_fixed;
	PID->Rotation.set_point = Rotation_set_point_fixed;
	return true;
}

bool PID_Init_file_read(struct TARGET_CAM_PID *const PID)
{
	assert(NULL != PID);
	FILE *fptr = fopen(path_txt, "r");
	if (NULL == fptr)
	{
		assert(!(NULL == fptr));
		return false;
	}
	if (false == PID_Init_file_read_One(fptr, Stringizing(PID->Straight), &PID->Straight))
	{
		fclose(fptr);
		return false;
	}
	fscanf(fptr, "\n");
	if (false == PID_Init_file_read_One(fptr, Stringizing(PID->Rotation), &PID->Rotation))
	{
		fclose(fptr);
		return false;
	}
	fclose(fptr);
	return true;
}

bool PID_Init_file_read_One(FILE *file, char *PID_string, PID_Parameter *PID_parameter)
{
	assert(NULL != file);
	assert(NULL != PID_string);
	assert(NULL != PID_parameter);

	char *string = (char*)malloc(strlen(PID_string) + 1);
	if (NULL == string)
	{
		assert(NULL != string);
		return false;
	}

	int	items = fscanf(file, "%s\nKp:%lf\nKi:%lf\nKd:%lf\nset_point:%lf\n",
		string, &PID_parameter->Kp, &PID_parameter->Ki, &PID_parameter->Kd, &PID_parameter->set_point);
	bool result = 5 == items && 0 == strcmp(PID_string, string);
	free((void*)string);
	return result;
}

bool PID_Init_command(struct TARGET_CAM_PID *const PID)
{
	assert(NULL != PID);

	if (false == PID_Init_command_One(Stringizing(PID->Straight), &PID->Straight))
	{
		return false;
	}
	printf("\n");
	if (false == PID_Init_command_One(Stringizing(PID->Rotation), &PID->Rotation))
	{
		return false;
	}
	return true;
}

bool PID_Init_command_One(char *PID_string, PID_Parameter *PID_parameter)
{
	assert(NULL != PID_string);
	assert(NULL != PID_parameter);

	puts(PID_string);
	printf("Kp:");
	scanf("%lf", &PID_parameter->Kp);
	printf("Ki:");
	scanf("%lf", &PID_parameter->Ki);
	printf("Kd:");
	scanf("%lf", &PID_parameter->Kd);
	printf("set_point:");
	scanf("%lf", &PID_parameter->set_point);
	return true;
}

bool PID_Init_file_write(struct TARGET_CAM_PID *const PID)
{
	assert(NULL != PID);
	FILE *fptr = fopen(path_txt, "w");
	if (NULL == fptr)
	{
		assert(NULL != fptr);
		return false;
	}

	if (false == PID_Init_file_write_One(fptr, Stringizing(PID->Straight), &PID->Straight))
	{
		fclose(fptr);
		return false;
	}
	fprintf(fptr, "\n");
	if (false == PID_Init_file_write_One(fptr, Stringizing(PID->Rotation), &PID->Rotation))
	{
		fclose(fptr);
		return false;
	}
	fclose(fptr);
	return true;
}

bool PID_Init_file_write_One(FILE *file, char *PID_string, PID_Parameter *PID_parameter)
{
	assert(NULL != file);
	assert(NULL != PID_string);
	assert(NULL != PID_parameter);

	int characters = fprintf(file, "%s\nKp:%lf\nKi:%lf\nKd:%lf\nset_point:%lf\n",
		PID_string, PID_parameter->Kp, PID_parameter->Ki, PID_parameter->Kd, PID_parameter->set_point);
	assert(characters >= 0);
	return characters >= 0 ? true : false;
}

bool Target_Cam_PID(struct TARGET_CAM_PID *target_cam_pid, struct CAM_TARGET *cam_target, struct WHEEL *wheel)
{
	if (NULL == cam_target || NULL == target_cam_pid || NULL == wheel)
	{
		assert(!(NULL == cam_target));
		assert(!(NULL == target_cam_pid));
		assert(!(NULL == wheel));
		return false;
	}
	if (
		cam_target->x > target_cam_pid->frame_width ||
		cam_target->x < 0 ||
		cam_target->y > target_cam_pid->frame_height ||
		cam_target->y < 0
		)
	{
		assert(!(cam_target->x > target_cam_pid->frame_width));
		assert(!(cam_target->x < 0));
		assert(!(cam_target->y > target_cam_pid->frame_height));
		assert(!(cam_target->y < 0));
		return false;
	}
	PID_execute_single((double)cam_target->x, &target_cam_pid->Rotation, &target_cam_pid->difference);
	PID_execute_single((double)cam_target->y, &target_cam_pid->Straight, &target_cam_pid->speed);
	wheel->sal = (int)(target_cam_pid->speed - target_cam_pid->difference / 2);
	wheel->sar = (int)(target_cam_pid->speed + target_cam_pid->difference / 2);
	PID_execute_speed_check_One(&wheel->sal, target_cam_pid);
	PID_execute_speed_check_One(&wheel->sar, target_cam_pid);
	return true;
}

bool PID_execute_single(const double input, PID_Parameter *const parameter, double *output)
{
	parameter->previous_clock = parameter->now_clock;
	parameter->now_clock = clock();
	parameter->delta_time = parameter->now_clock == parameter->previous_clock ?
		0.001 : (double)(parameter->now_clock - parameter->previous_clock) / CLOCKS_PER_SEC;
	parameter->previous_error = parameter->now_error;
	parameter->now_error = parameter->set_point - input;
	parameter->integral += parameter->now_error * parameter->delta_time;
	*output =
		parameter->Kp * parameter->now_error +
		parameter->Ki * parameter->integral +
		parameter->Kd * (parameter->now_error - parameter->previous_error) / parameter->delta_time;
	return true;
}

bool PID_execute_speed_check_One(int *const speed, const struct TARGET_CAM_PID *const PID)
{
	if (*speed < PID->speed_min)
	{
		*speed = PID->speed_min;
		return true;
	}
	else if (*speed > PID->speed_max)
	{
		*speed = PID->speed_max;
		return true;
	}
	return true;
}
