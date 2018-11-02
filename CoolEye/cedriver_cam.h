#pragma once
#ifndef CEDRIVER_CAM_H
#define CEDRIVER_CAM_H

#include "cedriver_imu.h"
#include "cedriver_thread.h"
#include "cedriver_global_config.h"

struct img_pkg
{
	unsigned char data[IMG_BUF_SIZE_WVGA];
	double timestamp;
};

struct d1_img_output_pkg
{
	img_pkg *left_img;
	img_pkg *right_img;
};

int gettimeofday(struct timeval *tp, void *tzp);



int ce_cam_capture_init();



void ce_cam_capture(CEThread *pThread);

void ce_cam_preprocess(CEThread *pThread);

void ce_cam_showimg(CECamShowing *pThread);

#endif

