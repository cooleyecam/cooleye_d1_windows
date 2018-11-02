
#include <QtWidgets/QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QDataStream>
#include <QThread>
#include <queue>
#include <time.h>
//#include <pthread.h>

#include "libusb.h"
#include "cedriver_usb.h"
#include "cedriver_cam.h"
#include "cedriver_config.h"
#include "cedriver_global_config.h"
#include "cedriver_threadsafe_queue.h"

using namespace std;

bool ce_cam_rst_flag_left = false;
bool ce_cam_rst_flag_right = false;

libusb_device_handle *pcaml_handle;
libusb_device_handle *pcamr_handle;

threadsafe_queue<img_pkg *> img_pkg_left_list;
threadsafe_queue<img_pkg *> img_pkg_right_list;
threadsafe_queue<d1_img_output_pkg *> img_pkg_list_d1;

int gettimeofday(struct timeval *tp, void *tzp)
{
#if 0
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;

	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
#else
	static const DWORDLONG FILETIME_to_timval_skew = 116444736000000000;
	FILETIME   tfile;
	::GetSystemTimeAsFileTime(&tfile);

	ULARGE_INTEGER _100ns;
	_100ns.LowPart = tfile.dwLowDateTime;
	_100ns.HighPart = tfile.dwHighDateTime;

	_100ns.QuadPart -= FILETIME_to_timval_skew;

	// Convert 100ns units to seconds;
	ULARGE_INTEGER largeint;
	largeint.QuadPart = _100ns.QuadPart / (10000 * 1000);

	// Convert 100ns units to seconds;
	tp->tv_sec = (long)(_100ns.QuadPart / (10000 * 1000));
	// Convert remainder to microseconds;
	tp->tv_usec = (long)((_100ns.QuadPart % (10000 * 1000)) / 10);
#endif

	return (0);
}

static libusb_device_handle* ce_cam_get_cam_handle(int cam_num)
{
	if (CAMD1_LEFT == cam_num)
		return pcaml_handle;
	else if (CAMD1_RIGHT == cam_num)
		return pcamr_handle;
	else
	{
		LOG("celog: Wrong cam number!\r\n");
		return NULL;
	}
}

static int ce_cam_ctrl_camera(int cam_num, unsigned char instruction)
{
	unsigned char buf[1];
	int r = libusb_control_transfer(ce_cam_get_cam_handle(cam_num), RT_H2D, instruction, 0, 0, buf, 0, 1000);
	if (r != 0)
	{
		LOG("celog: cam%d,i:0x%02X,r:%d\r\n", cam_num, instruction, r);
	}

	return r;
}

static int ce_cam_i2c_write(int cam_num, unsigned char reg, int value)
{
	unsigned char buf[1];
	int r = libusb_control_transfer(ce_cam_get_cam_handle(cam_num), RT_D2H, CAM_I2C_W, (CAM_I2C_ADDR << 8) + reg, value, buf, 1, 1000);
	if (r != 1)
	{
		LOG("celog: cam%d,i:0x%02X,r:%d\r\n", cam_num, CAM_I2C_W, r);
		return r;
	}
	if (buf[0] != SUCCESS)
	{
		LOG("celog: I2C Write failed.  Cam:%d, Addr:0x%02X, Reg:0x%02X, Write:0x%04X, I2C_return:%d\r\n", cam_num, CAM_I2C_ADDR, reg, value, buf[0]);
		return r;
	}
	return r;
}

static int ce_cam_sync_rst(int camlr)
{
	if (CAMD1_LEFT == camlr)
		ce_cam_rst_flag_left = true;
	else if (CAMD1_RIGHT == camlr)
		ce_cam_rst_flag_right = true;
	else
	{
		LOG("celog: Wrong cam number!\r\n");
		return ERROR;
	}

	while ((!ce_cam_rst_flag_left) || (!ce_cam_rst_flag_right));

	//usleep(100000);
	Sleep(1);

	ce_cam_i2c_write(camlr, 0x0C, 0x0001);
	ce_cam_ctrl_camera(camlr, STANDBY_SHORT);

	if (CAMD1_LEFT == camlr)
		ce_cam_rst_flag_left = false;
	else if (CAMD1_RIGHT == camlr)
		ce_cam_rst_flag_right = false;

	return SUCCESS;
}

static int ce_cam_set_af_mode(int camlr)
{
	if (CAMD1_LEFT == camlr)
		ce_cam_i2c_write(camlr, 0x70, 0x0303);
	else if (CAMD1_RIGHT == camlr)
		ce_cam_i2c_write(camlr, 0x70, 0x0000);
	//  ce_cam_i2c_write(camlr,0x70,0x0303);
	else
	{
		LOG("celog: Wrong cam number!\r\n");
		return ERROR;
	}

	return SUCCESS;
}

static void ce_cam_set_mt9v034_config_default(int camlr)
{
	ce_cam_i2c_write(camlr, 0x07, 0x0388);

	// CONTEXT A
	ce_cam_i2c_write(camlr, 0x04, 0x02F0);    // 720X480
	ce_cam_i2c_write(camlr, 0x03, 0x01E0);
	ce_cam_i2c_write(camlr, 0x05, 0x0106);    // HB
	ce_cam_i2c_write(camlr, 0x06, 0x024B);    // VB

	ce_cam_i2c_write(camlr, 0x0D, 0x0300);

	ce_cam_i2c_write(camlr, 0x01, 0x0001);
	ce_cam_i2c_write(camlr, 0x02, 0x0004);
	ce_cam_i2c_write(camlr, 0x08, 0x01BB);
	ce_cam_i2c_write(camlr, 0x09, 0x01D9);
	ce_cam_i2c_write(camlr, 0x0A, 0x0164);

	// CONTEXT B
	ce_cam_i2c_write(camlr, 0xCC, 0x02F0);
	ce_cam_i2c_write(camlr, 0xCB, 0x01E0);
	ce_cam_i2c_write(camlr, 0xCD, 0x0106);
	ce_cam_i2c_write(camlr, 0xCE, 0x024B);

	ce_cam_i2c_write(camlr, 0x0E, 0x0300);

	ce_cam_i2c_write(camlr, 0xC9, 0x0001);
	ce_cam_i2c_write(camlr, 0xCA, 0x0004);
	ce_cam_i2c_write(camlr, 0xCF, 0x01BB);
	ce_cam_i2c_write(camlr, 0xD0, 0x01D9);
	ce_cam_i2c_write(camlr, 0xD1, 0x0164);



	ce_cam_i2c_write(camlr, 0x70, 0x0000);

	ce_cam_set_af_mode(camlr);

	ce_cam_i2c_write(camlr, 0x0F, 0x0000);
	ce_cam_i2c_write(camlr, 0xAC, 0x0001);
	ce_cam_i2c_write(camlr, 0xAD, 0x01E0);
	ce_cam_i2c_write(camlr, 0xAB, 0x0040);
	ce_cam_i2c_write(camlr, 0xB0, 0xFFFF);
	ce_cam_i2c_write(camlr, 0xA5, 0x0037);
	ce_cam_i2c_write(camlr, 0x7F, 0x0000);
	ce_cam_i2c_write(camlr, 0xA6, 0x0001);
	ce_cam_i2c_write(camlr, 0xA8, 0x0000);
	ce_cam_i2c_write(camlr, 0xA9, 0x0000);
	ce_cam_i2c_write(camlr, 0xAA, 0x0000);


	ce_cam_i2c_write(camlr, 0x34, 0x0003);
	ce_cam_i2c_write(camlr, 0x3c, 0x0003);
	ce_cam_i2c_write(camlr, 0x2c, 0x0004);


	ce_cam_i2c_write(camlr, 0x25, 0x0020);
	ce_cam_i2c_write(camlr, 0xC2, 0x0840);

	ce_cam_i2c_write(camlr, 0x0C, 0x0001);

	ce_cam_i2c_write(camlr, 0x47, 0x0080);
	ce_cam_i2c_write(camlr, 0x48, 0x0000);
	ce_cam_i2c_write(camlr, 0x4C, 0x0002);

	ce_cam_i2c_write(camlr, 0x35, 0x0028);
	ce_cam_i2c_write(camlr, 0x36, 0x0028);

	ce_cam_i2c_write(camlr, 0xA5, 0x0030);
	ce_cam_i2c_write(camlr, 0xAD, 0x01E0);
	ce_cam_i2c_write(camlr, 0xAB, 0x0040);

	/*recommended register setting and performance impact   PDF-page14*/
	ce_cam_i2c_write(camlr, 0x20, 0x03C7);
	ce_cam_i2c_write(camlr, 0x24, 0x001B);
	ce_cam_i2c_write(camlr, 0x2B, 0x0003);
	ce_cam_i2c_write(camlr, 0x2F, 0x0003);
}

static void ce_cam_set_mt9v034_fps(int camlr)
{
	ce_cam_i2c_write(camlr, 0x05, ce_config_get_cf_img_HB());
	ce_cam_i2c_write(camlr, 0x06, ce_config_get_cf_img_VB());
}

static void ce_cam_set_mt9v034_EG_mode(int camlr)
{
	switch (ce_config_get_cf_cam_EG_mode())
	{
	case 0:
		ce_cam_i2c_write(camlr, 0xAF, 0x00);      //AEC

		ce_cam_i2c_write(camlr, 0x0B, ce_config_get_cf_cam_man_exp());   //Exposure Time
		ce_cam_i2c_write(camlr, 0x35, ce_config_get_cf_cam_man_gain());  //Gain
		break;
	case 1:
		ce_cam_i2c_write(camlr, 0xA5, ce_config_get_cf_cam_auto_EG_des());
		ce_cam_i2c_write(camlr, 0xA6, 0x01);
		ce_cam_i2c_write(camlr, 0xAC, ce_config_get_cf_cam_auto_EG_bottom());
		ce_cam_i2c_write(camlr, 0xAD, ce_config_get_cf_cam_auto_EG_top());
		ce_cam_i2c_write(camlr, 0xAE, 2);
		break;
	case 2:
		ce_cam_i2c_write(camlr, 0xAF, 0x01);
		ce_cam_i2c_write(camlr, 0xA5, ce_config_get_cf_cam_auto_EG_des());
		ce_cam_i2c_write(camlr, 0xA6, 0x01);
		ce_cam_i2c_write(camlr, 0xA8, 0x00);
		ce_cam_i2c_write(camlr, 0xAC, ce_config_get_cf_cam_auto_E_man_G_Ebottom());
		ce_cam_i2c_write(camlr, 0xAD, ce_config_get_cf_cam_auto_E_man_G_Etop());
		ce_cam_i2c_write(camlr, 0xAE, 2);
		ce_cam_i2c_write(camlr, 0x35, ce_config_get_cf_cam_auto_E_man_G());
		break;
	case 3:

		break;
	case 4:
		ce_cam_i2c_write(camlr, 0xA6, ce_config_get_cf_cam_agc_aec_skip_frame());
		ce_cam_i2c_write(camlr, 0xA8, 2);
		ce_cam_i2c_write(camlr, 0xA9, ce_config_get_cf_cam_agc_aec_skip_frame());
		ce_cam_i2c_write(camlr, 0xAA, 2);
		break;
	default:
		break;
	}
}

void ce_cam_get_soft_version(int camlr)
{
	unsigned char ver_buf[30];
	int j = libusb_control_transfer(ce_cam_get_cam_handle(camlr), RT_D2H, GET_THE_SOFT_VISION, 0, 0, ver_buf, 30, 1000);
	if (CAMD1_LEFT == camlr)
	{
		LOG("celog: the left cam version :");
	}
	else if (CAMD1_RIGHT == camlr)
	{
		LOG("celog: the right cam version :");
	}

	for (int i = 0; i< j; i++)
	{
		LOG("%c", ver_buf[i]);
	}

	LOG("\r\n");
}


int ce_cam_capture_init()
{
	int ret_val = 0;
	int r;
	r = ce_usb_open();
	int num_cy = r;
	if (r<1)
	{
		LOG("celog: No cameras found! r = %d\r\n", r);
		return ERROR;
	}
	else
	{
		LOG("celog: Number of device of interest found: %d\r\n", r);
	}


	libusb_device_handle *pusb_handle;
	for (int i = 0; i<num_cy; i++)
	{
		unsigned char buf = 0;
		pusb_handle = ce_usb_gethandle(i);
		int r_num = libusb_control_transfer(pusb_handle, RT_D2H, GET_CAM_LR, 0, 0, &buf, 1, 1000);
		if (r_num != 1)
		{
			LOG("celog: Get the device LR addr failed");
		}
		else
		{

			if (buf == 0xF1)
			{
				pcaml_handle = pusb_handle;
				LOG("Left camera found!\r\n");

				if (libusb_kernel_driver_active(pcaml_handle, 0) == 1)	//find out if kernel driver is attached
				{
					LOG() << "Kernel Driver Active" << endl;
					if (libusb_detach_kernel_driver(pcaml_handle, 0) == 0) //detach it
					{
						LOG() << "Kernel Driver Detached!" << endl;
					}
				}
				r = libusb_claim_interface(pcaml_handle, 0);            //claim interface 0 (the first) of device (mine had jsut 1)
				if (r < 0)
				{
					LOG() << "Cannot Claim Interface" << endl;
					return -3;
				}
				LOG() << "Claimed Interface" << endl;
			}
			else if (buf == 0xF0)
			{
				pcamr_handle = pusb_handle;
				LOG("Right camera found!\r\n");

				if (libusb_kernel_driver_active(pcamr_handle, 0) == 1)	//find out if kernel driver is attached
				{
					LOG() << "Kernel Driver Active" << endl;
					if (libusb_detach_kernel_driver(pcamr_handle, 0) == 0) //detach it
					{
						LOG() << "Kernel Driver Detached!" << endl;
					}
				}
				r = libusb_claim_interface(pcamr_handle, 0);            //claim interface 0 (the first) of device (mine had jsut 1)

				if (r < 0)
				{
					LOG() << "Cannot Claim Interface" << endl;
					return -4;
				}
				LOG() << "Claimed Interface" << endl;
			}
		}
	}

	if (ce_config_get_cf_cam_mode() & CAMD1_LEFT_ENABLE)
	{
		if (pcaml_handle == NULL)
			ret_val = ERROR;
	}

	if (ce_config_get_cf_cam_mode() & CAMD1_RIGHT_ENABLE)
	{
		if (pcamr_handle == NULL)
			ret_val = ERROR;
	}

	if (ERROR == ret_val)
	{
		LOG("celog: lost a cam left or right \r\n");
		return ERROR;
	}
	else
	{
		int temp = 0;
		int caml_addr = CAMD1_LEFT;
		int camr_addr = CAMD1_RIGHT;

		ce_cam_get_soft_version(caml_addr);
		ce_cam_get_soft_version(camr_addr);

		Sleep(1);
		return SUCCESS;
	}
}

void ce_cam_capture(CEThread *pThread)
{
	int camlr = pThread->GetCamLR();

	if (CAMD1_LEFT == camlr)
	{
		MySetThreadAffinityMask(LEFT_CAM_CAPTURE_THREAD_CPUID);
	}
	else if (CAMD1_RIGHT == camlr)
	{
		MySetThreadAffinityMask(RIGHT_CAM_CAPTURE_THREAD_CPUID);
	}

	ce_cam_set_mt9v034_config_default(camlr);

	ce_cam_set_mt9v034_fps(camlr);

	ce_cam_set_mt9v034_EG_mode(camlr);

	ce_cam_ctrl_camera(camlr, SET_MCLK_12MHz);

	//usleep(1000);
	Sleep(1);

	ce_cam_sync_rst(camlr);

	struct timeval cap_systime;

	int r, transferred = 0;
	unsigned char pass;

	img_pkg *timg_pkg;
	threadsafe_queue<img_pkg *> *tlist;
	if (CAMD1_LEFT == camlr)
	{
		tlist = &img_pkg_left_list;
	}
	else if (CAMD1_RIGHT == camlr)
	{
		tlist = &img_pkg_right_list;
	}

	libusb_device_handle *pcam_handle;
	pcam_handle = ce_cam_get_cam_handle(camlr);

	int buff_size = ce_config_get_cf_img_buff_size();
	int time_offset = ce_config_get_cf_img_time_offset();
	int img_size = ce_config_get_cf_img_size();
	while (1)
	{
		timg_pkg = new img_pkg;
		if (NULL == timg_pkg)
		{
			LOG("ce_cam_capture alloc memory failure! exit thread!\n");
			break;
		}

		memset(timg_pkg, 0, sizeof(img_pkg));
		r = libusb_bulk_transfer(pcam_handle, 0x82, timg_pkg->data, buff_size, &transferred, 1000);
		gettimeofday(&cap_systime, NULL);
		timg_pkg->timestamp = cap_systime.tv_sec + 0.000001*cap_systime.tv_usec - time_offset;

		if (r)
		{
			LOG("cam %d bulk transfer returned: %d\n", camlr, r);
		}

		pass = (timg_pkg->data[img_size + 0] == 0xFF
			&& timg_pkg->data[img_size + 1] == 0x00
			&& timg_pkg->data[img_size + 2] == 0xFE
			&& timg_pkg->data[img_size + 3] == 0x01);

		if (pass == 1)
		{
			img_pkg *timg_pkg_giveup = NULL;

			if (0 == tlist->push(timg_pkg, timg_pkg_giveup, 30))
			{
				delete timg_pkg_giveup;
			}
		}
		else
		{
			delete timg_pkg;
			timg_pkg = NULL;
			LOG("cam %d bulk transfer check failed: %d\n", camlr, r);

			ce_cam_ctrl_camera(camlr, SET_MCLK_12MHz);
		}

		if (!(pThread->IsRun()))//在每次循环判断是否可以运行，如果不行就退出循环
			break;
	}

}

void ce_cam_preprocess(CEThread *pThread)
{
	cv::Mat img_left(cv::Size(ce_config_get_cf_img_width(), ce_config_get_cf_img_height()), CV_8UC1);
	cv::Mat img_right(cv::Size(ce_config_get_cf_img_width(), ce_config_get_cf_img_height()), CV_8UC1);

	img_pkg timg_pkg;

	img_pkg *l_img_pkg = NULL;
	img_pkg *r_img_pkg = NULL;

    MySetThreadAffinityMask(CAM_PREPROCESS_THREAD_CPUID);

	while (1)
	{
		if ((!img_pkg_left_list.empty()) && (!img_pkg_right_list.empty()))
		{
			img_pkg_left_list.try_front(l_img_pkg);
			img_pkg_right_list.try_front(r_img_pkg);

			double diff_tamps = fabs(l_img_pkg->timestamp - r_img_pkg->timestamp);

			//qDebug() << diff_tamps;
			if (diff_tamps > 0.005)    // fps-27  diff  should not bigger than it.so set 5ms
			//if (0)    // fps-27  diff  should not bigger than it.so set 5ms
			{
				if (l_img_pkg->timestamp < r_img_pkg->timestamp)  // give up the early data
				{
					img_pkg_left_list.try_pop(l_img_pkg);
					delete l_img_pkg;
					l_img_pkg = NULL;
				}
				else
				{
					img_pkg_right_list.try_pop(r_img_pkg);
					delete r_img_pkg;
					r_img_pkg = NULL;
				}
			}
			else
			{
				img_pkg_left_list.try_pop(l_img_pkg);
				img_pkg_right_list.try_pop(r_img_pkg);

				d1_img_output_pkg *t_output_pkg = new d1_img_output_pkg;
				if (NULL == t_output_pkg)
				{
					printf("ce_cam_preprocess alloc memory failure!\n");
					delete l_img_pkg;
					delete r_img_pkg;
					continue;
				}

				t_output_pkg->left_img = l_img_pkg;
				t_output_pkg->right_img = r_img_pkg;

				t_output_pkg->left_img->timestamp = l_img_pkg->timestamp;      // merger the timestamp to left
				t_output_pkg->right_img->timestamp = l_img_pkg->timestamp;

				d1_img_output_pkg *t_pkg_giveup = NULL;

				img_pkg_list_d1.push(t_output_pkg, t_pkg_giveup, 30);
				if (NULL != t_pkg_giveup)
				{
					delete t_pkg_giveup->left_img;
					delete t_pkg_giveup->right_img;
					delete t_pkg_giveup;
				}
			}

		}
		if (!(pThread->IsRun()))//在每次循环判断是否可以运行，如果不行就退出循环
			break;
		Sleep(1);
	}
}

void ce_cam_showimg(CECamShowing *pThread)
{
	cv::Mat img_left(cv::Size(ce_config_get_cf_img_width(), ce_config_get_cf_img_height()), CV_8UC1);
	cv::Mat img_right(cv::Size(ce_config_get_cf_img_width(), ce_config_get_cf_img_height()), CV_8UC1);

    MySetThreadAffinityMask(CAM_SHOW_THREAD_CPUID);

	d1_img_output_pkg *img_lr_pkg;

	while (1)
	{
		if (!img_pkg_list_d1.try_pop(img_lr_pkg))
		{
			Sleep(1);
			continue;
		}

		memcpy(img_left.data, img_lr_pkg->left_img->data, ce_config_get_cf_img_size());
		//cv::imshow("left", img_left);

		memcpy(img_right.data, img_lr_pkg->right_img->data, ce_config_get_cf_img_size());
		//cv::imshow("right", img_right);

		pThread->ce_DisplayImageL(img_left);
		pThread->ce_DisplayImageR(img_right);

		delete img_lr_pkg->left_img;
		delete img_lr_pkg->right_img;
		delete img_lr_pkg;

		if (!(pThread->IsRun()))//在每次循环判断是否可以运行，如果不行就退出循环
			break;
	}

}
