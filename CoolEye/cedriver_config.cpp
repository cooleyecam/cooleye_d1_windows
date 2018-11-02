#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <fstream>

#include <QCoreApplication>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include <QDebug>
#include <QtWidgets>
#include <QThread>
#include <QImage>
#include <QMetaType>
#include <QVector>


#include "cedriver_config.h"
#include "cedriver_global_config.h"

using namespace std;

global_config_d1 gc_camd1;

static std::string ce_config_get_cf_string_para(std::vector<std::string> *tconfiglist, std::string cf_name)
{
	std::string stemp;

	for (int i = 0; i< tconfiglist->size(); i++)
	{
		stemp = tconfiglist->at(i).substr(0, tconfiglist->at(i).find("="));
		if (!stemp.compare(cf_name))
		{
			return tconfiglist->at(i).substr(tconfiglist->at(i).find("=") + 1, tconfiglist->at(i).find(";") - tconfiglist->at(i).find("=") - 1);
		}
	}

	return "GET DEVICE STRING ERROR!!!";
}

static int ce_config_get_cf_int_para(std::vector<std::string> *tconfiglist, std::string cf_name)
{
	std::string stemp;
	std::string scf = "ERROR";

	for (int i = 0; i< tconfiglist->size(); i++)
	{
		stemp = tconfiglist->at(i).substr(0, tconfiglist->at(i).find("="));
		if (!stemp.compare(cf_name))
		{
			scf = tconfiglist->at(i).substr(tconfiglist->at(i).find("=") + 1, tconfiglist->at(i).find(";") - tconfiglist->at(i).find("=") - 1);
			break;
		}
	}

	if (scf == "ERROR")
	{
		return ERROR;
	}
	else
	{
		return strtol(scf.c_str(), NULL, 10);
	}

}

static float ce_config_get_cf_float_para(std::vector<std::string> *tconfiglist, std::string cf_name)
{
	std::string stemp;
	std::string scf = "ERROR";

	for (int i = 0; i< tconfiglist->size(); i++)
	{
		stemp = tconfiglist->at(i).substr(0, tconfiglist->at(i).find("="));
		if (!stemp.compare(cf_name))
		{
			scf = tconfiglist->at(i).substr(tconfiglist->at(i).find("=") + 1, tconfiglist->at(i).find(";") - tconfiglist->at(i).find("=") - 1);
			break;
		}
	}

	if (scf == "ERROR")
	{
		return ERROR;
	}
	else
	{
		return strtof(scf.c_str(), NULL);
	}

}


static void ce_config_set_cf_float_para(string cecamd1_config_file, std::string cf_name, float pdata)
{

	std::ifstream in_settings(cecamd1_config_file.c_str());
	if (!in_settings)
	{
		std::cerr << "celog: opening file failed -- " << cecamd1_config_file << std::endl;
		exit(EXIT_FAILURE);
	}

	std::string filename;

	std::string timestamp;
	std::vector<std::string> settingsList;
	int index = 0;

	char temp[30] = { 0 };
	sprintf(temp, "%f", pdata);
	std::string scf = temp;

	std::string line_settings;
	while (getline(in_settings, line_settings))
	{
		std::string stemp = line_settings.substr(0, line_settings.find("="));
		if (!stemp.compare(cf_name))
		{
			line_settings.clear();
			line_settings = cf_name + "=" + scf + ";";
			index = settingsList.size();
		}

		settingsList.push_back(line_settings);
		line_settings.clear();
	}
	in_settings.close();

	std::ofstream out_settings(cecamd1_config_file.c_str());
	for (vector<std::string>::iterator iter = settingsList.begin(); iter != settingsList.end(); iter++)
	{
		out_settings << *iter << std::endl;
	}
	out_settings.close();
}


static void ce_config_get_img_config()
{
	switch (gc_camd1.gc_cam.cf_cam_resolution)
	{
	case CAMD1_RESOLUTION_VGA:
		gc_camd1.gc_img.cf_img_width = IMG_WIDTH_VGA;
		gc_camd1.gc_img.cf_img_height = IMG_HEIGHT_VGA;
		break;
	case CAMD1_RESOLUTION_WVGA:
		gc_camd1.gc_img.cf_img_width = IMG_WIDTH_WVGA;
		gc_camd1.gc_img.cf_img_height = IMG_HEIGHT_WVGA;
		break;
	default:
		gc_camd1.gc_img.cf_img_width = IMG_WIDTH_WVGA;
		gc_camd1.gc_img.cf_img_height = IMG_HEIGHT_WVGA;
		break;
	}

	gc_camd1.gc_img.cf_img_size = gc_camd1.gc_img.cf_img_width * gc_camd1.gc_img.cf_img_height;
	gc_camd1.gc_img.cf_img_buff_size = gc_camd1.gc_img.cf_img_size + 0x200;

	/////////HS--VS///////////////////////////////
	if (gc_camd1.gc_cam.cf_cam_FPS < 10)
	{
		gc_camd1.gc_cam.cf_cam_FPS = 10;
		gc_camd1.gc_img.cf_img_HB = 0x01FF;
	}
	else if (gc_camd1.gc_cam.cf_cam_FPS < 19)
		gc_camd1.gc_img.cf_img_HB = 0x01FF;
	else if (gc_camd1.gc_cam.cf_cam_FPS < 23)
		gc_camd1.gc_img.cf_img_HB = 0x00FF;
	else if (gc_camd1.gc_cam.cf_cam_FPS < 26)
		gc_camd1.gc_img.cf_img_HB = 0x008F;
	else if (gc_camd1.gc_cam.cf_cam_FPS < 28)
		gc_camd1.gc_img.cf_img_HB = 0x004F;
	else
	{
		gc_camd1.gc_cam.cf_cam_FPS = 27;
		gc_camd1.gc_img.cf_img_HB = 0x004F;
	}


	// WIDTH *(A + Q) + REG06(A + Q)  +4  = 1/FPS
	// A = cf_img_width    Q =  cf_img_VB  = reg05
	gc_camd1.gc_img.cf_img_VB = (CAMD1_SYS_CLKIN / gc_camd1.gc_cam.cf_cam_FPS - 4) / (gc_camd1.gc_img.cf_img_width + gc_camd1.gc_img.cf_img_HB) - gc_camd1.gc_img.cf_img_height;


	if (gc_camd1.gc_img.cf_img_VB < 0x2D)
		gc_camd1.gc_img.cf_img_VB = 0x2D;

	std::cout << "cf_img_HB: " << gc_camd1.gc_img.cf_img_HB << std::endl;
	std::cout << "cf_img_VB: " << gc_camd1.gc_img.cf_img_VB << std::endl;


	// time offset
	gc_camd1.gc_img.cf_img_time_offset = (1.0 / CAMD1_SYS_CLKIN) * (gc_camd1.gc_img.cf_img_width + gc_camd1.gc_img.cf_img_HB) * 480;

	std::cout << "cf_img_time_offset: " << gc_camd1.gc_img.cf_img_time_offset << std::endl;
}

void ce_config_load_settings(const char* settings_file)
{
	/******************************* Load Settings File ***************************************/
	std::string cecamd1_config_file = settings_file;

#if 1
	std::ifstream ifs_settings(cecamd1_config_file.c_str());	
	if (!ifs_settings)
	{
		qDebug() << "celog: opening file failed -- " << cecamd1_config_file.c_str();
		exit(EXIT_FAILURE);
	}
#else
	std::ifstream ifs_settings;
	ifs_settings.open(cecamd1_config_file.c_str());
	if (!ifs_settings.bad())
	{
		qDebug() << "celog: opening file failed -- " << cecamd1_config_file.c_str();
		exit(EXIT_FAILURE);
	}
#endif
	std::string filename;
	std::string line_settings;
	std::string timestamp;
	std::vector<std::string> settingsList;
	int linecount = 0;


	while (getline(ifs_settings, line_settings))
	{
		if (line_settings.at(0) != '#')
		{
			settingsList.push_back(line_settings);
			linecount++;
		}
	}
	ifs_settings.close();


	gc_camd1.gc_dev.cf_dev_name = ce_config_get_cf_string_para(&settingsList, "cf_dev_name");
	gc_camd1.gc_dev.cf_dev_version = ce_config_get_cf_string_para(&settingsList, "cf_dev_version");

	std::cout << "cf_dev_name: " << gc_camd1.gc_dev.cf_dev_name << std::endl;
	std::cout << "cf_dev_version: " << gc_camd1.gc_dev.cf_dev_version << std::endl;



	gc_camd1.gc_cam.cf_cam_mode = ce_config_get_cf_int_para(&settingsList, "cf_cam_mode");
	gc_camd1.gc_cam.cf_cam_resolution = ce_config_get_cf_int_para(&settingsList, "cf_cam_resolution");
	gc_camd1.gc_cam.cf_cam_FPS = ce_config_get_cf_int_para(&settingsList, "cf_cam_FPS");
	gc_camd1.gc_cam.cf_cam_EG_mode = ce_config_get_cf_int_para(&settingsList, "cf_cam_EG_mode");
	gc_camd1.gc_cam.cf_cam_man_exp = ce_config_get_cf_int_para(&settingsList, "cf_cam_man_exp");
	gc_camd1.gc_cam.cf_cam_man_gain = ce_config_get_cf_int_para(&settingsList, "cf_cam_man_gain");
	gc_camd1.gc_cam.cf_cam_auto_EG_top = ce_config_get_cf_int_para(&settingsList, "cf_cam_auto_EG_top");
	gc_camd1.gc_cam.cf_cam_auto_EG_bottom = ce_config_get_cf_int_para(&settingsList, "cf_cam_auto_EG_bottom");
	gc_camd1.gc_cam.cf_cam_auto_EG_des = ce_config_get_cf_int_para(&settingsList, "cf_cam_auto_EG_des");
	gc_camd1.gc_cam.cf_cam_auto_E_man_G_Etop = ce_config_get_cf_int_para(&settingsList, "cf_cam_auto_E_man_G_Etop");
	gc_camd1.gc_cam.cf_cam_auto_E_man_G_Ebottom = ce_config_get_cf_int_para(&settingsList, "cf_cam_auto_E_man_G_Ebottom");
	gc_camd1.gc_cam.cf_cam_auto_E_man_G = ce_config_get_cf_int_para(&settingsList, "cf_cam_auto_E_man_G");
	gc_camd1.gc_cam.cf_cam_agc_aec_skip_frame = ce_config_get_cf_int_para(&settingsList, "cf_cam_agc_aec_skip_frame");

	std::cout << "cf_cam_mode: " << gc_camd1.gc_cam.cf_cam_mode << std::endl;
	std::cout << "cf_cam_FPS: " << gc_camd1.gc_cam.cf_cam_FPS << std::endl;
	std::cout << "cf_cam_EG_mode: " << gc_camd1.gc_cam.cf_cam_EG_mode << std::endl;
	std::cout << "cf_cam_man_exp: " << gc_camd1.gc_cam.cf_cam_man_exp << std::endl;
	std::cout << "cf_cam_man_gain: " << gc_camd1.gc_cam.cf_cam_man_gain << std::endl;
	std::cout << "cf_cam_auto_EG_top: " << gc_camd1.gc_cam.cf_cam_auto_EG_top << std::endl;
	std::cout << "cf_cam_auto_EG_bottom: " << gc_camd1.gc_cam.cf_cam_auto_EG_bottom << std::endl;
	std::cout << "cf_cam_auto_EG_des: " << gc_camd1.gc_cam.cf_cam_auto_EG_des << std::endl;
	std::cout << "cf_cam_auto_E_man_G_Etop: " << gc_camd1.gc_cam.cf_cam_auto_E_man_G_Etop << std::endl;
	std::cout << "cf_cam_auto_E_man_G_Ebottom: " << gc_camd1.gc_cam.cf_cam_auto_E_man_G_Ebottom << std::endl;
	std::cout << "cf_cam_auto_E_man_G: " << gc_camd1.gc_cam.cf_cam_auto_E_man_G << std::endl;
	std::cout << "cf_cam_agc_aec_skip_frame: " << gc_camd1.gc_cam.cf_cam_agc_aec_skip_frame << std::endl;


	gc_camd1.gc_imu.cf_imu_uart = ce_config_get_cf_string_para(&settingsList, "cf_imu_uart");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_bias_X");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_bias_Y");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_bias_Z");
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_gyro_bias_X");
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_gyro_bias_Y");
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_gyro_bias_Z");

	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][0] = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_T[0][0]");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][1] = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_T[0][1]");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][2] = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_T[0][2]");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][0] = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_T[1][0]");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][1] = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_T[1][1]");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][2] = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_T[1][2]");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][0] = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_T[2][0]");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][1] = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_T[2][1]");
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][2] = ce_config_get_cf_float_para(&settingsList, "cf_imu_icm20689_acc_T[2][2]");


	gc_camd1.gc_imu.cf_imu_icm20689_sample_rate = ce_config_get_cf_int_para(&settingsList, "cf_imu_icm20689_sample_rate");

	std::cout << "cf_imu_uart: " << gc_camd1.gc_imu.cf_imu_uart << std::endl;
	std::cout << "cf_imu_acc_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X << std::endl;
	std::cout << "cf_imu_acc_bias_Y: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y << std::endl;
	std::cout << "cf_imu_acc_bias_Z: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z << std::endl;
	std::cout << "cf_imu_gyro_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X << std::endl;
	std::cout << "cf_imu_gyro_bias_Y: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y << std::endl;
	std::cout << "cf_imu_gyro_bias_Z: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z << std::endl;

	std::cout << "cf_imu_acc_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X << std::endl;


	std::cout << "cf_imu_icm20689_acc_T[0][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][0] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[0][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][1] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[0][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][2] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[1][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][0] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[1][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][1] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[1][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][2] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[2][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][0] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[2][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][1] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[2][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][2] << std::endl;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	ce_config_get_img_config();
}


void ce_config_temp_load_settings(void)
{


	gc_camd1.gc_dev.cf_dev_name = "CoolEyeD1";
	gc_camd1.gc_dev.cf_dev_version = "V1.0";

	qDebug() << "cf_dev_name: " << gc_camd1.gc_dev.cf_dev_name.c_str();
	qDebug() << "cf_dev_version: " << gc_camd1.gc_dev.cf_dev_version.c_str();

	gc_camd1.gc_cam.cf_cam_mode = 3;
	gc_camd1.gc_cam.cf_cam_resolution = 2;
	gc_camd1.gc_cam.cf_cam_FPS = 20;
	gc_camd1.gc_cam.cf_cam_EG_mode = 0;
	gc_camd1.gc_cam.cf_cam_man_exp = 200;
	gc_camd1.gc_cam.cf_cam_man_gain = 50;
	gc_camd1.gc_cam.cf_cam_auto_EG_top = 200;
	gc_camd1.gc_cam.cf_cam_auto_EG_bottom = 5;
	gc_camd1.gc_cam.cf_cam_auto_EG_des = 158;
	gc_camd1.gc_cam.cf_cam_auto_E_man_G_Etop = 300;
	gc_camd1.gc_cam.cf_cam_auto_E_man_G_Ebottom = 4;
	gc_camd1.gc_cam.cf_cam_auto_E_man_G = 500;
	gc_camd1.gc_cam.cf_cam_agc_aec_skip_frame = 0;

	qDebug() << "cf_cam_mode: " << gc_camd1.gc_cam.cf_cam_mode;
	qDebug() << "cf_cam_FPS: " << gc_camd1.gc_cam.cf_cam_FPS;
	qDebug() << "cf_cam_EG_mode: " << gc_camd1.gc_cam.cf_cam_EG_mode;
	qDebug() << "cf_cam_man_exp: " << gc_camd1.gc_cam.cf_cam_man_exp;
	qDebug() << "cf_cam_man_gain: " << gc_camd1.gc_cam.cf_cam_man_gain;
	qDebug() << "cf_cam_auto_EG_top: " << gc_camd1.gc_cam.cf_cam_auto_EG_top;
	qDebug() << "cf_cam_auto_EG_bottom: " << gc_camd1.gc_cam.cf_cam_auto_EG_bottom;
	qDebug() << "cf_cam_auto_EG_des: " << gc_camd1.gc_cam.cf_cam_auto_EG_des;
	qDebug() << "cf_cam_auto_E_man_G_Etop: " << gc_camd1.gc_cam.cf_cam_auto_E_man_G_Etop;
	qDebug() << "cf_cam_auto_E_man_G_Ebottom: " << gc_camd1.gc_cam.cf_cam_auto_E_man_G_Ebottom;
	qDebug() << "cf_cam_auto_E_man_G: " << gc_camd1.gc_cam.cf_cam_auto_E_man_G;
	qDebug() << "cf_cam_agc_aec_skip_frame: " << gc_camd1.gc_cam.cf_cam_agc_aec_skip_frame;


	gc_camd1.gc_imu.cf_imu_uart = "/dev/ttyUSB0";
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z =0;

	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][0] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][1] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][2] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][0] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][2] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][0] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][1] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][2] = 0;


	gc_camd1.gc_imu.cf_imu_icm20689_sample_rate = 1000;

	qDebug() << "cf_imu_uart: " << gc_camd1.gc_imu.cf_imu_uart.c_str();
	qDebug() << "cf_imu_acc_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X;
	qDebug() << "cf_imu_acc_bias_Y: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y;
	qDebug() << "cf_imu_acc_bias_Z: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z;
	qDebug() << "cf_imu_gyro_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X;
	qDebug() << "cf_imu_gyro_bias_Y: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y;
	qDebug() << "cf_imu_gyro_bias_Z: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z;

	qDebug() << "cf_imu_acc_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X;


	qDebug() << "cf_imu_icm20689_acc_T[0][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][0];
	qDebug() << "cf_imu_icm20689_acc_T[0][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][1];
	qDebug() << "cf_imu_icm20689_acc_T[0][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][2];
	qDebug() << "cf_imu_icm20689_acc_T[1][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][0];
	qDebug() << "cf_imu_icm20689_acc_T[1][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][1];
	qDebug() << "cf_imu_icm20689_acc_T[1][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][2];
	qDebug() << "cf_imu_icm20689_acc_T[2][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][0];
	qDebug() << "cf_imu_icm20689_acc_T[2][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][1];
	qDebug() << "cf_imu_icm20689_acc_T[2][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][2];
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	ce_config_get_img_config();
}



void  ce_config_rst_imu_offset(void)
{
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z = 0;

	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][0] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][1] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][2] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][0] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][1] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][2] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][0] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][1] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][2] = 0;
	gc_camd1.gc_imu.cf_imu_icm20689_sample_rate = 1000;


	std::cout << "-----------------------------" << std::endl;
	std::cout << "celog: begien the cali IMU..." << std::endl;
	std::cout << "celog: reset all the bias and scale..." << std::endl;
	std::cout << "cf_imu_acc_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X << std::endl;
	std::cout << "cf_imu_acc_bias_Y: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y << std::endl;
	std::cout << "cf_imu_acc_bias_Z: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z << std::endl;
	std::cout << "cf_imu_gyro_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X << std::endl;
	std::cout << "cf_imu_gyro_bias_Y: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y << std::endl;
	std::cout << "cf_imu_gyro_bias_Z: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z << std::endl;

	std::cout << "cf_imu_icm20689_acc_T[0][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][0] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[0][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][1] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[0][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][2] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[1][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][0] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[1][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][1] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[1][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][2] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[2][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][0] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[2][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][1] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[2][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][2] << std::endl;
	std::cout << "cf_imu_icm20689_sample_rate " << gc_camd1.gc_imu.cf_imu_icm20689_sample_rate << std::endl;
	std::cout << "-----------------------------" << std::endl;
}


void  ce_config_rewrite_imu_offset(std::string filepath, float *gyro_offs, float *accel_offs, float **accel_T)
{
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X = gyro_offs[0];
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y = gyro_offs[1];
	gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z = gyro_offs[2];

	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X = accel_offs[0];
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y = accel_offs[1];
	gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z = accel_offs[2];

	memcpy(gc_camd1.gc_imu.cf_imu_icm20689_acc_T, accel_T, sizeof(gc_camd1.gc_imu.cf_imu_icm20689_acc_T));

	std::cout << "----------------------------------------------------" << std::endl;
	std::cout << "set cf float: " << std::endl;

	std::cout << "cf_imu_gyro_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X << std::endl;
	std::cout << "cf_imu_gyro_bias_Y: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y << std::endl;
	std::cout << "cf_imu_gyro_bias_Z: " << gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z << std::endl;

	std::cout << "cf_imu_acc_bias_X: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X << std::endl;
	std::cout << "cf_imu_acc_bias_Y: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y << std::endl;
	std::cout << "cf_imu_acc_bias_Z: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z << std::endl;


	std::cout << "cf_imu_icm20689_acc_T[0][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][0] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[0][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][1] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[0][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][2] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[1][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][0] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[1][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][1] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[1][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][2] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[2][0]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][0] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[2][1]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][1] << std::endl;
	std::cout << "cf_imu_icm20689_acc_T[2][2]: " << gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][2] << std::endl;

	std::cout << "1----------------------------------------------------" << std::endl;
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_bias_X", gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_bias_Y", gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_bias_Z", gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z);
	std::cout << "2----------------------------------------------------" << std::endl;
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_gyro_bias_X", gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_gyro_bias_Y", gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_gyro_bias_Z", gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z);
	std::cout << "3----------------------------------------------------" << std::endl;
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_T[0][0]", gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][0]);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_T[0][1]", gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][1]);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_T[0][2]", gc_camd1.gc_imu.cf_imu_icm20689_acc_T[0][2]);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_T[1][0]", gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][0]);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_T[1][1]", gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][1]);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_T[1][2]", gc_camd1.gc_imu.cf_imu_icm20689_acc_T[1][2]);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_T[2][0]", gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][0]);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_T[2][1]", gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][1]);
	ce_config_set_cf_float_para(filepath, "cf_imu_icm20689_acc_T[2][2]", gc_camd1.gc_imu.cf_imu_icm20689_acc_T[2][2]);


	std::cout << "celog: writ the answer to the config success..." << std::endl;
	std::cout << "----------------------------------------------------" << std::endl;
}


























std::string ce_config_get_cf_dev_name() { return gc_camd1.gc_dev.cf_dev_name; }
std::string ce_config_get_cf_dev_version() { return gc_camd1.gc_dev.cf_dev_version; }


std::string ce_config_get_cf_imu_uart() { return gc_camd1.gc_imu.cf_imu_uart; }
int ce_config_get_cf_imu_icm20689_acc_bias_X() { return gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_X; }
int ce_config_get_cf_imu_icm20689_acc_bias_Y() { return gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Y; }
int ce_config_get_cf_imu_icm20689_acc_bias_Z() { return gc_camd1.gc_imu.cf_imu_icm20689_acc_bias_Z; }
int ce_config_get_cf_imu_icm20689_gyro_bias_X() { return gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_X; }
int ce_config_get_cf_imu_icm20689_gyro_bias_Y() { return gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Y; }
int ce_config_get_cf_imu_icm20689_gyro_bias_Z() { return gc_camd1.gc_imu.cf_imu_icm20689_gyro_bias_Z; }

void ce_config_get_cf_imu_icm20689_acc_T(float **mat)
{
	memcpy(mat, gc_camd1.gc_imu.cf_imu_icm20689_acc_T, sizeof(gc_camd1.gc_imu.cf_imu_icm20689_acc_T));
}



int ce_config_get_cf_imu_icm20689_sample_rate() { return gc_camd1.gc_imu.cf_imu_icm20689_sample_rate; }

int ce_config_get_cf_cam_mode() { return gc_camd1.gc_cam.cf_cam_mode; }
int ce_config_get_cf_cam_resolution() { return gc_camd1.gc_cam.cf_cam_resolution; }
int ce_config_get_cf_cam_FPS() { return gc_camd1.gc_cam.cf_cam_FPS; }
int ce_config_get_cf_cam_EG_mode() { return gc_camd1.gc_cam.cf_cam_EG_mode; }
int ce_config_get_cf_cam_man_exp() { return gc_camd1.gc_cam.cf_cam_man_exp; }
int ce_config_get_cf_cam_man_gain() { return gc_camd1.gc_cam.cf_cam_man_gain; }
int ce_config_get_cf_cam_auto_EG_top() { return gc_camd1.gc_cam.cf_cam_auto_EG_top; }
int ce_config_get_cf_cam_auto_EG_bottom() { return gc_camd1.gc_cam.cf_cam_auto_EG_bottom; }
int ce_config_get_cf_cam_auto_EG_des() { return gc_camd1.gc_cam.cf_cam_auto_EG_des; }
int ce_config_get_cf_cam_auto_E_man_G_Etop() { return gc_camd1.gc_cam.cf_cam_auto_E_man_G_Etop; }
int ce_config_get_cf_cam_auto_E_man_G_Ebottom() { return gc_camd1.gc_cam.cf_cam_auto_E_man_G_Ebottom; }
int ce_config_get_cf_cam_auto_E_man_G() { return gc_camd1.gc_cam.cf_cam_auto_E_man_G; }
int ce_config_get_cf_cam_agc_aec_skip_frame() { return gc_camd1.gc_cam.cf_cam_agc_aec_skip_frame; }


int ce_config_get_cf_img_width() { return gc_camd1.gc_img.cf_img_width; }
int ce_config_get_cf_img_height() { return gc_camd1.gc_img.cf_img_height; }
int ce_config_get_cf_img_size() { return gc_camd1.gc_img.cf_img_size; }
int ce_config_get_cf_img_buff_size() { return gc_camd1.gc_img.cf_img_buff_size; }
int ce_config_get_cf_img_HB() { return gc_camd1.gc_img.cf_img_HB; }
int ce_config_get_cf_img_VB() { return gc_camd1.gc_img.cf_img_VB; }
double ce_config_get_cf_img_time_offset() { return gc_camd1.gc_img.cf_img_time_offset; }


