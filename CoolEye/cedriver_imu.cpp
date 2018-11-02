#include <QtWidgets/QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QDataStream>
#include <QThread>
#include <queue>
#include <time.h>
//#include <pthread.h>

#include <QSerialPort>
#include <QSerialPortInfo>

#include "libusb.h"
#include "cedriver_usb.h"
#include "cedriver_cam.h"
#include "cedriver_imu.h"
#include "cedriver_thread.h"
#include "cedriver_config.h"
#include "cedriver_global_config.h"
#include "cedriver_threadsafe_queue.h"



threadsafe_queue<icm20689_pkg *> icm20689_pkg_list;


int ce_imu_uart_fd = 0;

void ce_imu_data_feed(CEIMUPreProcess *pThread)
{

	QSerialPort	*sPort = (pThread->getSPort());
	
	unsigned char imu_frame[IMU_FRAME_LEN];
	unsigned char imu_frame_buf[2 * IMU_FRAME_LEN];

	memset(imu_frame, 0, IMU_FRAME_LEN);
	memset(imu_frame_buf, 0, 2 * IMU_FRAME_LEN);

	double time_interval = (double)1 / ce_config_get_cf_imu_icm20689_sample_rate();
	double timestamp;
	double start_time;
	double last_time;
	double new_time;
	double last_pred;
	double new_pred;

	struct timeval getIMUTime;

	gettimeofday(&getIMUTime, NULL);

	start_time = getIMUTime.tv_sec + 0.000001 * getIMUTime.tv_usec;

	new_time = start_time;
	new_pred = start_time;

	last_time = start_time - (double)1 / ce_config_get_cf_imu_icm20689_sample_rate();
	last_pred = start_time - (double)1 / ce_config_get_cf_imu_icm20689_sample_rate();

	int num_get = 0;
	
	sPort->clear();

	while (1)
	{

		//start_time = getIMUTime.tv_sec + 0.000001 * getIMUTime.tv_usec;

		//while (num_get < IMU_FRAME_LEN)
		//{
		//	int temp_read_get = sPort->read(ce_imu_uart_fd, &imu_frame_buf[num_get], IMU_FRAME_LEN);
		//	QByteArray response_data = sPort->waitForReadyRead(IMU_FRAME_LEN);

		//	num_get += temp_read_get;
		//}

		////std::cout << "start_time :"<< std::setprecision(15)  << start_time <<std::endl;
		//gettimeofday(&getIMUTime, NULL);

		//int position_pkghead = ce_imu_find_pkg_head(imu_frame_buf, num_get);
		//if (position_pkghead == -2)
		//{
		//	memset(imu_frame_buf, 0, 2 * IMU_FRAME_LEN);
		//	num_get = 0;
		//	continue;
		//}
		//else if (position_pkghead == -1)
		//{
		//	memset(imu_frame_buf, 0, 2 * IMU_FRAME_LEN);
		//	num_get = 1;
		//	imu_frame_buf[0] = CMD_HEAD1;
		//	continue;
		//}

		//int valid_len = num_get - position_pkghead;

		//if (valid_len<IMU_FRAME_LEN)
		//{
		//	for (int i = 0; i<valid_len; i++)
		//		imu_frame_buf[i] = imu_frame_buf[position_pkghead + i];

		//	for (int i = valid_len; i<2 * IMU_FRAME_LEN; i++)
		//		imu_frame_buf[i] = 0;

		//	num_get = valid_len;
		//	continue;
		//}

		//unsigned char checksum = 0;
		//for (int i = 2; i<(IMU_FRAME_LEN - 1); i++)checksum += imu_frame_buf[position_pkghead + i];
		//if (checksum != imu_frame_buf[position_pkghead + (IMU_FRAME_LEN - 1)])
		//{
		//	for (int i = 2; i<valid_len; i++)
		//		imu_frame_buf[i - 2] = imu_frame_buf[position_pkghead + i];

		//	for (int i = valid_len - 2; i<2 * IMU_FRAME_LEN; i++)
		//		imu_frame_buf[i] = 0;

		//	num_get = valid_len - 2;
		//	continue;
		//}

		//if (valid_len > IMU_FRAME_LEN)
		//{
		//	tcflush(ce_imu_uart_fd, TCIFLUSH);
		//	memset(imu_frame_buf, 0, 2 * IMU_FRAME_LEN);
		//	num_get = 0;
		//	continue;
		//}

		//new_time = getIMUTime.tv_sec + 0.000001*getIMUTime.tv_usec;

		//double newtime_interval = new_time - last_time;
		//if (newtime_interval<0.004 || newtime_interval>0.006)
		//	newtime_interval = 0.005;
		//time_interval = time_interval * 0.999 + 0.001*newtime_interval;

		//if (new_time - last_pred < time_interval)
		//	new_pred = new_time;
		//else
		//	new_pred = last_pred + time_interval + 0.001*(new_time - last_pred - time_interval);

		//last_time = new_time;
		//last_pred = new_pred;

		//memcpy(imu_frame, &imu_frame_buf[position_pkghead], IMU_FRAME_LEN);
		//timestamp = new_pred;

		//icm20689_pkg *ticm20689_pkg = new icm20689_pkg;
		//if (NULL == ticm20689_pkg)
		//{
		//	printf("ce_imu_data_feed alloc memory failure! exit thread!\n");
		//	break;
		//}
		//memset(ticm20689_pkg, 0, sizeof(ticm20689_pkg));

		//ce_imu_format_imu_frame(imu_frame, timestamp, ticm20689_pkg);


		//icm20689_pkg *icm20689_giveup = NULL;
		//icm20689_pkg_list.push(ticm20689_pkg, icm20689_giveup, IMU_FIFO_SIZE - 5);
		//if (NULL != icm20689_giveup)
		//{
		//	delete icm20689_giveup;
		//	icm20689_giveup = NULL;
		//}


		//memset(imu_frame_buf, 0, 2 * IMU_FRAME_LEN);
		//num_get = 0;


		if (!(pThread->IsRun()))//在每次循环判断是否可以运行，如果不行就退出循环
			break;
	}

}


