
#include "libusb.h"

#include "cedriver_thread.h"
#include "cedriver_cam.h"
#include "cedriver_usb.h"
#include "cedriver_global_config.h"
#include "cedriver_threadsafe_queue.h"

#include <queue>
#include <QMutex>

#include <opencv2/opencv.hpp> //包含头文件
#include <iostream>

void CECamD1CaptureL::run()
{
	m_isCanRun = true;
	cam_lr = CAMD1_LEFT;
	ce_cam_capture(this);
}

void CECamD1CaptureR::run()
{
	m_isCanRun = true;
	cam_lr = CAMD1_RIGHT;
	ce_cam_capture(this);
}

void CEIMUPreProcess::Init(QSerialPort *pSPort)
{
	m_pSPort = pSPort;
}

void CEIMUPreProcess::run()
{
	m_isCanRun = true;

    MySetThreadAffinityMask(IMU_THREAD_CPUID);

	while (1)
	{
		Sleep(1000);

        if (!IsRun())//在每次循环判断是否可以运行，如果不行就退出循环
			break;
	}

}

void CEIMUPreProcess::ceSlotPreProcessIMU()
{
	QByteArray tIMUData;
	ceIMUListBuff tIMUListBuff;
	struct timeval getIMUTime;


	const char head_55 = 0x55;
	const char head_AA = 0xAA;
	char check_sum = 0;

	tIMUData = m_pSPort->readAll();

	gettimeofday(&getIMUTime, NULL);
	double timestamp = getIMUTime.tv_sec + 0.000001*getIMUTime.tv_usec - 0.0035;

	if (32 != tIMUData.size()) // 如果不是32个 证明不是最新数据 丢
		return;

	if (head_55 != tIMUData[0])
		return;

	if (head_AA != tIMUData[1])
		return;


	check_sum = 0;
	for (int i = 2; i < 31; i++)
	{
		check_sum = check_sum + tIMUData[i];
	}

	if (check_sum != tIMUData[31])
		return;



	int imu_data[10];

	tIMUListBuff.num = tIMUData[2];

	imu_data[0] = *(short*)(&tIMUData[3]);
	imu_data[1] = *(short*)(&tIMUData[5]);
	imu_data[2] = *(short*)(&tIMUData[7]);
	imu_data[3] = *(short*)(&tIMUData[9]);
	imu_data[4] = *(short*)(&tIMUData[11]);
	imu_data[5] = *(short*)(&tIMUData[13]);
	imu_data[6] = *(int*)(&tIMUData[15]);
	imu_data[7] = *(int*)(&tIMUData[19]);
	imu_data[8] = *(int*)(&tIMUData[23]);
	imu_data[9] = *(int*)(&tIMUData[27]);

	tIMUListBuff.rx = 1.0*imu_data[0] / 32768 * 2000;
	tIMUListBuff.ry = 1.0*imu_data[1] / 32768 * 2000;
	tIMUListBuff.rz = 1.0*imu_data[2] / 32768 * 2000;

	//float facc_nobias[3] = {
	//	(float)(imu_data[3] - acc_offset[0]),
	//	(float)(imu_data[4] - acc_offset[1]),
	//	(float)(imu_data[5] - acc_offset[2]) };

	float facc_nobias[3] = {
		(float)(imu_data[3] - 0),
		(float)(imu_data[4] - 0),
		(float)(imu_data[5] - 0) };


	tIMUListBuff.ax = 1.0*facc_nobias[0] / 16384 * 9.81;
	tIMUListBuff.ay = 1.0*facc_nobias[1] / 16384 * 9.81;
	tIMUListBuff.az = 1.0*facc_nobias[2] / 16384 * 9.81;


	float quat[4] = {
		(float)imu_data[6],
		(float)imu_data[7],
		(float)imu_data[8],
		(float)imu_data[9] };

	tIMUListBuff.qw = quat[0];
	tIMUListBuff.qx = quat[1];
	tIMUListBuff.qy = quat[2];
	tIMUListBuff.qz = quat[3];

	tIMUListBuff.timestamp = timestamp;

}

/////////////////////////////////////////////////////////

void CEPreProcess::run()
{
	m_isCanRun = true;
	ce_cam_preprocess(this);
}

void CECamShowing::run()
{
	m_isCanRun = true;
	ce_cam_showimg(this);
}

int MySetThreadAffinityMask(int nCpuId)
{
	if (0 == SetThreadAffinityMask(GetCurrentThread(), nCpuId))
	{
		LOG("Set CPU[%d] affinity failed! threadid[%lu] file:%s(%d),function:%s\n",
			nCpuId, GetCurrentThreadId(), __FILE__, __LINE__, __FUNCTION__);

		return ERROR;
	}
	else
	{
		LOG("Set CPU[%d] affinitysucceed! threadid[%lu] file:%s(%d),function:%s\n",
			nCpuId, GetCurrentThreadId(), __FILE__, __LINE__, __FUNCTION__);

		return SUCCESS;
	}
}