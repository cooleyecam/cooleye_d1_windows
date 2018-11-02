#pragma once
#ifndef CEDRIVER_THREAD_H
#define CEDRIVER_THREAD_H

#include <QtWidgets/QMainWindow>
#include <QThread>
#include <QMetaType>
#include <QMutex>
#include <qDebug>

#include <QSerialPort>
#include <QSerialPortInfo>

#include <opencv2/opencv.hpp>
#include <iostream>


struct ceIMUListBuff
{
	unsigned char num;
	float rx, ry, rz;
	float ax, ay, az;
	float qw, qx, qy, qz;
	double timestamp;
};

class CEThread : public QThread
{
	Q_OBJECT
public slots:
    virtual void stopImmediately()
    {
        //QMutexLocker locker(&m_lock);
        m_isCanRun = false;
    }

	QMutex *GetMutex() {return &m_lock;}

	bool IsRun()
    {
        //QMutexLocker locker(&m_lock);
        return m_isCanRun;
    };
	int GetCamLR()
	{
		//QMutexLocker locker(&m_lock);
		return cam_lr;
	}

public:
	QMutex m_lock;
	bool m_isCanRun;
	int cam_lr;
};

class CECamD1CaptureL : public CEThread
{
	Q_OBJECT
public:
	virtual void run();
};

class CECamD1CaptureR : public CEThread
{
	Q_OBJECT
public:
	virtual void run();
};

class CEIMUPreProcess : public CEThread
{
	Q_OBJECT
public:
	virtual void run();

	void Init(QSerialPort *pSPort = NULL);

public slots:
	void ceSlotPreProcessIMU();
	QSerialPort* getSPort()
	{
		return m_pSPort;
	}
private:
	QSerialPort * m_pSPort;
};

class CEPreProcess : public CEThread
{
	Q_OBJECT
public:
	virtual void run();
};

class CECamShowing : public CEThread
{
	Q_OBJECT
public:
	virtual void run();

public slots:
	void ce_DisplayImageL(cv::Mat mat)
	{
		emit ceSignal_DisplayImageL(mat);
	}

	void ce_DisplayImageR(cv::Mat mat)
	{
		emit ceSignal_DisplayImageR(mat);
	}

signals:
	void ceSignal_DisplayImageL(cv::Mat mat);
	void ceSignal_DisplayImageR(cv::Mat mat);
	void ceSignal_PlotStamps(double stamp);
	void ceSignal_PlotDiffStamps(double stamp);
};

int MySetThreadAffinityMask(int nCpuId);

#endif

