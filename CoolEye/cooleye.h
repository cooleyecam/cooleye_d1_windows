#pragma once
#ifndef COOLEYE_H
#define COOLEYE_H

#include <QtWidgets/QMainWindow>
#include <QThread>
#include <QMetaType>
#include <QMutex>
#include <qDebug>
#include "ui_cooleye.h"
#include <QSerialPort>
#include <QSerialPortInfo>


#include "cedriver_cam.h"
#include "libusb.h"

#include <opencv2/opencv.hpp>
#include <iostream>


class CoolEye : public QMainWindow
{
	Q_OBJECT

public:
	CoolEye(QWidget *parent = Q_NULLPTR);
	~CoolEye();

private:
	Ui::CoolEyeClass ui;

	CECamD1CaptureL	camD1_L_capture;
	CECamD1CaptureR	camD1_R_capture;
	CEPreProcess	PreProcess;
	CECamShowing	CamShowing;
	CEIMUPreProcess	IMUD1PreProcess;
	QSerialPort		ceD1_IMUPort;
	QByteArray		ceIMUData;

private slots:

	void SL_Caputre(void);
	void SL_IMU(void);
	void SL_Switch_LR(void);
	void ceSlotDisplayL(cv::Mat mat);
	void ceSlotDisplayR(cv::Mat mat);
};

#endif
