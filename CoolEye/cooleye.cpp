#include <Windows.h>

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

#include "CoolEye.h"
#include "qcustomplot.h"
#include "cedriver_config.h"
#include "cedriver_cam.h"
#include "cedriver_global_config.h"
#include "cedriver_threadsafe_queue.h"


CoolEye::CoolEye(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	showMaximized();

	ce_config_load_settings("cecfg_std.txt");	
								
	ui.label_image_caml->resize(ce_config_get_cf_img_width(), ce_config_get_cf_img_height());
	ui.label_image_camr->resize(ce_config_get_cf_img_width(), ce_config_get_cf_img_height());

	qRegisterMetaType<cv::Mat>("cv::Mat");

	connect(&CamShowing, SIGNAL(ceSignal_DisplayImageL(cv::Mat)), this, SLOT(ceSlotDisplayL(cv::Mat)));
	connect(&CamShowing, SIGNAL(ceSignal_DisplayImageR(cv::Mat)), this, SLOT(ceSlotDisplayR(cv::Mat)));

	connect(ui.pB_Capture, SIGNAL(clicked(bool)), this, SLOT(SL_Caputre()));
	connect(ui.pB_Capture_2, SIGNAL(clicked(bool)), this, SLOT(SL_IMU()));
	connect(ui.pB_Switch_LR, SIGNAL(clicked(bool)), this, SLOT(SL_Switch_LR()));

	//////////////////////////IMU capture
	IMUD1PreProcess.Init(&ceD1_IMUPort);
	connect(&ceD1_IMUPort, SIGNAL(readyRead()), &IMUD1PreProcess, SLOT(ceSlotPreProcessIMU()));

	foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
	{
		qDebug() << "Name:" << info.portName();
		qDebug() << "Description:" << info.description();
		qDebug() << "Manufacturer:" << info.manufacturer();

		//这里相当于自动识别串口号之后添加到了cmb，如果要手动选择可以用下面列表的方式添加进去
		QSerialPort serial;
		serial.setPort(info);

		serial.setBaudRate(460800);    	//波特率
		serial.setDataBits(QSerialPort::Data8);
		serial.setParity(QSerialPort::NoParity);
		serial.setStopBits(QSerialPort::OneStop);
		serial.setFlowControl(QSerialPort::NoFlowControl);

		if (serial.open(QIODevice::ReadWrite))
		{
			ui.cbB_uart->addItem(info.portName());
			serial.close();
		}
		else
		{
			qDebug() << "uart open failed!" << endl;
		}

	}

}

CoolEye::~CoolEye()
{
	CamShowing.stopImmediately();
	PreProcess.stopImmediately();
	camD1_L_capture.stopImmediately();
	camD1_R_capture.stopImmediately();

	CamShowing.wait();
	PreProcess.wait();
	camD1_L_capture.wait();
	camD1_R_capture.wait();


}

void CoolEye::SL_Caputre(void)
{
	if (ui.pB_Capture->text() == "Capture")
	{
		if (ce_cam_capture_init() == SUCCESS)   //  1 打开成功
		{
			CamShowing.start();
			PreProcess.start();

			camD1_L_capture.start();
			camD1_R_capture.start();

			ui.pB_Capture->setText("Stop");
			statusBar()->showMessage("Log: Capture image begin", 10000);
		}
		else
		{
			statusBar()->showMessage("Log: Open Camera failed!", 10000);
		}
	}
	else
	{
		ui.pB_Capture->setText("Busying");
		ui.pB_Capture->setDisabled(true);
		statusBar()->showMessage("Log: Capture Busying", 10000);

		CamShowing.stopImmediately();
		PreProcess.stopImmediately();

		camD1_L_capture.stopImmediately();
		camD1_R_capture.stopImmediately();

		CamShowing.wait();
		PreProcess.wait();
		camD1_L_capture.wait();
		camD1_R_capture.wait();

		ui.pB_Capture->setText("Capture");
		statusBar()->showMessage("Log: Capture Stop", 10000);
		ui.pB_Capture->setDisabled(false);
	}
}

void CoolEye::SL_IMU(void)
{
	if (ui.pB_Capture_2->text() == "IMU")
	{
		ceD1_IMUPort.setPortName(ui.cbB_uart->currentText());    //端口号
		if (ceD1_IMUPort.open(QIODevice::ReadWrite))   //  1 打开成功
		{
			ui.pB_Capture_2->setText("Stop");
			statusBar()->showMessage("Log: Capture IMU open sucess", 10000);

			ceD1_IMUPort.setBaudRate(460800);    	//波特率
			ceD1_IMUPort.setDataBits(QSerialPort::Data8);
			ceD1_IMUPort.setParity(QSerialPort::NoParity);
			ceD1_IMUPort.setStopBits(QSerialPort::OneStop);
			ceD1_IMUPort.setFlowControl(QSerialPort::NoFlowControl);
			ceD1_IMUPort.clearError();
			ceD1_IMUPort.clear();

			ceD1_IMUPort.setDataTerminalReady(true);
			const char ceIMUStartCMD[10] = {0x55,0xAA,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			ceD1_IMUPort.write(ceIMUStartCMD, 10);
			ceD1_IMUPort.waitForBytesWritten();
		}
		else
		{
			statusBar()->showMessage("Log: Open IMU failed!", 10000);
		}
	}
	else
	{
		ui.pB_Capture_2->setText("Busying");
		ui.pB_Capture_2->setDisabled(true);
		statusBar()->showMessage("Log: Capture IMU Busying", 10000);

		ceD1_IMUPort.close();

		ui.pB_Capture_2->setText("IMU");
		statusBar()->showMessage("Log: Capture IMU Stop", 10000);
		ui.pB_Capture_2->setDisabled(false);
	}

}

void CoolEye::SL_Switch_LR(void)
{
	int tx,ty,th,tw;

	tx = ui.label_image_caml->x();
	ty = ui.label_image_caml->y();
	th = ui.label_image_caml->height();
	tw = ui.label_image_caml->width();

	ui.label_image_caml->setGeometry(ui.label_image_camr->geometry());

	ui.label_image_camr->setGeometry(QRect(tx, ty, tw, th));

}

void CoolEye::ceSlotDisplayL(cv::Mat mat)
{
	cv::Mat rgb;
	QImage img;

	img = QImage((const uchar*)(mat.data), mat.cols, mat.rows, mat.cols*mat.channels(), QImage::Format_Indexed8);
	ui.label_image_caml->setPixmap(QPixmap::fromImage(img));
}

void CoolEye::ceSlotDisplayR(cv::Mat mat)
{
	cv::Mat rgb;
	QImage img;

	img = QImage((const uchar*)(mat.data), mat.cols, mat.rows, mat.cols*mat.channels(), QImage::Format_Indexed8);
	ui.label_image_camr->setPixmap(QPixmap::fromImage(img));
}

