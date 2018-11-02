#include "CoolEye.h"
#include "cedriver_cam.h"
#include <QtWidgets/QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include "libusb.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	CoolEye main_window;
	main_window.show();

    MySetThreadAffinityMask(MAIN_THREAD_CPUID);

	return app.exec();
}
