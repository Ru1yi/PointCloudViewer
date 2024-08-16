#include "Lidar_test.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Lidar_test w;
    w.show();
    return a.exec();
}
