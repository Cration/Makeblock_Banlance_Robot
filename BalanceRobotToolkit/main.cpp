#include "balancerobot.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    BalanceRobot w;
    w.show();

    return a.exec();
}
