#ifndef BALANCEROBOT_H
#define BALANCEROBOT_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QGraphicsItem>
#include <QThread>
#include <QList>

namespace Ui {
class BalanceRobot;
}

class BalanceRobot : public QMainWindow
{
    Q_OBJECT

public:
    explicit BalanceRobot(QWidget *parent = 0);
    ~BalanceRobot();

private:
    bool plotInput, plotOutput, plotP, plotI, plotD, plotV;
    float p;
    float i;
    float d;
    int motor_min;
    int motor_max;
    int sample_time;
    int accele_time;
    float accele_angle;
    float position_scaler;
    float speed_scaler;
    float falling_angle;
    float balance_angle;
    float deadL,deadR;

    int enableBalance;
    qreal timeAxis;
    qreal timeLen;
    QStringList dataList;
    QList<float> inputList;

    Ui::BalanceRobot *ui;
    QSerialPort *serial;
    void parseCmdLine(QString cmd);
    QGraphicsScene *scene;
    QGraphicsScene *vecscene;
    QGraphicsEllipseItem * vecPoint;

public slots:
    void enumSerial();
    void connectSerial();
    void closeActiveSerial();
    void readData();
    void writeData(const QByteArray &data);

    void requestParams();
    void updateParams();
    void showDebug();
    void selectFile();
    void exportDataList();
    void calcZieglerPID();
    void togglePlot();

    void clearGraph();
    void clearBuf();
    void drawBasicElement();
    void updateGraph(float setpoint, float input, float output, float param3, float param4, float param5);
    void changBalance();

    void drawVectorAxis();
    void updateVectorPos(float vx, float vy);
};

#endif // BALANCEROBOT_H
