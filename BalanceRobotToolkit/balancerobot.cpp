#include "balancerobot.h"
#include "ui_balancerobot.h"
#include <QMessageBox>
#include <QScrollBar>
#include <QStyle>
#include <QtWidgets>
#include <QPainter>

#define MAX_VALUE 50
qreal dy;

static const float PI          = 3.1415926536f;

BalanceRobot::BalanceRobot(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::BalanceRobot)
{
    ui->setupUi(this);
    serial = new QSerialPort(this);

    connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
    connect(ui->connectButton, SIGNAL(clicked()), this, SLOT(connectSerial()));
    connect(ui->checkEnable, SIGNAL(clicked()), this, SLOT(changBalance()));
    connect(ui->btnRead, SIGNAL(clicked()), this, SLOT(requestParams()));
    connect(ui->btnApply, SIGNAL(clicked()), this, SLOT(updateParams()));
    connect(ui->btnExport, SIGNAL(clicked()), this, SLOT(exportDataList()));
    connect(ui->btnSelectFile, SIGNAL(clicked()), this, SLOT(selectFile()));

    connect(ui->btnPlotInput, SIGNAL(clicked()), this, SLOT(togglePlot()));
    connect(ui->btnPlotOutput, SIGNAL(clicked()), this, SLOT(togglePlot()));
    connect(ui->btnPlotP, SIGNAL(clicked()), this, SLOT(togglePlot()));
    connect(ui->btnPlotI, SIGNAL(clicked()), this, SLOT(togglePlot()));
    connect(ui->btnPlotD, SIGNAL(clicked()), this, SLOT(togglePlot()));
    connect(ui->btnPlotV, SIGNAL(clicked()), this, SLOT(togglePlot()));
    connect(ui->btnClearBuf, SIGNAL(clicked()), this, SLOT(clearBuf()));
    connect(ui->param_p, SIGNAL(editingFinished()), this, SLOT(updateParams()));
    connect(ui->param_i, SIGNAL(editingFinished()), this, SLOT(updateParams()));
    connect(ui->param_d, SIGNAL(editingFinished()), this, SLOT(updateParams()));
    connect(ui->param_posScaler, SIGNAL(editingFinished()), this, SLOT(updateParams()));
    connect(ui->param_speedScaler, SIGNAL(editingFinished()), this, SLOT(updateParams()));



    ui->baudBox->addItem(QStringLiteral("9600"),QSerialPort::Baud9600);
    ui->baudBox->addItem(QStringLiteral("115200"),QSerialPort::Baud115200);
    enumSerial();
    scene = new QGraphicsScene(0, 0, this->ui->pidGraph->size().width() - 2,
                                     this->ui->pidGraph->size().height() - 2);

    vecscene = new QGraphicsScene(0, 0, this->ui->vectroGraph->size().width() - 2,
                                     this->ui->vectroGraph->size().height() - 2);
    this->ui->pidGraph->setScene(scene);
    this->ui->vectroGraph->setScene(vecscene);
    drawVectorAxis();
    dy = scene->height()/(MAX_VALUE*2); //draw every 1 deg
    timeLen = scene->width();
    plotInput=plotOutput=plotP=plotI=plotD=plotV=true;
}

void BalanceRobot::clearBuf()
{
    ui->txt_bufcnt->setText(QString::number(0));
    dataList.clear();
}

BalanceRobot::~BalanceRobot()
{
    delete ui;
}

void BalanceRobot::togglePlot()
{
    QPushButton *button = (QPushButton *)sender();
    bool * toPlot;
    if(button==ui->btnPlotInput){
        toPlot = &plotInput;
    }else if(button==ui->btnPlotOutput){
        toPlot = &plotOutput;
    }else if(button==ui->btnPlotP){
        toPlot = &plotP;
    }else if(button==ui->btnPlotI){
        toPlot = &plotI;
    }else if(button==ui->btnPlotD){
        toPlot = &plotD;
    }else if(button==ui->btnPlotV){
        toPlot = &plotV;
    }
    if(*toPlot){
        *toPlot=false;
        button->setStyleSheet("");
    }else{
        *toPlot=true;
        if(button==ui->btnPlotInput){
            button->setStyleSheet("color:red");
        }else if(button==ui->btnPlotOutput){
            button->setStyleSheet("color:blue");
        }else if(button==ui->btnPlotP){
            button->setStyleSheet("color:green");
        }else if(button==ui->btnPlotI){
            button->setStyleSheet("color:rgb(170, 170, 0)");
        }else if(button==ui->btnPlotD){
            button->setStyleSheet("color:magenta");
        }else if(button==ui->btnPlotV){
            button->setStyleSheet("color:rgb(255, 170, 0)");
        }
    }
}

void BalanceRobot::enumSerial()
{
    ui->serialBox->clear();
    static const QString blankString = QObject::tr("N/A");
    QString description;

    QString manufacturer;
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        QStringList list;
        description = info.description();
        manufacturer = info.manufacturer();
        list << info.portName()
             << (!description.isEmpty() ? description : blankString)
             << (!manufacturer.isEmpty() ? manufacturer : blankString)
             << info.systemLocation()
             << (info.vendorIdentifier() ? QString::number(info.vendorIdentifier(), 16) : blankString)
             << (info.productIdentifier() ? QString::number(info.productIdentifier(), 16) : blankString);

        ui->serialBox->addItem(list.first(), list);
    }
}

void BalanceRobot::connectSerial()
{
    QString name = ui->serialBox->currentText();
    qint32 baud = static_cast<QSerialPort::BaudRate>(ui->baudBox->itemData(ui->baudBox->currentIndex()).toInt());
    serial->setPortName(name);
#if 1
    if (serial->open(QIODevice::ReadWrite)) {
        if (serial->setBaudRate(baud)
                && serial->setDataBits(QSerialPort::Data8)
                && serial->setParity(QSerialPort::NoParity)
                && serial->setStopBits(QSerialPort::OneStop)
                && serial->setFlowControl(QSerialPort::NoFlowControl)) {
            //serial->flush();
            //serial->clear();
            //serial->setReadBufferSize(10240);
            ui->statusBar->showMessage(tr("Connected to %1").arg(name));
            ui->connectButton->setText(tr("Disconnect"));
            ui->connectButton->setCheckable(true);
            connect(ui->connectButton, SIGNAL(clicked()), this, SLOT(closeActiveSerial()));
            disconnect(ui->connectButton, SIGNAL(clicked()), this, SLOT(connectSerial()));
            QTimer::singleShot(500, this, SLOT(requestParams()));
            QTimer::singleShot(1000, this, SLOT(showDebug()));
        } else {
            serial->close();
            QMessageBox::critical(this, tr("Error"), serial->errorString());
            ui->statusBar->showMessage(tr("Open error"));
        }
    } else {
        QMessageBox::critical(this, tr("Error"), serial->errorString());
        ui->statusBar->showMessage(tr("Configure error"));
    }
#else
    // this one works, but don't know if safe
    serial->setBaudRate(baud);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    if (serial->open(QIODevice::ReadWrite)) {
        ui->statusBar->showMessage(tr("Connected to %1").arg(name));
        ui->connectButton->setText(tr("Disconnect"));
        ui->connectButton->setCheckable(true);
        connect(ui->connectButton, SIGNAL(clicked()), this, SLOT(closeActiveSerial()));
        disconnect(ui->connectButton, SIGNAL(clicked()), this, SLOT(connectSerial()));
        QTimer::singleShot(500, this, SLOT(requestParams()));
    } else {
        QMessageBox::critical(this, tr("Error"), serial->errorString());
        ui->statusBar->showMessage(tr("Configure error"));
    }
#endif
}

void BalanceRobot::closeActiveSerial()
{
    serial->flush();
    serial->close();
    ui->connectButton->setText(tr("Connect"));
    ui->connectButton->setCheckable(false);

    disconnect(ui->connectButton, SIGNAL(clicked()), this, SLOT(closeActiveSerial()));
    connect(ui->connectButton, SIGNAL(clicked()), this, SLOT(connectSerial()));
    ui->statusBar->showMessage(tr("Disconnect to %1").arg(serial->portName()));
}

void BalanceRobot::readData()
{
    while(serial->canReadLine()){
        QByteArray data = serial->readLine(128);
        //QByteArray data = serial->readAll();
        parseCmdLine(QString(data));
    }
}

void BalanceRobot::writeData(const QByteArray &data)
{
    serial->write(data);
}

void BalanceRobot::parseCmdLine(QString cmd)
{
    QStringList parts = cmd.split(":");
    if(parts[0].contains("dbg")){
        QStringList tmp = parts[1].split(",");
        if(tmp.length()!=6) return;
        float param0 = tmp[0].toFloat();
        float param1 = tmp[1].toFloat();
        float param2 = tmp[2].toFloat();
        float param3 = tmp[3].toFloat();
        float param4 = tmp[4].toFloat();
        float param5 = tmp[5].toFloat();
        this->updateGraph(param0,param1,param2,param3,param4,param5);
    }else if(parts[0].contains("pid")){
        QStringList tmp = parts[1].split(",");
        p = tmp[0].toFloat();
        i = tmp[1].toFloat();
        d = tmp[2].toFloat();
        ui->param_p->setText(QString::number(p));
        ui->param_i->setText(QString::number(i));
        ui->param_d->setText(QString::number(d));
    }else if(parts[0].contains("motor")){
        QStringList tmp = parts[1].split(",");
        motor_min = tmp[0].toInt();
        motor_max = tmp[1].toInt();
        ui->param_min->setText(QString::number(motor_min));
        ui->param_max->setText(QString::number(motor_max));
    }else if(parts[0].contains("time")){
        QStringList tmp = parts[1].split(",");
        sample_time = tmp[0].toFloat();
        accele_time = tmp[1].toFloat();
        ui->sample_time->setText(QString::number(sample_time));
        ui->accele_time->setText(QString::number(accele_time));
    }else if(parts[0].contains("scaler")){
        QStringList tmp = parts[1].split(",");
        position_scaler = tmp[0].toFloat();
        speed_scaler = tmp[1].toFloat();
        ui->param_posScaler->setText(QString::number(position_scaler));
        ui->param_speedScaler->setText(QString::number(speed_scaler));
    }else if(parts[0].contains("en")){
        QStringList tmp = parts[1].split(",");
        enableBalance = tmp[1].toInt();
        ui->checkEnable->setChecked(enableBalance);
        if(enableBalance){
            this->clearBuf();
        }
    }else if(parts[0].contains("angle")){
        QStringList tmp = parts[1].split(",");
        falling_angle = tmp[0].toFloat();
        balance_angle = tmp[1].toFloat();
        accele_angle = tmp[2].toFloat();
        ui->falling_angle->setText(QString::number(falling_angle));
        ui->balance_angle->setText(QString::number(balance_angle));
        ui->accele_angle->setText(QString::number(accele_angle));
    }else if(parts[0].contains("deadband")){
        QStringList tmp = parts[1].split(",");
        deadL = tmp[0].toFloat();
        deadR = tmp[1].toFloat();
        ui->deadL->setText(QString::number(deadL));
        ui->deadR->setText(QString::number(deadR));
    }else{
        qDebug()<<"In:"+cmd;
    }
}

void BalanceRobot::requestParams()
{
    QString str = "a\nb\nc\nd\ng\nf\nh\n";
    if(serial->isOpen())
        serial->write(str.toLocal8Bit());
}

void BalanceRobot::showDebug()
{
    QString str = QString("D:%1,%2\n").arg(1).arg(0);
    if(serial->isOpen())
        serial->write(str.toLocal8Bit());
}

void BalanceRobot::updateParams()
{
    p = ui->param_p->text().toFloat();
    i = ui->param_i->text().toFloat();
    d = ui->param_d->text().toFloat();
    motor_min = ui->param_min->text().toInt();
    motor_max = ui->param_max->text().toInt();
    sample_time = ui->sample_time->text().toInt();
    accele_time = ui->accele_time->text().toInt();
    position_scaler = ui->param_posScaler->text().toFloat();
    speed_scaler = ui->param_speedScaler->text().toFloat();
    deadL = ui->deadL->text().toInt();
    deadR = ui->deadR->text().toInt();
    falling_angle = ui->falling_angle->text().toFloat();
    balance_angle = ui->balance_angle->text().toFloat();
    accele_angle = ui->accele_angle->text().toFloat();

    QString str = QString("A:%1,%2,%3\n").arg(p,0,'f',2).arg(i,0,'f',2).arg(d,0,'f',2);
    if(serial->isOpen())
        serial->write(str.toLocal8Bit());

    str = QString("B:%1,%2\n").arg(motor_min).arg(motor_max);
    if(serial->isOpen())
        serial->write(str.toLocal8Bit());

    str = QString("C:%1,%2\n").arg(sample_time).arg(accele_time);
    if(serial->isOpen())
        serial->write(str.toLocal8Bit());
    str = QString("F:%1,%2,%3\n").arg(falling_angle).arg(balance_angle).arg(accele_angle);
    if(serial->isOpen())
        serial->write(str.toLocal8Bit());

    str = QString("G:%1,%2\n").arg(position_scaler).arg(speed_scaler);
    if(serial->isOpen())
        serial->write(str.toLocal8Bit());

    str = QString("H:%1,%2\n").arg(deadL).arg(deadR);
    if(serial->isOpen())
        serial->write(str.toLocal8Bit());
}

void BalanceRobot::clearGraph()
{
    timeAxis = 0;
    scene->clear();
}

void BalanceRobot::drawBasicElement()
{


}

void BalanceRobot::drawVectorAxis()
{
    vecscene->clear();
    vecscene->addLine(vecscene->width()/2, 0, vecscene->width()/2, vecscene->height());
    vecscene->addLine(0, vecscene->height()/2, vecscene->width(), vecscene->height()/2);
    vecPoint = vecscene->addEllipse(150-2.5,
                                 150-2.5,
                                 5,
                                 5,
                                 QPen(Qt::red), QBrush(Qt::red));
    //vecPoint->setPos(-100,-100);
    ui->pidGraph->show();
}

void BalanceRobot::updateVectorPos(float vx, float vy)
{
    vecPoint->setPos(vx,-vy);
    ui->pidGraph->show();
}

void BalanceRobot::changBalance()
{
    enableBalance = ui->checkEnable->isChecked();
    QString str = QString("D:%1,%2\n").arg(1).arg(enableBalance);
    if(serial->isOpen())
        serial->write(str.toLocal8Bit());
}

void BalanceRobot::updateGraph(float param0, float param1, float param2,float param3,float param4, float param5)
{
    if(plotInput)
    scene->addEllipse(timeAxis,
                      scene->height()/2-dy*param0,
                      2,
                      2,
                      QPen(Qt::red), QBrush(Qt::SolidPattern));
    inputList<<param0;
    if(plotOutput)
    scene->addEllipse(timeAxis,
                      scene->height()/2-dy*param1,
                      2,
                      2,
                      QPen(Qt::blue), QBrush(Qt::SolidPattern));
    if(plotP)
    scene->addEllipse(timeAxis,
                      scene->height()/2-dy*param2,
                      2,
                      2,
                      QPen(Qt::green), QBrush(Qt::SolidPattern));
    if(plotI)
    scene->addEllipse(timeAxis,
                      scene->height()/2-dy*param3,
                      2,
                      2,
                      QPen(Qt::darkYellow), QBrush(Qt::SolidPattern));
    if(plotD)
    scene->addEllipse(timeAxis,
                      scene->height()/2-dy*param4,
                      2,
                      2,
                      QPen(Qt::magenta), QBrush(Qt::SolidPattern));
    if(plotV)
    scene->addEllipse(timeAxis,
                      scene->height()/2-dy*param5,
                      2,
                      2,
                      QPen(QColor(255, 170, 0, 255)), QBrush(Qt::SolidPattern));

    dataList<<QString("%1,%2,%3,%4,%5,%6\n").arg(param0).arg(param1).arg(param2).arg(param3).arg(param4).arg(param5);
    ui->txt_bufcnt->setText(QString::number(dataList.length()));
    updateVectorPos(param0*100,param1*100);
    timeAxis+=1;
    if(timeAxis>timeLen){
        timeAxis = 0;
        clearGraph();
        if(ui->pidCheckBox->isChecked()){
            calcZieglerPID();
        }
        //dataList.clear();
        inputList.clear();
    }
}

void BalanceRobot::calcZieglerPID()
{
    QList<int> cycles;
    int deltaTime,delatTimeSum=0;
    int len = inputList.length();
    for(int i=0;i<len-1;i++){
        float a0 = inputList.at(i);
        float a1 = inputList.at(i+1);
        if(a0<=0 && a1>0){
            cycles<<i;
        }
    }
    if(cycles.length()<5) return;
    for(int i=1;i<cycles.length();i++){
        deltaTime = cycles.at(i)-cycles.at(i-1);
        delatTimeSum+=deltaTime;
    }
    float Tu = (float)delatTimeSum/(cycles.length()-1)*sample_time/1000;
    float Ku = p;
    QGraphicsTextItem * io = new QGraphicsTextItem;
    io->setPos(10,10);
    //Kp = 0.7*Ku
    //Ki = 0.4*Kp/Tu
    //Kd = 0.15*Kp*Tu
    float Kp = 0.7*Ku;
    float Ki = 0.4*Kp/Tu;
    float Kd = 0.08*Kp*Tu; //0.15->0.08
    io->setPlainText(QString("Ku=%1\nTu=%2s\nP=0.7*Ku=%3\nI=0.4*Kp/Tu=%4\nD=Kd=0.08*Kp*Tu=%5\n").arg(Ku).arg(Tu).arg(Kp).arg(Ki).arg(Kd));
    scene->addItem(io);
}

void BalanceRobot::selectFile()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"",tr("Files (*.txt)"));
    ui->filePathEdit->setText(fileName);
}

void BalanceRobot::exportDataList()
{
    QString fileName = ui->filePathEdit->text();
    QFile file(fileName);
    if(!file.open(QIODevice::WriteOnly)) {
        QMessageBox::information(0, "open file fail", file.errorString());
    }
    QTextStream out(&file);
    for(int i=0;i<dataList.length();i++){
        out<<dataList.at(i);
    }
    file.close();
}
