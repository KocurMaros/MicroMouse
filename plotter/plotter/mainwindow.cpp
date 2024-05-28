#include "mainwindow.h"
#include <QtCharts/QValueAxis>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QCategoryAxis>
#include <QRandomGenerator>
#include <QVBoxLayout>
#include <QWidget>
#include <iostream>
#include <stdio.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    motorChart(new QChart()),
    motorSeriesA(new QLineSeries()),
    motorSeriesB(new QLineSeries()),
    tofSeries(new QBarSeries()),
    tofChart(new QBarSet("BarSet")),
    gyroChart(new QPolarChart()),
    gyroSeries(new QLineSeries()),
    timer(new QTimer(this)),
    x(0)
{
    setWindowTitle("Real-Time Plot");
    //initalize arrays to 0
    timestampArray.reserve(100);
    motorArrayA.reserve(100);
    motorArrayB.reserve(100);
    gyroZArray.reserve(100);


    // Set up UDP server
    localIP = "192.168.137.1";
    localPort = 3333;
    bufferSize = 1024;
    udpSocket = new QUdpSocket(this);
    udpSocket->bind(QHostAddress(localIP), localPort);
    connect(udpSocket, &QUdpSocket::readyRead, this, &MainWindow::readPendingDatagrams);

    // Setup line motorChart
    motorChart->legend()->hide();
    motorChart->addSeries(motorSeriesA);
    motorChart->addSeries(motorSeriesB);
    motorChart->createDefaultAxes();

    auto *axisX = new QValueAxis;
    axisX->setRange(0, 100);
    axisX->setLabelFormat("%i");
    motorChart->setAxisX(axisX, motorSeriesA);
    motorChart->setAxisX(axisX, motorSeriesB);

    auto *axisY = new QValueAxis;
    axisY->setRange(-50, 50);
    axisY->setLabelFormat("%i");
    motorChart->setAxisY(axisY, motorSeriesA);
    motorChart->setAxisY(axisY, motorSeriesB);
    motorChart->setTitle("Motor 1 and 2");

    motorSeriesA->setName("Tof 1 Series");
    motorSeriesA->setColor(Qt::blue);

    motorSeriesB->setName("Tof 2 Series");
    motorSeriesB->setColor(Qt::red);

    // Setup bar chart
    tofChart->append({0, 0, 0, 0}); // Initialize with 4 values
    tofSeries->append(tofChart);

    QChart *tofChart = new QChart();
    tofChart->addSeries(tofSeries);
    tofChart->legend()->hide();
    tofChart->setTitle("TOF 1, 2, 3, 4");

    QStringList categories;
    categories << "1" << "2" << "3" << "4";
    QBarCategoryAxis *axisXBar = new QBarCategoryAxis();
    axisXBar->append(categories);
    tofChart->addAxis(axisXBar, Qt::AlignBottom);
    tofSeries->attachAxis(axisXBar);

    auto *axisYBar = new QValueAxis;
    axisYBar->setRange(0, 100);
    tofChart->addAxis(axisYBar, Qt::AlignLeft);
    tofSeries->attachAxis(axisYBar);

    // Setup compass chart
    gyroChart->addSeries(gyroSeries);

    QValueAxis *angularAxis = new QValueAxis();
    angularAxis->setTickCount(9);
    angularAxis->setLabelFormat("%.0f");
    angularAxis->setRange(0, 360);
    gyroChart->addAxis(angularAxis, QPolarChart::PolarOrientationAngular);
    gyroSeries->attachAxis(angularAxis);

    QValueAxis *radialAxis = new QValueAxis();
    radialAxis->setLabelFormat("%d");
    radialAxis->setRange(0, 100);
    radialAxis->setVisible(false);
    gyroChart->addAxis(radialAxis, QPolarChart::PolarOrientationRadial);
    gyroSeries->attachAxis(radialAxis);
    gyroChart->legend()->hide();
    gyroChart->setTitle("Gyro, Z axis");

    // Configure layout
    plotMotor = new QChartView(motorChart);
    plotMotor->setRenderHint(QPainter::Antialiasing);

    plotTof = new QChartView(tofChart);
    plotTof->setRenderHint(QPainter::Antialiasing);

    plotGyro = new QChartView(gyroChart);
    plotGyro->setRenderHint(QPainter::Antialiasing);

    centralWidget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(centralWidget);
    layout->addWidget(plotMotor);
    layout->addWidget(plotTof);
    layout->addWidget(plotGyro);

    setCentralWidget(centralWidget);

    connect(timer, &QTimer::timeout, this, &MainWindow::updateChart);
    timer->start(50);  // Update every 100 ms
}

MainWindow::~MainWindow()
{
}

void MainWindow::readPendingDatagrams() {
        while (udpSocket->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(udpSocket->pendingDatagramSize());
            QHostAddress sender;
            quint16 senderPort;

            udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

            QList<QByteArray> values = datagram.split(',');

            double timestamp = values[0].toDouble();
            double tofY_1 = values[1].toDouble();
            double tofY_2 = values[2].toDouble();
            double tofY_3 = values[3].toDouble();
            double tofY_4 = values[4].toDouble();
            double gyroZ = values[5].toDouble();
            double motorA = values[6].toDouble();
            double motorB = values[7].toDouble();
            {
            std::scoped_lock lock(mut);
            timestampArray.append(timestamp);
            motorArrayA.append(motorA);
            motorArrayB.append(motorB);
            tofArray.append(QVector<double>({tofY_1, tofY_2, tofY_3, tofY_4}));
            gyroZArray.append(gyroZ);
            }
        }
    }

void MainWindow::updateChart()
{
    if(!tofArray.isEmpty()) {
        std::scoped_lock lock(mut);
        motorSeriesA->append(timestampArray.back(), motorArrayA.back());
        motorSeriesB->append(timestampArray.back(), motorArrayB.back());
        motorChart->axes(Qt::Horizontal).first()->setRange(qMax<qreal>(0, timestampArray.back() - 100), timestampArray.back());

        // Update bar motorChart
            for (int i = 0; i < tofChart->count(); ++i) {
                tofChart->replace(i, tofArray.back()[i]);
            }

        // Update compass motorChart
        gyroSeries->clear();
        gyroSeries->append(gyroZArray.back(), 100);  // Point on the perimeter of the compass
        gyroSeries->append(gyroZArray.back(), 0);    // Center of the compass

        if (motorSeriesA->count() > 100) {
            motorSeriesA->remove(0);
            motorSeriesB->remove(0);
        }
    }
}
