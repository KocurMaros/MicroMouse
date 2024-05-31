#include "mainwindow.h"
#include <QtCharts/QValueAxis>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarSeries>
#include <QtCharts/QCategoryAxis>
#include <QRandomGenerator>
#include <QVBoxLayout>
#include <QWidget>
#include <iostream>
#include <qglobal.h>
#include <stdio.h>

#define BUFFER_SIZE 1024
#define MOTOR_AXIS_LIMIT 600
#define BATTERY_VOLTAGE_MAX 4200
#define BATTERY_VOLTAGE_MIN 3300

#define CONNECT_MESSAGE QString("micromouse")

#define BATTERY_PERCENTAGE(voltage) (voltage - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) *100
#define BATTERY_TEXT(voltage) "Battery: " + QString::number(BATTERY_PERCENTAGE(voltage)) + "%"

#define GYRO_TEXT(frequency) "Gyro frequency: " + QString::number(frequency) + "Hz"

#define POS_X_TXT(x) "X: " + QString::number(x)
#define POS_Y_TXT(y) "Y: " + QString::number(y)

#ifdef WIN32
#define M_PI 3.14159
#endif


#define DEG2RAD(angle) (angle * M_PI / 180)
#define RAD2DEG(angle) (angle * 180 / M_PI)


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, motorChart(new QChart())
	, motorSeriesA(new QLineSeries())
	, motorSeriesB(new QLineSeries())
	, controlSignalChart(new QChart())
	, controlSeries(new QLineSeries())
	, axisXBar(new QBarCategoryAxis())
	, encoderTicksChart(new QChart())
	, leftEncoderSeries(new QLineSeries())
	, rightEncoderSeries(new QLineSeries())
	, tofSeries(new QBarSeries())
	, tofChart(new QBarSet("BarSet"))
	, gyroChart(new QPolarChart())
	, gyroSeries(new QLineSeries())
	, timer(new QTimer(this))
	, x(0)
	, batteryVoltage(0)
	, gyroFrequency(0)
	, motorA(0)
	, motorB(0)
    , motorAMax(0)
    , motorAMin(0)
    , motorBMax(0)
    , motorBMin(0)
    , controlMax(0)
    , controlMin(0)
    , encoderLeftMax(0)
    , encoderLeftMin(0)
    , encoderRightMax(0)
    , encoderRightMin(0)
	, motorBufferSize(0)
    , motorLowerLimit(0)
    , motorUpperLimit(250)
    , controlLowerLimit(-300)
    , controlUpperLimit(300)
    , encoderLowerLimit(-1000)
    , encoderUpperLimit(1000)
{
	setWindowTitle("Real-Time Plot");
	//initalize arrays to 0
	gyroZArray.reserve(100);


	// Set up UDP server
	localIP = "10.42.0.1";
	localPort = 3333;
	udpSocket = new QUdpSocket(this);
	udpSocket->bind(QHostAddress(localIP), localPort);
	connect(udpSocket, &QUdpSocket::readyRead, this, &MainWindow::readPendingDatagrams);

	{
		motorSeriesA->setName("Tof 1 Series");
		motorSeriesA->setColor(Qt::blue);

		motorSeriesB->setName("Tof 2 Series");
		motorSeriesB->setColor(Qt::red);

		// Setup line motorChart
		motorChart->legend()->hide();
		motorChart->addSeries(motorSeriesA);
		motorChart->addSeries(motorSeriesB);
		motorChart->createDefaultAxes();

		auto *axisX = new QValueAxis;
		axisX->setRange(0, MOTOR_AXIS_LIMIT);
		axisX->setLabelFormat("%i");
		motorChart->setAxisX(axisX, motorSeriesA);
		motorChart->setAxisX(axisX, motorSeriesB);

		auto *axisY = new QValueAxis;
        axisY->setRange(motorLowerLimit, motorUpperLimit);
		axisY->setLabelFormat("%i");
		motorChart->setAxisY(axisY, motorSeriesA);
		motorChart->setAxisY(axisY, motorSeriesB);
		motorChart->setTitle("Motor 1 and 2");

		plotMotor = new QChartView(motorChart);
		plotMotor->setRenderHint(QPainter::Antialiasing);
	}

	{
		controlSignalChart->legend()->hide();
		controlSignalChart->addSeries(controlSeries);
		controlSignalChart->createDefaultAxes();

		auto *axisX = new QValueAxis;
		axisX->setRange(0, MOTOR_AXIS_LIMIT);
		axisX->setLabelFormat("%i");
		controlSignalChart->setAxisX(axisX, controlSeries);

		auto *axisY = new QValueAxis;
        axisY->setRange(controlLowerLimit, controlUpperLimit);
		axisY->setLabelFormat("%i");
		controlSignalChart->setAxisY(axisY, controlSeries);
		controlSignalChart->setTitle("Control signal");

		plotControl = new QChartView(controlSignalChart);
		plotControl->setRenderHint(QPainter::Antialiasing);
	}

	{
		encoderTicksChart->legend()->hide();
		encoderTicksChart->addSeries(leftEncoderSeries);
		encoderTicksChart->addSeries(rightEncoderSeries);
		encoderTicksChart->createDefaultAxes();

		auto *axisX = new QValueAxis;
		axisX->setRange(0, MOTOR_AXIS_LIMIT);
		axisX->setLabelFormat("%i");
		encoderTicksChart->setAxisX(axisX, leftEncoderSeries);
		encoderTicksChart->setAxisX(axisX, rightEncoderSeries);

		auto *axisY = new QValueAxis;
        axisY->setRange(encoderLowerLimit, encoderUpperLimit);
		axisY->setLabelFormat("%i");
		encoderTicksChart->setAxisY(axisY, leftEncoderSeries);
		encoderTicksChart->setAxisY(axisY, rightEncoderSeries);
		encoderTicksChart->setTitle("Encoder ticks");

		plotEncoderTicks = new QChartView(encoderTicksChart);
		plotEncoderTicks->setRenderHint(QPainter::Antialiasing);
	}

	batteryLineEdit = new QLineEdit(this);
	batteryLineEdit->setText(BATTERY_TEXT(0));

	gyroFrequencyLineEdit = new QLineEdit(this);
	gyroFrequencyLineEdit->setText(GYRO_TEXT(0));

	// Setup bar chart
	tofChart->append({ 0, 0, 0, 0 }); // Initialize with 4 values
	tofSeries->append(tofChart);
	tofSeries->setLabelsAngle(20);


	QChart *tofChart = new QChart();
	tofChart->addSeries(tofSeries);
	tofChart->legend()->hide();
	tofChart->setTitle("TOF 1, 2, 3, 4");

	categories << "1"
			   << "2"
			   << "3"
			   << "4";
	axisXBar->append(categories);
	tofChart->addAxis(axisXBar, Qt::AlignBottom);
	tofSeries->attachAxis(axisXBar);

	auto *axisYBar = new QValueAxis();
	axisYBar->setRange(0, 40);
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

	plotTof = new QChartView(tofChart);
	plotTof->setRenderHint(QPainter::Antialiasing);

	plotGyro = new QChartView(gyroChart);
	plotGyro->setRenderHint(QPainter::Antialiasing);

	batteryLineEdit = new QLineEdit(this);
	batteryLineEdit->setText(BATTERY_TEXT(0));
	batteryLineEdit->setReadOnly(true);

	gyroFrequencyLineEdit = new QLineEdit(this);
	gyroFrequencyLineEdit->setText(GYRO_TEXT(0));
	gyroFrequencyLineEdit->setReadOnly(true);

	posXLineEdit = new QLineEdit(this);
	posXLineEdit->setText(POS_X_TXT(0));
	posXLineEdit->setReadOnly(true);

	posYLineEdit = new QLineEdit(this);
	posYLineEdit->setText(POS_Y_TXT(0));
	posYLineEdit->setReadOnly(true);

	clearMotorChartButton = new QPushButton("Clear Motor Chart");
	connect(clearMotorChartButton, &QPushButton::clicked, [this]() {
		motorSeriesA->clear();
		motorSeriesB->clear();
		leftEncoderSeries->clear();
		rightEncoderSeries->clear();
		motorBufferSize = 0;
		motorBufferSizeLast = 0;
	});

	pauseButton = new QPushButton("Pause");
	centralWidget = new QWidget(this);
	QVBoxLayout *layout = new QVBoxLayout(centralWidget);
	layout->addWidget(plotMotor);
	layout->addWidget(plotControl);
	layout->addWidget(plotTof);

	QBoxLayout *hLayout = new QHBoxLayout();
	hLayout->addWidget(plotGyro);
	hLayout->addWidget(plotEncoderTicks);

	layout->addLayout(hLayout);
	layout->addWidget(batteryLineEdit);
	layout->addWidget(gyroFrequencyLineEdit);
	layout->addWidget(posXLineEdit);
	layout->addWidget(posYLineEdit);
	layout->addWidget(clearMotorChartButton);
	layout->addWidget(pauseButton);

	setCentralWidget(centralWidget);

	connect(timer, &QTimer::timeout, this, &MainWindow::updateChart);
	timer->start(50); // Update every 100 ms

	connect(pauseButton, &QPushButton::clicked, [this]() {
		if (timer->isActive()) {
			timer->stop();
			pauseButton->setText("Continue");
			std::cout << "Continued" << std::endl;
		}
		else {
			timer->start();
			pauseButton->setText("Pause");
			std::cout << "Paused" << std::endl;
		}
	});

	disconnectedTimer = new QTimer(this);
	disconnectedTimer->setSingleShot(true);

	connect(disconnectedTimer, &QTimer::timeout, [this]() {
		setWindowTitle("Disconnected");
		disconnectedTimer->stop();
	});

	QSize winSize(1000,800);
	this->setMinimumSize(winSize);
}

MainWindow::~MainWindow()
{
	motorChart->deleteLater();
	motorSeriesA->deleteLater();
	leftEncoderSeries->deleteLater();
	rightEncoderSeries->deleteLater();
	motorSeriesB->deleteLater();
	tofSeries->deleteLater();
	tofChart->deleteLater();
	gyroChart->deleteLater();
	gyroSeries->deleteLater();
	clearMotorChartButton->deleteLater();
	encoderTicksChart->deleteLater();
}

void MainWindow::readPendingDatagrams()
{
	while (udpSocket->hasPendingDatagrams()) {
		QByteArray datagram;
		datagram.resize(udpSocket->pendingDatagramSize());
		QHostAddress sender;
		quint16 senderPort;

		udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

		if (datagram == CONNECT_MESSAGE) {
			setWindowTitle("Connected");
			motorSeriesA->clear();
			motorSeriesB->clear();
			controlSeries->clear();
			leftEncoderSeries->clear();
			rightEncoderSeries->clear();
			motorBufferSize = 0;
			motorBufferSizeLast = 0;
			continue;
		}

		QList<QByteArray> values = datagram.split(',');

		double tofY_1 = values[1].toDouble();
		double tofY_2 = values[2].toDouble();
		double tofY_3 = values[3].toDouble();
		double tofY_4 = values[4].toDouble();
		double gyroZ = values[5].toDouble();
		motorBufferSizeLast.store(motorBufferSize.load());
		motorBufferSize++;

		{
			std::scoped_lock lock(mut);
			timestamp = values[0].toDouble() / 1'000'000;
			motorA = values[6].toDouble();
			motorB = values[7].toDouble();
			tofArray.append(QVector<double>({ tofY_1, tofY_2, tofY_3, tofY_4 }));
			gyroZArray.append(gyroZ);
			batteryVoltage = values[8].toDouble();
			batteryLineEdit->setText(BATTERY_TEXT(batteryVoltage));
			gyroFrequency = values[9].toDouble();
			gyroFrequencyLineEdit->setText(GYRO_TEXT(gyroFrequency));
			posXLineEdit->setText(POS_X_TXT(values[10].toDouble()));
			posXLineEdit->setText(POS_X_TXT(values[11].toDouble()));
			control = values[12].toDouble();
			leftenc = values[13].toDouble();
			rightenc = values[14].toDouble();
		}
	}
	disconnectedTimer->start(2000);
}

void MainWindow::updateChart()
{
	if (tofArray.isEmpty()) {
		return;
	}

	if (motorBufferSize == motorBufferSizeLast) {
		return;
	}

	std::scoped_lock lock(mut);

	motorSeriesA->append(timestamp, motorA);
	motorSeriesB->append(timestamp, motorB);
	leftEncoderSeries->append(timestamp, leftenc);
	rightEncoderSeries->append(timestamp, rightenc);

	controlSeries->append(timestamp, control);
	auto start = motorSeriesA->at(0).x();
	motorChart->axes(Qt::Horizontal).first()->setRange(start, timestamp);
	controlSignalChart->axes(Qt::Horizontal).first()->setRange(start, timestamp);
	encoderTicksChart->axes(Qt::Horizontal).first()->setRange(start, timestamp);

    findMinMax(&motorAMin, &motorAMax, motorLowerLimit, motorUpperLimit, motorSeriesA);
    findMinMax(&motorBMin, &motorBMax, motorLowerLimit, motorUpperLimit, motorSeriesB);
    findMinMax(&controlMin, &controlMax, controlLowerLimit, controlUpperLimit, controlSeries);
    findMinMax(&encoderLeftMin, &encoderLeftMax, encoderLowerLimit, encoderUpperLimit, leftEncoderSeries);
    findMinMax(&encoderRightMin, &encoderRightMax, encoderLowerLimit, encoderUpperLimit, rightEncoderSeries);

    controlSignalChart->axes(Qt::Vertical).first()->setRange(controlMin, controlMax);
    motorChart->axes(Qt::Vertical).first()->setRange(std::min(motorAMin,motorBMin), std::max(motorAMax,motorBMax));
    encoderTicksChart->axes(Qt::Vertical).first()->setRange(std::min(encoderLeftMin,encoderRightMin), std::max(encoderLeftMax,encoderRightMax));

	// Update bar motorChart
	QStringList cat;
	for (int i = 0; i < 4; ++i) {
        const auto val = tofArray.back()[i] * 100;
		tofChart->replace(i, val);
		// Set the distance from metres to centimetres.
		cat << QString::number(val);
	}
	categories = cat;
	axisXBar->setCategories(categories);

	// Update compass motorChart
	gyroSeries->clear();
	gyroSeries->append(RAD2DEG(gyroZArray.back()), 100); // Point on the perimeter of the compass
	gyroSeries->append(RAD2DEG(gyroZArray.back()), 0);	// Center of the compass

	// qDebug() << "Series size: " << motorSeriesA->count();
	if (motorBufferSize > MOTOR_AXIS_LIMIT) {
		motorSeriesA->remove(0);
		motorSeriesB->remove(0);
		controlSeries->remove(0);
		leftEncoderSeries->remove(0);
		rightEncoderSeries->remove(0);
		motorBufferSize--;
	}
}

void MainWindow::findMinMax(double *min, double *max, int lowerLimit, int upperLimit,  QLineSeries *series){
    *min = lowerLimit;
    *max = upperLimit;
    for(int i = 0; i<series->points().size();i++){
        if (series->points().at(i).y() > *max){
            *max = series->points().at(i).y();
        }
        if (series->points().at(i).y() < *min){
            *min = series->points().at(i).y();
        }
    }
}
