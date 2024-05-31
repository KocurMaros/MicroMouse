#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QLineSeries>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QPolarChart>
#include <QPushButton>
#include <QUdpSocket>
#include <QTimer>
#include <QLineEdit>
#include <mutex>
#include <atomic>

#ifdef WIN32
QT_CHARTS_USE_NAMESPACE
#endif

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow();

private slots:
	void updateChart();
	void readPendingDatagrams();

private:
	QChartView *plotMotor;
	QChart *motorChart;
	QLineSeries *motorSeriesA;
	QLineSeries *motorSeriesB;

    double motorAMax;
    double motorAMin;

    double motorBMax;
    double motorBMin;

    double controlMax;
    double controlMin;

    double encoderLeftMin;
    double encoderLeftMax;

    double encoderRightMin;
    double encoderRightMax;

    int motorLowerLimit;
    int motorUpperLimit;

    int controlLowerLimit;
    int controlUpperLimit;

    int encoderLowerLimit;
    int encoderUpperLimit;


	QChartView *plotControl;
	QChart *controlSignalChart;
	QLineSeries *controlSeries;

	// 50'000
	// 2 encoders
	QChartView *plotEncoderTicks;
	QChart *encoderTicksChart;
	QLineSeries *leftEncoderSeries;
	QLineSeries *rightEncoderSeries;
	double leftenc;
	double rightenc;

	QLineEdit *batteryLineEdit;

	QChartView *plotGyro;
	QLineSeries *gyroSeries;
	QLineEdit *gyroFrequencyLineEdit;
	QPolarChart *gyroChart;

	QBarSet *tofChart;
	QBarCategoryAxis *axisXBar;

	QStringList categories;
	QTimer *timer;
	QWidget *centralWidget;
	QChartView *plotTof;
	QBarSeries *tofSeries;
	QUdpSocket *udpSocket;
	// 4.2 - 3.3
	QLineEdit *posXLineEdit;
	QLineEdit *posYLineEdit;
	QPushButton *clearMotorChartButton;
	QPushButton *pauseButton;
	QString localIP;
	quint16 localPort;
	double timestamp;
	double motorA;
	double control;
	double motorB;
	QVector<QVector<double>> tofArray;
	QVector<double> gyroZArray;
	double batteryVoltage;
	double gyroFrequency;
	std::mutex mut;
	int bufferSize;
	int x;
	std::atomic<int> motorBufferSize;
	std::atomic<int> motorBufferSizeLast;
	bool connected;
	QTimer *disconnectedTimer;
    void findMinMax(double *min, double *max ,int lowerLimit, int upperLimit, QLineSeries *series);
};

#endif // MAINWINDOW_H
