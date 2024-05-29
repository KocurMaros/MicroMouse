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
	QChart *motorChart;
	QBarSeries *barSeries;
	QBarSet *tofChart;
	QStringList categories;
	QBarCategoryAxis *axisXBar;
	QPolarChart *gyroChart;
	QTimer *timer;
	QWidget *centralWidget;
	QChartView *plotMotor;
	QChartView *plotTof;
	QChartView *plotGyro;
	QLineSeries *motorSeriesA;
	QLineSeries *motorSeriesB;
	QBarSeries *tofSeries;
	QLineSeries *gyroSeries;
	QUdpSocket *udpSocket;
	// 4.2 - 3.3
	QLineEdit *batteryLineEdit;
	QLineEdit *gyroFrequencyLineEdit;
	QLineEdit *posXLineEdit;
	QLineEdit *posYLineEdit;
	QPushButton *clearMotorChartButton;
	QPushButton *pauseButton;
	QString localIP;
	quint16 localPort;
	double timeStamp;
	double motorA;
	double motorB;
	QVector<QVector<double>> tofArray;
	QVector<double> gyroZArray;
	double batteryVoltage;
	double gyroFrequency;
	std::mutex mut;
	int bufferSize;
	int x;
	std::atomic<int> motorBufferSize;
};

#endif // MAINWINDOW_H
