#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QPolarChart>
#include <QUdpSocket>
#include <QTimer>
#include <mutex>

QT_CHARTS_USE_NAMESPACE

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
    QString localIP;
    quint16 localPort;
    QVector<double> timestampArray;
    QVector<double> motorArrayA;
    QVector<double> motorArrayB;
    QVector<QVector<double>> tofArray;
    QVector<double> gyroZArray;
    std::mutex mut;
    int bufferSize;
    int x;
};

#endif // MAINWINDOW_H
