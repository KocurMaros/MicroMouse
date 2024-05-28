#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarCategoryAxis>
#include <QTimer>

QT_CHARTS_USE_NAMESPACE

    class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateChart();

private:
    QChart *chart;
    QLineSeries *series1;
    QLineSeries *series2;
    QBarSeries *barSeries;
    QBarSet *barSet;
    QTimer *timer;
    int x;
};

#endif // MAINWINDOW_H
