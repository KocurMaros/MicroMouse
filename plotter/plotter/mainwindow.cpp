#include "mainwindow.h"
#include <QtCharts/QValueAxis>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QCategoryAxis>
#include <QRandomGenerator>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    chart(new QChart()),
    series1(new QLineSeries()),
    series2(new QLineSeries()),
    barSeries(new QBarSeries()),
    barSet(new QBarSet("BarSet")),
    compassChart(new QPolarChart()),
    compassSeries(new QLineSeries()),
    timer(new QTimer(this)),
    x(0)
{
    // Setup line chart
    chart->legend()->hide();
    chart->addSeries(series1);
    chart->addSeries(series2);
    chart->createDefaultAxes();

    auto *axisX = new QValueAxis;
    axisX->setRange(0, 100);
    axisX->setLabelFormat("%i");
    chart->setAxisX(axisX, series1);
    chart->setAxisX(axisX, series2);

    auto *axisY = new QValueAxis;
    axisY->setRange(-50, 50);
    axisY->setLabelFormat("%i");
    chart->setAxisY(axisY, series1);
    chart->setAxisY(axisY, series2);

    series1->setName("Series 1");
    series1->setColor(Qt::blue);

    series2->setName("Series 2");
    series2->setColor(Qt::red);

    // Setup bar chart
    barSet->append({0, 0, 0, 0}); // Initialize with 4 values
    barSeries->append(barSet);

    QChart *barChart = new QChart();
    barChart->addSeries(barSeries);
    barChart->legend()->hide();

    QStringList categories;
    categories << "A" << "B" << "C" << "D";
    QBarCategoryAxis *axisXBar = new QBarCategoryAxis();
    axisXBar->append(categories);
    barChart->addAxis(axisXBar, Qt::AlignBottom);
    barSeries->attachAxis(axisXBar);

    auto *axisYBar = new QValueAxis;
    axisYBar->setRange(0, 100);
    barChart->addAxis(axisYBar, Qt::AlignLeft);
    barSeries->attachAxis(axisYBar);

    // Setup compass chart
    compassChart->addSeries(compassSeries);

    QValueAxis *angularAxis = new QValueAxis();
    angularAxis->setTickCount(9);
    angularAxis->setLabelFormat("%.0f");
    angularAxis->setRange(0, 360);
    compassChart->addAxis(angularAxis, QPolarChart::PolarOrientationAngular);
    compassSeries->attachAxis(angularAxis);

    QValueAxis *radialAxis = new QValueAxis();
    radialAxis->setLabelFormat("%d");
    radialAxis->setRange(0, 100);
    compassChart->addAxis(radialAxis, QPolarChart::PolarOrientationRadial);
    compassSeries->attachAxis(radialAxis);

    // Configure layout
    QChartView *lineChartView = new QChartView(chart);
    lineChartView->setRenderHint(QPainter::Antialiasing);

    QChartView *barChartView = new QChartView(barChart);
    barChartView->setRenderHint(QPainter::Antialiasing);

    QChartView *compassChartView = new QChartView(compassChart);
    compassChartView->setRenderHint(QPainter::Antialiasing);

    QWidget *centralWidget = new QWidget;
    QVBoxLayout *layout = new QVBoxLayout(centralWidget);
    layout->addWidget(lineChartView);
    layout->addWidget(barChartView);
    layout->addWidget(compassChartView);

    setCentralWidget(centralWidget);

    connect(timer, &QTimer::timeout, this, &MainWindow::updateChart);
    timer->start(100);  // Update every 100 ms
}

MainWindow::~MainWindow()
{
}

void MainWindow::updateChart()
{
    qreal y1 = QRandomGenerator::global()->bounded(-50, 50);
    qreal y2 = QRandomGenerator::global()->bounded(-50, 50);
    series1->append(x, y1);
    series2->append(x, y2);
    x++;

    if (series1->count() > 100) {
        series1->remove(0);
        series2->remove(0);
    }

    chart->axes(Qt::Horizontal).first()->setRange(qMax(0, x - 100), x);

    // Update bar chart
    for (int i = 0; i < barSet->count(); ++i) {
        barSet->replace(i, QRandomGenerator::global()->bounded(0, 100));
    }

    // Update compass chart
    compassSeries->clear();
    qreal angle = QRandomGenerator::global()->bounded(0, 360);
    compassSeries->append(angle, 100);  // Point on the perimeter of the compass
    compassSeries->append(angle, 0);    // Center of the compass
}
