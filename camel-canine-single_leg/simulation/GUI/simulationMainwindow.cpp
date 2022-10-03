//
// Created by jaehoon on 22. 4. 26.
//

#include "simulationMainwindow.h"
#include "ui_simulationMainwindow.h"
#include "SingleLeggedSharedMemory.hpp"
#include <iostream>

extern pSHM sharedMemory;

MainWindow *MainUI;

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow) {
    MainUI = this;
    ui->setupUi(this);

    ui->widget->legend->setVisible(true);
    ui->widget->legend->setFont(QFont("Helvetica", 9));
    ui->widget->addGraph();
    ui->widget->graph(0)->setName("position_z");
    ui->widget->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget->addGraph();
    ui->widget->graph(1)->setName("desired position_z");
    ui->widget->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);
    ui->widget->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_2->legend->setVisible(true);
    ui->widget_2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_2->addGraph();
    ui->widget_2->graph(0)->setName("velocity_z");
    ui->widget_2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_2->addGraph();
    ui->widget_2->graph(1)->setName("desired velocity_z");
    ui->widget_2->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_2->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);
    ui->widget_2->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_3->legend->setVisible(true);
    ui->widget_3->legend->setFont(QFont("Helvetica", 9));
    ui->widget_3->addGraph();
    ui->widget_3->graph(0)->setName("hip_torque");
    ui->widget_3->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_3->addGraph();
    ui->widget_3->graph(1)->setName("knee_torque");
    ui->widget_3->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->widget_3->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);
    ui->widget_3->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::realtimeDataSlot() {
    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0;
    int fps = 120;
    static double lastPointKey = 0;
    if ((key-lastPointKey > double(1/fps))&&(!isSimulationEnd)) // at most add point every 10 ms
    {
        plotWidget1();
        plotWidget2();
        plotWidget3();
        lastPointKey = key;
    }
}

void MainWindow::on_pushButton_clicked() {
    std::cout << "'Run' button is clicked" << std::endl;
    isSimulationEnd = false;
    if (button1) { button1 = false; }
    else { button1 = true; }
}

void MainWindow::plotWidget1() {
    if (sharedMemory->positionZ < yMinWidget1) { yMinWidget1 = sharedMemory->positionZ; }
    if (sharedMemory->positionZ > yMaxWidget1) { yMaxWidget1 = sharedMemory->positionZ; }
    if (sharedMemory->desiredPositionZ < yMinWidget1) { yMinWidget1 = sharedMemory->desiredPositionZ; }
    if (sharedMemory->desiredPositionZ > yMaxWidget1) { yMaxWidget1 = sharedMemory->desiredPositionZ; }
    ui->widget->graph(0)->addData(sharedMemory->localTime, sharedMemory->positionZ);
    ui->widget->graph(1)->addData(sharedMemory->localTime, sharedMemory->desiredPositionZ);

    // set axes ranges, so we see all data:
    ui->widget->xAxis->setRange(sharedMemory->localTime - intervalTime, sharedMemory->localTime + 0.001);
    ui->widget->yAxis->setRange(yMinWidget1 - 0.001, yMaxWidget1 + 0.001);
    ui->widget->replot();
}

void MainWindow::plotWidget2() {
    if (sharedMemory->velocityZ < yMinWidget2) { yMinWidget2 = sharedMemory->velocityZ; }
    if (sharedMemory->velocityZ > yMaxWidget2) { yMaxWidget2 = sharedMemory->velocityZ; }
    if (sharedMemory->desiredVelocityZ < yMinWidget2) { yMinWidget2 = sharedMemory->desiredVelocityZ; }
    if (sharedMemory->desiredVelocityZ > yMaxWidget2) { yMaxWidget2 = sharedMemory->desiredVelocityZ; }
    ui->widget_2->graph(0)->addData(sharedMemory->localTime, sharedMemory->velocityZ);
    ui->widget_2->graph(1)->addData(sharedMemory->localTime, sharedMemory->desiredVelocityZ);

    // set axes ranges, so we see all data:
    ui->widget_2->xAxis->setRange(sharedMemory->localTime - intervalTime, sharedMemory->localTime + 0.001);
    ui->widget_2->yAxis->setRange(-0.7, 0.7);
    ui->widget_2->replot();
}

void MainWindow::plotWidget3() {
    if (sharedMemory->jointTorque[0] < yMinWidget3) { yMinWidget3 = sharedMemory->jointTorque[0]; }
    if (sharedMemory->jointTorque[0] > yMaxWidget3) { yMaxWidget3 = sharedMemory->jointTorque[0]; }
    if (sharedMemory->jointTorque[1] < yMinWidget3) { yMinWidget3 = sharedMemory->jointTorque[1]; }
    if (sharedMemory->jointTorque[1] > yMaxWidget3) { yMaxWidget3 = sharedMemory->jointTorque[1]; }
    ui->widget_3->graph(0)->addData(sharedMemory->localTime, sharedMemory->jointTorque[0]);
    ui->widget_3->graph(1)->addData(sharedMemory->localTime, sharedMemory->jointTorque[1]);

    // set axes ranges, so we see all data:
    ui->widget_3->xAxis->setRange(sharedMemory->localTime - intervalTime, sharedMemory->localTime + 0.001);
    ui->widget_3->yAxis->setRange(yMinWidget3 - 0.01, yMaxWidget3 + 0.01);
    ui->widget_3->replot();
}

