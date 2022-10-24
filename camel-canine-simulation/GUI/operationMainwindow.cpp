
#include "operationMainwindow.h"
#include "ui_operationMainwindow.h"
#include "CanineSimSharedMemory.hpp"
#include <iostream>

extern pSHM sharedMemory;
MainWindow* MainUI;

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    MainUI = this;
    ui->setupUi(this);

    ui->kneeTorMin->setText("KneeTorMin(N*m):");
    ui->kneeTorMax->setText("KneeTorMax(N*m):");
    ui->hipTorMin->setText("hipTorMin(N*m):");
    ui->hipTorMax->setText("hipTorMax(N*m):");

    ui->widget_pos0->legend->setVisible(true);
    ui->widget_pos0->legend->setFont(QFont("Helvetica", 9));
    ui->widget_pos0->addGraph();
    ui->widget_pos0->graph(0)->setName("position-base");
    ui->widget_pos0->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_pos0->addGraph();
    ui->widget_pos0->graph(1)->setName("desired position-base");
    ui->widget_pos0->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_pos0->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_pos1->legend->setVisible(true);
    ui->widget_pos1->legend->setFont(QFont("Helvetica", 9));
    ui->widget_pos1->addGraph();
    ui->widget_pos1->graph(0)->setName("position-knee");
    ui->widget_pos1->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_pos1->addGraph();
    ui->widget_pos1->graph(1)->setName("desired position-knee");
    ui->widget_pos1->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_pos1->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_pos2->legend->setVisible(true);
    ui->widget_pos2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_pos2->addGraph();
    ui->widget_pos2->graph(0)->setName("position-hip");
    ui->widget_pos2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_pos2->addGraph();
    ui->widget_pos2->graph(1)->setName("desired position-hip");
    ui->widget_pos2->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_pos2->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_vel0->legend->setVisible(true);
    ui->widget_vel0->legend->setFont(QFont("Helvetica", 9));
    ui->widget_vel0->addGraph();
    ui->widget_vel0->graph(0)->setName("velocity-base");
    ui->widget_vel0->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_vel0->addGraph();
    ui->widget_vel0->graph(1)->setName("desired velocity-base");
    ui->widget_vel0->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_vel0->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_vel1->legend->setVisible(true);
    ui->widget_vel1->legend->setFont(QFont("Helvetica", 9));
    ui->widget_vel1->addGraph();
    ui->widget_vel1->graph(0)->setName("velocity-knee");
    ui->widget_vel1->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_vel1->addGraph();
    ui->widget_vel1->graph(1)->setName("desired velocity-knee");
    ui->widget_vel1->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_vel1->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_vel2->legend->setVisible(true);
    ui->widget_vel2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_vel2->addGraph();
    ui->widget_vel2->graph(0)->setName("velocity-hip");
    ui->widget_vel2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_vel2->addGraph();
    ui->widget_vel2->graph(1)->setName("desired velocity-hip");
    ui->widget_vel2->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_vel2->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_tor1->legend->setVisible(true);
    ui->widget_tor1->legend->setFont(QFont("Helvetica", 9));
    ui->widget_tor1->addGraph();
    ui->widget_tor1->graph(0)->setName("torque-knee");
    ui->widget_tor1->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_tor1->addGraph();
    ui->widget_tor1->graph(1)->setName("desired torque-knee");
    ui->widget_tor1->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_tor1->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_tor2->legend->setVisible(true);
    ui->widget_tor2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_tor2->addGraph();
    ui->widget_tor2->graph(0)->setName("torque-hip");
    ui->widget_tor2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_tor2->addGraph();
    ui->widget_tor2->graph(1)->setName("desired torque-hip");
    ui->widget_tor2->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_tor2->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    std::cout << "'Run' button is clicked" << std::endl;
    button1 = !button1;
}


void MainWindow::realtimeDataSlot()
{
    static QTime time(QTime::currentTime());
    double key = time.elapsed() / 1000.0;
    int fps = 120;
    static double lastPointKey = 0;
    if (key - lastPointKey > double(1 / fps)) // at most add point every 10 ms
    {
        plotWidget_pos0();
        plotWidget_pos1();
        plotWidget_pos2();
        plotWidget_vel0();
        plotWidget_vel1();
        plotWidget_vel2();
        plotWidget_tor1();
        plotWidget_tor2();
        updateKneeTorMin();
        updateKneeTorMax();
        updateHipTorMin();
        updateHipTorMax();
        lastPointKey = key;
    }

// calculate frames per second:
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key - lastFpsKey > 2) // average fps over 2 seconds
    {
        ui->statusBar->showMessage(
            QString("%1 FPS, Total Data points: %2")
                .arg(frameCount / (key - lastFpsKey), 0, 'f', 0)
                .arg(ui->widget_pos0->graph(0)->data()->size() + ui->widget_pos0->graph(1)->data()->size()), 0);
        lastFpsKey = key;
        frameCount = 0;
    }
}

void MainWindow::updateKneeTorMin()
{
    ui->kneeTorMinValue->setText(QString::number(yMinWidgetTor1));
}

void MainWindow::updateKneeTorMax()
{
    ui->kneeTorMaxValue->setText(QString::number(yMaxWidgetTor1));
}

void MainWindow::updateHipTorMin()
{
    ui->hipTorMinValue->setText(QString::number(yMinWidgetTor2));
}

void MainWindow::updateHipTorMax()
{
    ui->hipTorMaxValue->setText(QString::number(yMaxWidgetTor2));
}

void MainWindow::plotWidget_pos0()
{
    if (sharedMemory->PositionBase < yMinWidgetPos0)
    {
        yMinWidgetPos0 = sharedMemory->PositionBase;
    }
    if (sharedMemory->PositionBase > yMaxWidgetPos0)
    {
        yMaxWidgetPos0 = sharedMemory->PositionBase;
    }
    if (sharedMemory->DesiredPositionBase < yMinWidgetPos0)
    {
        yMinWidgetPos0 = sharedMemory->DesiredPositionBase;
    }
    if (sharedMemory->DesiredPositionBase > yMaxWidgetPos0)
    {
        yMaxWidgetPos0 = sharedMemory->DesiredPositionBase;
    }
    ui->widget_pos0->graph(0)->addData(sharedMemory->simTime, sharedMemory->PositionBase);
    ui->widget_pos0->graph(1)->addData(sharedMemory->simTime, sharedMemory->DesiredPositionBase);

    // set axes ranges, so we see all data:
    ui->widget_pos0->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_pos0->yAxis->setRange(yMinWidgetPos0 - 0.001, yMaxWidgetPos0 + 0.001);
    ui->widget_pos0->replot();
}

void MainWindow::plotWidget_pos1()
{
    if (sharedMemory->PositionKnee < yMinWidgetPos1)
    {
        yMinWidgetPos1 = sharedMemory->PositionKnee;
    }
    if (sharedMemory->PositionKnee > yMaxWidgetPos1)
    {
        yMaxWidgetPos1 = sharedMemory->PositionKnee;
    }
    if (sharedMemory->DesiredPositionKnee < yMinWidgetPos1)
    {
        yMinWidgetPos1 = sharedMemory->DesiredPositionKnee;
    }
    if (sharedMemory->DesiredPositionKnee > yMaxWidgetPos1)
    {
        yMaxWidgetPos1 = sharedMemory->DesiredPositionKnee;
    }
    ui->widget_pos1->graph(0)->addData(sharedMemory->simTime, sharedMemory->PositionKnee);
    ui->widget_pos1->graph(1)->addData(sharedMemory->simTime, sharedMemory->DesiredPositionKnee);

    // set axes ranges, so we see all data:
    ui->widget_pos1->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_pos1->yAxis->setRange(yMinWidgetPos1 - 0.001, yMaxWidgetPos1 + 0.001);
    ui->widget_pos1->replot();
}

void MainWindow::plotWidget_pos2()
{
    if (sharedMemory->PositionHip < yMinWidgetPos2)
    {
        yMinWidgetPos2 = sharedMemory->PositionHip;
    }
    if (sharedMemory->PositionHip > yMaxWidgetPos2)
    {
        yMaxWidgetPos2 = sharedMemory->PositionHip;
    }
    if (sharedMemory->DesiredPositionHip < yMinWidgetPos2)
    {
        yMinWidgetPos2 = sharedMemory->DesiredPositionHip;
    }
    if (sharedMemory->DesiredPositionHip > yMaxWidgetPos2)
    {
        yMaxWidgetPos2 = sharedMemory->DesiredPositionHip;
    }
    ui->widget_pos2->graph(0)->addData(sharedMemory->simTime, sharedMemory->PositionHip);
    ui->widget_pos2->graph(1)->addData(sharedMemory->simTime, sharedMemory->DesiredPositionHip);

    // set axes ranges, so we see all data:
    ui->widget_pos2->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_pos2->yAxis->setRange(yMinWidgetPos2 - 0.001, yMaxWidgetPos2 + 0.001);
    ui->widget_pos2->replot();
}

void MainWindow::plotWidget_vel0()
{
    if (sharedMemory->VelocityBase < yMinWidgetVel0)
    {
        yMinWidgetVel0 = sharedMemory->VelocityBase;
    }
    if (sharedMemory->VelocityBase > yMaxWidgetVel0)
    {
        yMaxWidgetVel0 = sharedMemory->VelocityBase;
    }
    if (sharedMemory->DesiredVelocityBase < yMinWidgetVel0)
    {
        yMinWidgetVel0 = sharedMemory->DesiredVelocityBase;
    }
    if (sharedMemory->DesiredVelocityBase > yMaxWidgetVel0)
    {
        yMaxWidgetVel0 = sharedMemory->DesiredVelocityBase;
    }
    ui->widget_vel0->graph(0)->addData(sharedMemory->simTime, sharedMemory->VelocityBase);
    ui->widget_vel0->graph(1)->addData(sharedMemory->simTime, sharedMemory->DesiredVelocityBase);

    // set axes ranges, so we see all data:
    ui->widget_vel0->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_vel0->yAxis->setRange(yMinWidgetVel0 - 0.001, yMaxWidgetVel0 + 0.001);
    ui->widget_vel0->replot();
}

void MainWindow::plotWidget_vel1()
{
    if (sharedMemory->VelocityKnee < yMinWidgetVel1)
    {
        yMinWidgetVel1 = sharedMemory->VelocityKnee;
    }
    if (sharedMemory->VelocityKnee > yMaxWidgetVel1)
    {
        yMaxWidgetVel1 = sharedMemory->VelocityKnee;
    }
    if (sharedMemory->DesiredVelocityKnee < yMinWidgetVel1)
    {
        yMinWidgetVel1 = sharedMemory->DesiredVelocityKnee;
    }
    if (sharedMemory->DesiredVelocityKnee > yMaxWidgetVel1)
    {
        yMaxWidgetVel1 = sharedMemory->DesiredVelocityKnee;
    }
    ui->widget_vel1->graph(0)->addData(sharedMemory->simTime, sharedMemory->VelocityKnee);
    ui->widget_vel1->graph(1)->addData(sharedMemory->simTime, sharedMemory->DesiredVelocityKnee);

    // set axes ranges, so we see all data:
    ui->widget_vel1->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_vel1->yAxis->setRange(yMinWidgetVel1 - 0.001, yMaxWidgetVel1 + 0.001);
    ui->widget_vel1->replot();
}

void MainWindow::plotWidget_vel2()
{
    if (sharedMemory->VelocityHip < yMinWidgetVel2)
    {
        yMinWidgetVel2 = sharedMemory->VelocityHip;
    }
    if (sharedMemory->VelocityHip > yMaxWidgetVel2)
    {
        yMaxWidgetVel2 = sharedMemory->VelocityHip;
    }
    if (sharedMemory->DesiredVelocityHip < yMinWidgetVel2)
    {
        yMinWidgetVel2 = sharedMemory->DesiredVelocityHip;
    }
    if (sharedMemory->DesiredVelocityHip > yMaxWidgetVel2)
    {
        yMaxWidgetVel2 = sharedMemory->DesiredVelocityHip;
    }
    ui->widget_vel2->graph(0)->addData(sharedMemory->simTime, sharedMemory->VelocityHip);
    ui->widget_vel2->graph(1)->addData(sharedMemory->simTime, sharedMemory->DesiredVelocityHip);

    // set axes ranges, so we see all data:
    ui->widget_vel2->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_vel2->yAxis->setRange(yMinWidgetVel2 - 0.001, yMaxWidgetVel2 + 0.001);
    ui->widget_vel2->replot();
}

void MainWindow::plotWidget_tor1()
{
    if (sharedMemory->TorqueKnee < yMinWidgetTor1)
    {
        yMinWidgetTor1 = sharedMemory->TorqueKnee;
    }
    if (sharedMemory->TorqueKnee > yMaxWidgetTor1)
    {
        yMaxWidgetTor1 = sharedMemory->TorqueKnee;
    }
    if (sharedMemory->DesiredTorqueKnee < yMinWidgetTor1)
    {
        yMinWidgetTor1 = sharedMemory->DesiredTorqueKnee;
    }
    if (sharedMemory->DesiredTorqueKnee > yMaxWidgetTor1)
    {
        yMaxWidgetTor1 = sharedMemory->DesiredTorqueKnee;
    }
    ui->widget_tor1->graph(0)->addData(sharedMemory->simTime, sharedMemory->TorqueKnee);
    ui->widget_tor1->graph(1)->addData(sharedMemory->simTime, sharedMemory->DesiredTorqueKnee);

    // set axes ranges, so we see all data:
    ui->widget_tor1->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_tor1->yAxis->setRange(yMinWidgetTor1 - 0.001, yMaxWidgetTor1 + 0.001);
    ui->widget_tor1->replot();
}

void MainWindow::plotWidget_tor2()
{
    if (sharedMemory->TorqueHip < yMinWidgetTor2)
    {
        yMinWidgetTor2 = sharedMemory->TorqueHip;
    }
    if (sharedMemory->TorqueHip > yMaxWidgetTor2)
    {
        yMaxWidgetTor2 = sharedMemory->TorqueHip;
    }
    if (sharedMemory->DesiredTorqueHip < yMinWidgetTor2)
    {
        yMinWidgetTor2 = sharedMemory->DesiredTorqueHip;
    }
    if (sharedMemory->DesiredTorqueHip > yMaxWidgetTor2)
    {
        yMaxWidgetTor2 = sharedMemory->DesiredTorqueHip;
    }
    ui->widget_tor2->graph(0)->addData(sharedMemory->simTime, sharedMemory->TorqueHip);
    ui->widget_tor2->graph(1)->addData(sharedMemory->simTime, sharedMemory->DesiredTorqueHip);

    // set axes ranges, so we see all data:
    ui->widget_tor2->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_tor2->yAxis->setRange(yMinWidgetTor2 - 0.001, yMaxWidgetTor2 + 0.001);
    ui->widget_tor2->replot();
}