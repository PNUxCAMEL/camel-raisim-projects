#include "mainwindow.h"
#include "ui_mainwindow.h"

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;
extern pCUSTOM_DATA sharedCustom;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    InitLineEdit();
    InitTable(ui->TW_MOTOR);
    GraphInitialize();

    displayTimer = new QTimer();
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(10);

    graphTimer = new QTimer();
    connect(graphTimer, SIGNAL(timeout()), this, SLOT(GraphUpdate()));
    graphTimer->start(2);

    graphOffset = 15.0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::InitLineEdit(){
    ui->LE_CAN_STATUS->setStyleSheet("background-color:red");
    ui->LE_VISUAL_STATUS->setStyleSheet("background-color:red");
    ui->LE_MOTOR_ON_STATUS->setStyleSheet("background-color:red");
    ui->LE_MOTOR_OFF_STATUS->setStyleSheet("background-color:red");
    ui->LE_HOME->setStyleSheet("background-color:pink");
    ui->LE_PD_CONTROL->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM1->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM2->setStyleSheet("background-color:pink");
}

void MainWindow::InitTable(QTableWidget *table){
    QFont tableFont;
    tableFont.setPointSize(7);

    const int col_num = 7;
    const int row_num = 2;
    const int col_width = 45;
    const int item_height = 25;
    const int item_width = 30;
    const int low_height = 25;

    // Horizontal - Column
    for(int i=0; i<col_num; i++){
        table->insertColumn(i);
        table->setHorizontalHeaderItem(i, new QTableWidgetItem());
        table->horizontalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->horizontalHeaderItem(i)->setFont(tableFont);
        table->horizontalHeaderItem(i)->setSizeHint(QSize(col_width, item_height));
        table->setColumnWidth(i, col_width);
    }

    table->horizontalHeaderItem(0)->setText("Status");
    table->horizontalHeaderItem(1)->setText("Error");
    table->horizontalHeaderItem(2)->setText("Temp");
    table->horizontalHeaderItem(3)->setText("Voltage");
    table->horizontalHeaderItem(4)->setText("Angle\nMotor");
    table->horizontalHeaderItem(5)->setText("Torque\nDes");
    table->horizontalHeaderItem(6)->setText("Torque\nMotor");


    // Vertical - Row
    for(int i=0; i<row_num; i++){
        table->insertRow(i);
        table->setRowHeight(i,low_height);
        table->setVerticalHeaderItem(i, new QTableWidgetItem());
        table->verticalHeaderItem(i)->setSizeHint(QSize(item_width, item_height));
        table->verticalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->verticalHeaderItem(i)->setFont(tableFont);
    }
    table->verticalHeaderItem(0)->setText("MT H");
    table->verticalHeaderItem(1)->setText("MT K");


    for(int i=0; i<row_num; i++){
        for(int j=0; j<col_num; j++){
            table->setItem(i, j, new QTableWidgetItem());
            table->item(i,j)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
            table->item(i,j)->setFlags(table->item(i,j)->flags() & ~Qt::ItemIsEditable);
            table->item(i,j)->setFont(tableFont);
        }
    }

    int col_width_sum = col_width * (col_num+1);
//    table->setMinimumWidth(item_width + col_width_sum + 15);
//    table->setMaximumWidth(item_width + col_width_sum + 15);
    table->setMinimumHeight(low_height*(row_num+2));
    table->setMaximumHeight(low_height*(row_num+2));
//    table->resizeColumnsToContents();

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

void MainWindow::GraphInitialize(){
    QPen myPen, dotPen, filterPen;
    myPen.setWidthF(1);
    filterPen.setStyle(Qt::DotLine);
    filterPen.setWidth(1);
    dotPen.setStyle(Qt::DotLine);
    dotPen.setWidth(20);
    dotPen.setWidthF(2);
    dotPen.setColor(Qt::gray);

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");

    ui->PLOT_POS_HIP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_POS_KNEE->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_HIP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_KNEE->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_TORQUE_HIP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_TORQUE_KNEE->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->PLOT_CUSTOM_1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_5->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_6->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    // Custom Plot : Robot
    myPen.setWidthF(1.5);
    myPen.setColor(Qt::blue);
    ui->PLOT_POS_HIP->legend->setVisible(true);
    ui->PLOT_POS_HIP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_POS_HIP->addGraph();
    ui->PLOT_POS_HIP->graph(0)->setPen(myPen);
    ui->PLOT_POS_HIP->graph(0)->setName("position hip");

    ui->PLOT_POS_KNEE->legend->setVisible(true);
    ui->PLOT_POS_KNEE->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_POS_KNEE->addGraph();
    ui->PLOT_POS_KNEE->graph(0)->setPen(myPen);
    ui->PLOT_POS_KNEE->graph(0)->setName("position knee");

    ui->PLOT_VEL_HIP->legend->setVisible(true);
    ui->PLOT_VEL_HIP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_HIP->addGraph();
    ui->PLOT_VEL_HIP->graph(0)->setPen(myPen);
    ui->PLOT_VEL_HIP->graph(0)->setName("velocity hip");

    ui->PLOT_VEL_KNEE->legend->setVisible(true);
    ui->PLOT_VEL_KNEE->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_KNEE->addGraph();
    ui->PLOT_VEL_KNEE->graph(0)->setPen(myPen);
    ui->PLOT_VEL_KNEE->graph(0)->setName("velocity knee");

    ui->PLOT_TORQUE_HIP->legend->setVisible(true);
    ui->PLOT_TORQUE_HIP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_TORQUE_HIP->addGraph();
    ui->PLOT_TORQUE_HIP->graph(0)->setPen(myPen);
    ui->PLOT_TORQUE_HIP->graph(0)->setName("torque hip desired");
    ui->PLOT_TORQUE_HIP->addGraph();
    myPen.setColor(Qt::red);
    ui->PLOT_TORQUE_HIP->graph(1)->setPen(myPen);
    ui->PLOT_TORQUE_HIP->graph(1)->setName("torque hip current");

    myPen.setColor(Qt::blue);
    ui->PLOT_TORQUE_KNEE->legend->setVisible(true);
    ui->PLOT_TORQUE_KNEE->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_TORQUE_KNEE->addGraph();
    ui->PLOT_TORQUE_KNEE->graph(0)->setPen(myPen);
    ui->PLOT_TORQUE_KNEE->graph(0)->setName("torque knee desired");
    ui->PLOT_TORQUE_KNEE->addGraph();
    myPen.setColor(Qt::red);
    ui->PLOT_TORQUE_KNEE->graph(1)->setPen(myPen);
    ui->PLOT_TORQUE_KNEE->graph(1)->setName("torque knee current");

    // Custom Plot : Custom
    myPen.setColor(Qt::blue);
    ui->PLOT_CUSTOM_1->addGraph();
    ui->PLOT_CUSTOM_1->graph(0)->setPen(myPen);
    myPen.setColor(Qt::red);
    ui->PLOT_CUSTOM_1->addGraph();
    ui->PLOT_CUSTOM_1->graph(1)->setPen(myPen);

    myPen.setColor(Qt::blue);
    ui->PLOT_CUSTOM_2->addGraph();
    ui->PLOT_CUSTOM_2->graph(0)->setPen(myPen);
    myPen.setColor(Qt::red);
    ui->PLOT_CUSTOM_2->addGraph();
    ui->PLOT_CUSTOM_2->graph(1)->setPen(myPen);

    myPen.setColor(Qt::blue);
    ui->PLOT_CUSTOM_3->addGraph();
    ui->PLOT_CUSTOM_3->graph(0)->setPen(myPen);
    myPen.setColor(Qt::red);
    ui->PLOT_CUSTOM_3->addGraph();
    ui->PLOT_CUSTOM_3->graph(1)->setPen(myPen);

    myPen.setColor(Qt::blue);
    ui->PLOT_CUSTOM_4->addGraph();
    ui->PLOT_CUSTOM_4->graph(0)->setPen(myPen);
    myPen.setColor(Qt::red);
    ui->PLOT_CUSTOM_4->addGraph();
    ui->PLOT_CUSTOM_4->graph(1)->setPen(myPen);

    myPen.setColor(Qt::blue);
    ui->PLOT_CUSTOM_5->addGraph();
    ui->PLOT_CUSTOM_5->graph(0)->setPen(myPen);
    myPen.setColor(Qt::red);
    ui->PLOT_CUSTOM_5->addGraph();
    ui->PLOT_CUSTOM_5->graph(1)->setPen(myPen);

    myPen.setColor(Qt::blue);
    ui->PLOT_CUSTOM_6->addGraph();
    ui->PLOT_CUSTOM_6->graph(0)->setPen(myPen);
    myPen.setColor(Qt::red);
    ui->PLOT_CUSTOM_6->addGraph();
    ui->PLOT_CUSTOM_6->graph(1)->setPen(myPen);

    ui->PLOT_POS_HIP->xAxis->setTicker(timeTicker);
    ui->PLOT_POS_KNEE->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_HIP->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_KNEE->xAxis->setTicker(timeTicker);
    ui->PLOT_TORQUE_HIP->xAxis->setTicker(timeTicker);
    ui->PLOT_TORQUE_KNEE->xAxis->setTicker(timeTicker);

    ui->PLOT_POS_HIP->yAxis->setRange(-1, 1);
    ui->PLOT_POS_KNEE->yAxis->setRange(-1, 1);
    ui->PLOT_VEL_HIP->yAxis->setRange(-1, 1);
    ui->PLOT_VEL_KNEE->yAxis->setRange(-1, 1);
    ui->PLOT_TORQUE_HIP->yAxis->setRange(-1, 1);
    ui->PLOT_TORQUE_KNEE->yAxis->setRange(-1, 1);

    ui->PLOT_CUSTOM_1->xAxis->setTicker(timeTicker);
    ui->PLOT_CUSTOM_2->xAxis->setTicker(timeTicker);
    ui->PLOT_CUSTOM_3->xAxis->setTicker(timeTicker);
    ui->PLOT_CUSTOM_4->xAxis->setTicker(timeTicker);
    ui->PLOT_CUSTOM_5->xAxis->setTicker(timeTicker);
    ui->PLOT_CUSTOM_6->xAxis->setTicker(timeTicker);

    ui->PLOT_CUSTOM_1->yAxis->setRange(-1, 1);
    ui->PLOT_CUSTOM_2->yAxis->setRange(-1, 1);
    ui->PLOT_CUSTOM_3->yAxis->setRange(-1, 1);
    ui->PLOT_CUSTOM_4->yAxis->setRange(-1, 1);
    ui->PLOT_CUSTOM_5->yAxis->setRange(-1, 1);
    ui->PLOT_CUSTOM_6->yAxis->setRange(-1, 1);


//    ui->plot_custom->xAxis->setTicker(timeTicker);
//    ui->plot_custom->axisRect()->setupFullAxesBox();
//    ui->plot_custom->yAxis->setRange(-1, 1);
//    connect(ui->plot_custom->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_custom_2->xAxis2, SLOT(setRange(QCPRange)));
//    connect(ui->plot_custom->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_custom_2->yAxis2, SLOT(setRange(QCPRange)));

//    ui->plot_custom_2->xAxis->setTicker(timeTicker);
//    ui->plot_custom_2->axisRect()->setupFullAxesBox();
//    ui->plot_custom_2->yAxis->setRange(-1, 1);
//    connect(ui->plot_custom_2->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_custom_2->xAxis2, SLOT(setRange(QCPRange)));
//    connect(ui->plot_custom_2->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_custom_2->yAxis2, SLOT(setRange(QCPRange)));

//    ui->plot_custom_3->xAxis->setTicker(timeTicker);
//    ui->plot_custom_3->axisRect()->setupFullAxesBox();
//    ui->plot_custom_3->yAxis->setRange(-1, 1);
//    connect(ui->plot_custom_3->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_custom_3->xAxis2, SLOT(setRange(QCPRange)));
//    connect(ui->plot_custom_3->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_custom_3->yAxis2, SLOT(setRange(QCPRange)));

}

void MainWindow::GraphUpdate()
{
    ui->PLOT_POS_HIP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[HIP_IDX]);
    ui->PLOT_POS_KNEE->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorPosition[KNEE_IDX]);
    ui->PLOT_VEL_HIP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[HIP_IDX]);
    ui->PLOT_VEL_KNEE->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[KNEE_IDX]);
    ui->PLOT_TORQUE_HIP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[HIP_IDX]);
    ui->PLOT_TORQUE_HIP->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorTorque[HIP_IDX]);
    ui->PLOT_TORQUE_KNEE->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[KNEE_IDX]);
    ui->PLOT_TORQUE_KNEE->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorTorque[KNEE_IDX]);

    ui->PLOT_POS_HIP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_POS_KNEE->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_HIP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_KNEE->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_TORQUE_HIP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_TORQUE_KNEE->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);

    ui->PLOT_POS_HIP->replot();
    ui->PLOT_POS_KNEE->replot();
    ui->PLOT_VEL_HIP->replot();
    ui->PLOT_VEL_KNEE->replot();
    ui->PLOT_TORQUE_HIP->replot();
    ui->PLOT_TORQUE_KNEE->replot();
}

void MainWindow::DisplayUpdate()
{
    if(sharedMemory->canStatus)
    {
        ui->LE_CAN_STATUS->setStyleSheet("background-color:lightgreen");
    }
    else
    {
        ui->LE_CAN_STATUS->setStyleSheet("background-color:red");
    }

    ui->TW_MOTOR->item(0, 0)->setText(QString().sprintf("id : 0x%x", MOTOR_HIP_ID));
    ui->TW_MOTOR->item(1, 0)->setText(QString().sprintf("id : 0x%x", MOTOR_KNEE_ID));
    for(int index = 0; index < MOTOR_NUM; index++)
    {
        if(sharedMemory->motorStatus)
        {
            ui->TW_MOTOR->item(index, 0)->setBackgroundColor(QColor(75, 75, 255));
        }
        else
        {
            ui->TW_MOTOR->item(index, 0)->setBackgroundColor(QColor(255, 75, 75));
        }

        ui->TW_MOTOR->item(index, 1)->setText(QString().sprintf("%d", sharedMemory->motorErrorStatus[index]));
        if(sharedMemory->motorErrorStatus[index] != 0)
        {
            ui->TW_MOTOR->item(index, 1)->setBackgroundColor(QColor(255, 100, 100));
        }
        else
        {
            ui->TW_MOTOR->item(index, 1)->setBackgroundColor(QColor(100, 255, 100));
        }
        ui->TW_MOTOR->item(index, 2)->setText(QString().sprintf("%d", sharedMemory->motorTemp[index]));
        ui->TW_MOTOR->item(index, 3)->setText(QString().sprintf("%.1f", sharedMemory->motorVoltage[index]));
        ui->TW_MOTOR->item(index, 4)->setText(QString().sprintf("%.1f", sharedMemory->motorPosition[index] * R2D));
        ui->TW_MOTOR->item(index, 5)->setText(QString().sprintf("%.1f", sharedMemory->motorDesiredTorque[index]));
        ui->TW_MOTOR->item(index, 6)->setText(QString().sprintf("%.1f", sharedMemory->motorTorque[index]));
    }
}

void MainWindow::on_BT_CAN_ON_clicked()
{
    sharedCommand->userCommand = CAN_ON;
    sharedMemory->newCommand = true;
}

void MainWindow::on_BT_VISUAL_ON_clicked()
{
    ui->LE_VISUAL_STATUS->setStyleSheet("background-color:lightgreen");
    sharedCommand->userCommand = VISUAL_ON;
    sharedMemory->newCommand = true;
}

void MainWindow::on_BT_MOTOR_ON_clicked()
{
    ui->LE_MOTOR_ON_STATUS->setStyleSheet("background-color:lightgreen");
    ui->LE_MOTOR_OFF_STATUS->setStyleSheet("background-color:red");
    sharedCommand->userCommand = MOTOR_ON;
    sharedMemory->newCommand = true;
}

void MainWindow::on_BT_MOTOR_OFF_clicked()
{
    ui->LE_MOTOR_OFF_STATUS->setStyleSheet("background-color:lightgreen");
    ui->LE_MOTOR_ON_STATUS->setStyleSheet("background-color:red");
    ui->LE_HOME->setStyleSheet("background-color:pink");
    ui->LE_PD_CONTROL->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM1->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM2->setStyleSheet("background-color:pink");
    sharedCommand->userCommand = MOTOR_OFF;
    sharedMemory->newCommand = true;
}

void MainWindow::on_BT_HOME_clicked()
{
    ui->LE_HOME->setStyleSheet("background-color:lightblue");
    ui->LE_PD_CONTROL->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM1->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM2->setStyleSheet("background-color:pink");
    sharedCommand->userCommand = HOME;
    sharedMemory->newCommand = true;
}

void MainWindow::on_BT_PD_CONTROL_clicked()
{
    ui->LE_PD_CONTROL->setStyleSheet("background-color:lightblue");
    ui->LE_HOME->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM1->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM2->setStyleSheet("background-color:pink");
    sharedCommand->userCommand = PD_CMD;
    sharedMemory->newCommand = true;
}


void MainWindow::on_BT_CUSTOM1_clicked()
{
    ui->LE_CUSTOM1->setStyleSheet("background-color:lightblue");
    ui->LE_HOME->setStyleSheet("background-color:pink");
    ui->LE_PD_CONTROL->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM2->setStyleSheet("background-color:pink");
    sharedCommand->userCommand = CUSTOM_1;
    sharedMemory->newCommand = true;
}

void MainWindow::on_BT_CUSTOM2_clicked()
{
    ui->LE_CUSTOM2->setStyleSheet("background-color:lightblue");
    ui->LE_HOME->setStyleSheet("background-color:pink");
    ui->LE_PD_CONTROL->setStyleSheet("background-color:pink");
    ui->LE_CUSTOM1->setStyleSheet("background-color:pink");
    sharedCommand->userCommand = CUSTOM_2;
    sharedMemory->newCommand = true;
}