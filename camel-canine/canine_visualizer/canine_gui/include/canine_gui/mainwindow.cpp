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
    const int row_num = 12;
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

    table->horizontalHeaderItem(0)->setText("Motor ID");
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
    table->verticalHeaderItem(0)->setText("LFHR");
    table->verticalHeaderItem(1)->setText("LFHP");
    table->verticalHeaderItem(2)->setText("LFKP");
    table->verticalHeaderItem(3)->setText("RFHR");
    table->verticalHeaderItem(4)->setText("RFHP");
    table->verticalHeaderItem(5)->setText("RFKP");
    table->verticalHeaderItem(6)->setText("LBHR");
    table->verticalHeaderItem(7)->setText("LBHP");
    table->verticalHeaderItem(8)->setText("LBKP");
    table->verticalHeaderItem(9)->setText("RBHR");
    table->verticalHeaderItem(10)->setText("RBHP");
    table->verticalHeaderItem(11)->setText("RBKP");


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

    ui->PLOT_POS_BASE_X->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_POS_BASE_Y->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_POS_BASE_Z->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_BASE_X->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_BASE_Y->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_BASE_Z->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->PLOT_POS_BASE_ROLL->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_POS_BASE_PITCH->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_POS_BASE_YAW->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_BASE_ROLL->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_BASE_PITCH->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_BASE_YAW->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->PLOT_VEL_LFHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_LFHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_LFKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_RFHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_RFHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_RFKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->PLOT_VEL_LBHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_LBHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_LBKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_RBHR->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_RBHP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_VEL_RBKP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);


    ui->PLOT_CUSTOM_1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_5->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->PLOT_CUSTOM_6->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    myPen.setWidthF(1.5);
    myPen.setColor(Qt::blue);
    ui->PLOT_POS_BASE_X->legend->setVisible(true);
    ui->PLOT_POS_BASE_X->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_POS_BASE_X->addGraph();
    ui->PLOT_POS_BASE_X->graph(0)->setPen(myPen);
    ui->PLOT_POS_BASE_X->graph(0)->setName("position x");

    ui->PLOT_POS_BASE_Y->legend->setVisible(true);
    ui->PLOT_POS_BASE_Y->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_POS_BASE_Y->addGraph();
    ui->PLOT_POS_BASE_Y->graph(0)->setPen(myPen);
    ui->PLOT_POS_BASE_Y->graph(0)->setName("position y");

    ui->PLOT_POS_BASE_Z->legend->setVisible(true);
    ui->PLOT_POS_BASE_Z->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_POS_BASE_Z->addGraph();
    ui->PLOT_POS_BASE_Z->graph(0)->setPen(myPen);
    ui->PLOT_POS_BASE_Z->graph(0)->setName("position z");

    ui->PLOT_VEL_BASE_X->legend->setVisible(true);
    ui->PLOT_VEL_BASE_X->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_BASE_X->addGraph();
    ui->PLOT_VEL_BASE_X->graph(0)->setPen(myPen);
    ui->PLOT_VEL_BASE_X->graph(0)->setName("velocity x");

    ui->PLOT_VEL_BASE_Y->legend->setVisible(true);
    ui->PLOT_VEL_BASE_Y->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_BASE_Y->addGraph();
    ui->PLOT_VEL_BASE_Y->graph(0)->setPen(myPen);
    ui->PLOT_VEL_BASE_Y->graph(0)->setName("velocity y");

    ui->PLOT_VEL_BASE_Z->legend->setVisible(true);
    ui->PLOT_VEL_BASE_Z->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_BASE_Z->addGraph();
    ui->PLOT_VEL_BASE_Z->graph(0)->setPen(myPen);
    ui->PLOT_VEL_BASE_Z->graph(0)->setName("velocity z");

    ui->PLOT_POS_BASE_ROLL->legend->setVisible(true);
    ui->PLOT_POS_BASE_ROLL->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_POS_BASE_ROLL->addGraph();
    ui->PLOT_POS_BASE_ROLL->graph(0)->setPen(myPen);
    ui->PLOT_POS_BASE_ROLL->graph(0)->setName("position roll");

    ui->PLOT_POS_BASE_PITCH->legend->setVisible(true);
    ui->PLOT_POS_BASE_PITCH->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_POS_BASE_PITCH->addGraph();
    ui->PLOT_POS_BASE_PITCH->graph(0)->setPen(myPen);
    ui->PLOT_POS_BASE_PITCH->graph(0)->setName("position pitch");

    ui->PLOT_POS_BASE_YAW->legend->setVisible(true);
    ui->PLOT_POS_BASE_YAW->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_POS_BASE_YAW->addGraph();
    ui->PLOT_POS_BASE_YAW->graph(0)->setPen(myPen);
    ui->PLOT_POS_BASE_YAW->graph(0)->setName("position yaw");

    ui->PLOT_VEL_BASE_ROLL->legend->setVisible(true);
    ui->PLOT_VEL_BASE_ROLL->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_BASE_ROLL->addGraph();
    ui->PLOT_VEL_BASE_ROLL->graph(0)->setPen(myPen);
    ui->PLOT_VEL_BASE_ROLL->graph(0)->setName("velocity roll");

    ui->PLOT_VEL_BASE_PITCH->legend->setVisible(true);
    ui->PLOT_VEL_BASE_PITCH->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_BASE_PITCH->addGraph();
    ui->PLOT_VEL_BASE_PITCH->graph(0)->setPen(myPen);
    ui->PLOT_VEL_BASE_PITCH->graph(0)->setName("velocity pitch");

    ui->PLOT_VEL_BASE_YAW->legend->setVisible(true);
    ui->PLOT_VEL_BASE_YAW->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_BASE_YAW->addGraph();
    ui->PLOT_VEL_BASE_YAW->graph(0)->setPen(myPen);
    ui->PLOT_VEL_BASE_YAW->graph(0)->setName("velocity yaw");

    ui->PLOT_VEL_LFHR->legend->setVisible(true);
    ui->PLOT_VEL_LFHR->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_LFHR->addGraph();
    ui->PLOT_VEL_LFHR->graph(0)->setPen(myPen);
    ui->PLOT_VEL_LFHR->graph(0)->setName("velocity LFHR");

    ui->PLOT_VEL_LFHP->legend->setVisible(true);
    ui->PLOT_VEL_LFHP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_LFHP->addGraph();
    ui->PLOT_VEL_LFHP->graph(0)->setPen(myPen);
    ui->PLOT_VEL_LFHP->graph(0)->setName("velocity LFHP");

    ui->PLOT_VEL_LFKP->legend->setVisible(true);
    ui->PLOT_VEL_LFKP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_LFKP->addGraph();
    ui->PLOT_VEL_LFKP->graph(0)->setPen(myPen);
    ui->PLOT_VEL_LFKP->graph(0)->setName("velocity LFKP");

    ui->PLOT_VEL_RFHR->legend->setVisible(true);
    ui->PLOT_VEL_RFHR->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_RFHR->addGraph();
    ui->PLOT_VEL_RFHR->graph(0)->setPen(myPen);
    ui->PLOT_VEL_RFHR->graph(0)->setName("velocity RFHR");

    ui->PLOT_VEL_RFHP->legend->setVisible(true);
    ui->PLOT_VEL_RFHP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_RFHP->addGraph();
    ui->PLOT_VEL_RFHP->graph(0)->setPen(myPen);
    ui->PLOT_VEL_RFHP->graph(0)->setName("velocity RFHP");

    ui->PLOT_VEL_RFKP->legend->setVisible(true);
    ui->PLOT_VEL_RFKP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_RFKP->addGraph();
    ui->PLOT_VEL_RFKP->graph(0)->setPen(myPen);
    ui->PLOT_VEL_RFKP->graph(0)->setName("velocity RFKP");

    ui->PLOT_VEL_LBHR->legend->setVisible(true);
    ui->PLOT_VEL_LBHR->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_LBHR->addGraph();
    ui->PLOT_VEL_LBHR->graph(0)->setPen(myPen);
    ui->PLOT_VEL_LBHR->graph(0)->setName("velocity LBHR");

    ui->PLOT_VEL_LBHP->legend->setVisible(true);
    ui->PLOT_VEL_LBHP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_LBHP->addGraph();
    ui->PLOT_VEL_LBHP->graph(0)->setPen(myPen);
    ui->PLOT_VEL_LBHP->graph(0)->setName("velocity LBHP");

    ui->PLOT_VEL_LBKP->legend->setVisible(true);
    ui->PLOT_VEL_LBKP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_LBKP->addGraph();
    ui->PLOT_VEL_LBKP->graph(0)->setPen(myPen);
    ui->PLOT_VEL_LBKP->graph(0)->setName("velocity LBKP");

    ui->PLOT_VEL_RBHR->legend->setVisible(true);
    ui->PLOT_VEL_RBHR->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_RBHR->addGraph();
    ui->PLOT_VEL_RBHR->graph(0)->setPen(myPen);
    ui->PLOT_VEL_RBHR->graph(0)->setName("velocity RBHR");

    ui->PLOT_VEL_RBHP->legend->setVisible(true);
    ui->PLOT_VEL_RBHP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_RBHP->addGraph();
    ui->PLOT_VEL_RBHP->graph(0)->setPen(myPen);
    ui->PLOT_VEL_RBHP->graph(0)->setName("velocity RBHP");

    ui->PLOT_VEL_RBKP->legend->setVisible(true);
    ui->PLOT_VEL_RBKP->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_VEL_RBKP->addGraph();
    ui->PLOT_VEL_RBKP->graph(0)->setPen(myPen);
    ui->PLOT_VEL_RBKP->graph(0)->setName("velocity RBKP");

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

    ui->PLOT_CUSTOM_1->legend->setVisible(true);
    ui->PLOT_CUSTOM_1->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_CUSTOM_1->graph(0)->setName("tau LBHR");
    ui->PLOT_CUSTOM_1->graph(1)->setName("des_tau LBHR");
    ui->PLOT_CUSTOM_3->legend->setVisible(true);
    ui->PLOT_CUSTOM_3->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_CUSTOM_3->graph(0)->setName("tau LBHP");
    ui->PLOT_CUSTOM_3->graph(1)->setName("des_tau LBHP");
    ui->PLOT_CUSTOM_5->legend->setVisible(true);
    ui->PLOT_CUSTOM_5->legend->setFont(QFont("Helvetica", 9));
    ui->PLOT_CUSTOM_5->graph(0)->setName("tau LBKP");
    ui->PLOT_CUSTOM_5->graph(1)->setName("des_tau LBKP");

    ui->PLOT_POS_BASE_X->xAxis->setTicker(timeTicker);
    ui->PLOT_POS_BASE_Y->xAxis->setTicker(timeTicker);
    ui->PLOT_POS_BASE_Z->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_BASE_X->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_BASE_Y->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_BASE_Z->xAxis->setTicker(timeTicker);

    ui->PLOT_POS_BASE_ROLL->xAxis->setTicker(timeTicker);
    ui->PLOT_POS_BASE_PITCH->xAxis->setTicker(timeTicker);
    ui->PLOT_POS_BASE_YAW->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_BASE_ROLL->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_BASE_PITCH->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_BASE_YAW->xAxis->setTicker(timeTicker);

    ui->PLOT_VEL_LFHR->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_LFHP->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_LFKP->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_RFHR->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_RFHP->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_RFKP->xAxis->setTicker(timeTicker);

    ui->PLOT_VEL_LBHR->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_LBHP->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_LBKP->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_RBHR->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_RBHP->xAxis->setTicker(timeTicker);
    ui->PLOT_VEL_RBKP->xAxis->setTicker(timeTicker);

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
    ui->PLOT_POS_BASE_X->graph(0)->addData(sharedMemory->localTime, sharedMemory->basePosition[0]);
    ui->PLOT_POS_BASE_Y->graph(0)->addData(sharedMemory->localTime, sharedMemory->basePosition[1]);
    ui->PLOT_POS_BASE_Z->graph(0)->addData(sharedMemory->localTime, sharedMemory->basePosition[2]);
    ui->PLOT_VEL_BASE_X->graph(0)->addData(sharedMemory->localTime, sharedMemory->baseVelocity[0]);
    ui->PLOT_VEL_BASE_Y->graph(0)->addData(sharedMemory->localTime, sharedMemory->baseVelocity[1]);
    ui->PLOT_VEL_BASE_Z->graph(0)->addData(sharedMemory->localTime, sharedMemory->baseVelocity[2]);

    ui->PLOT_POS_BASE_ROLL->graph(0)->addData(sharedMemory->localTime, sharedMemory->baseEulerPosition[0]);
    ui->PLOT_POS_BASE_PITCH->graph(0)->addData(sharedMemory->localTime, sharedMemory->baseEulerPosition[1]);
    ui->PLOT_POS_BASE_YAW->graph(0)->addData(sharedMemory->localTime, sharedMemory->baseEulerPosition[2]);
    ui->PLOT_VEL_BASE_ROLL->graph(0)->addData(sharedMemory->localTime, sharedMemory->baseEulerVelocity[0]);
    ui->PLOT_VEL_BASE_PITCH->graph(0)->addData(sharedMemory->localTime, sharedMemory->baseEulerVelocity[1]);
    ui->PLOT_VEL_BASE_YAW->graph(0)->addData(sharedMemory->localTime, sharedMemory->baseEulerVelocity[2]);

    ui->PLOT_VEL_LFHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[LFHR_IDX]);
    ui->PLOT_VEL_LFHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[LFHP_IDX]);
    ui->PLOT_VEL_LFKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[LFKP_IDX]);
    ui->PLOT_VEL_RFHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[RFHR_IDX]);
    ui->PLOT_VEL_RFHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[RFHP_IDX]);
    ui->PLOT_VEL_RFKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[RFKP_IDX]);

    ui->PLOT_VEL_LBHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[LBHR_IDX]);
    ui->PLOT_VEL_LBHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[LBHP_IDX]);
    ui->PLOT_VEL_LBKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[LBKP_IDX]);
    ui->PLOT_VEL_RBHR->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[RBHR_IDX]);
    ui->PLOT_VEL_RBHP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[RBHP_IDX]);
    ui->PLOT_VEL_RBKP->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorVelocity[RBKP_IDX]);

    ui->PLOT_CUSTOM_1->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[LBHR_IDX]);
    ui->PLOT_CUSTOM_3->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[LBHP_IDX]);
    ui->PLOT_CUSTOM_5->graph(0)->addData(sharedMemory->localTime, sharedMemory->motorTorque[LBKP_IDX]);
    ui->PLOT_CUSTOM_1->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[LBHR_IDX]);
    ui->PLOT_CUSTOM_3->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[LBHP_IDX]);
    ui->PLOT_CUSTOM_5->graph(1)->addData(sharedMemory->localTime, sharedMemory->motorDesiredTorque[LBKP_IDX]);

    ui->PLOT_POS_BASE_X->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_POS_BASE_Y->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_POS_BASE_Z->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_BASE_X->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_BASE_Y->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_BASE_Z->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);

    ui->PLOT_POS_BASE_ROLL->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_POS_BASE_PITCH->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_POS_BASE_YAW->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_BASE_ROLL->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_BASE_PITCH->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_BASE_YAW->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);

    ui->PLOT_VEL_LFHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_LFHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_LFKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_RFHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_RFHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_RFKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);

    ui->PLOT_VEL_LBHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_LBHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_LBKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_RBHR->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_RBHP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_VEL_RBKP->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);

    ui->PLOT_CUSTOM_1->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_CUSTOM_3->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);
    ui->PLOT_CUSTOM_5->xAxis->setRange(sharedMemory->localTime - graphOffset, sharedMemory->localTime + graphOffset);

    ui->PLOT_POS_BASE_X->replot();
    ui->PLOT_POS_BASE_Y->replot();
    ui->PLOT_POS_BASE_Z->replot();
    ui->PLOT_VEL_BASE_X->replot();
    ui->PLOT_VEL_BASE_Y->replot();
    ui->PLOT_VEL_BASE_Z->replot();

    ui->PLOT_POS_BASE_ROLL->replot();
    ui->PLOT_POS_BASE_PITCH->replot();
    ui->PLOT_POS_BASE_YAW->replot();
    ui->PLOT_VEL_BASE_ROLL->replot();
    ui->PLOT_VEL_BASE_PITCH->replot();
    ui->PLOT_VEL_BASE_YAW->replot();

    ui->PLOT_VEL_LFHR->replot();
    ui->PLOT_VEL_LFHP->replot();
    ui->PLOT_VEL_LFKP->replot();
    ui->PLOT_VEL_RFHR->replot();
    ui->PLOT_VEL_RFHP->replot();
    ui->PLOT_VEL_RFKP->replot();

    ui->PLOT_VEL_LBHR->replot();
    ui->PLOT_VEL_LBHP->replot();
    ui->PLOT_VEL_LBKP->replot();
    ui->PLOT_VEL_RBHR->replot();
    ui->PLOT_VEL_RBHP->replot();
    ui->PLOT_VEL_RBKP->replot();

    ui->PLOT_CUSTOM_1->replot();
    ui->PLOT_CUSTOM_3->replot();
    ui->PLOT_CUSTOM_5->replot();
}

void MainWindow::DisplayUpdate()
{
    if((sharedMemory->can1Status) && (sharedMemory->can2Status))
    {
        ui->LE_CAN_STATUS->setStyleSheet("background-color:lightgreen");
    }
    else
    {
        ui->LE_CAN_STATUS->setStyleSheet("background-color:red");
    }

    ui->TW_MOTOR->item(0, 0)->setText(QString().sprintf("0x%x", MOTOR_LFHR_ID));
    ui->TW_MOTOR->item(1, 0)->setText(QString().sprintf("0x%x", MOTOR_LFHP_ID));
    ui->TW_MOTOR->item(2, 0)->setText(QString().sprintf("0x%x", MOTOR_LFKP_ID));
    ui->TW_MOTOR->item(3, 0)->setText(QString().sprintf("0x%x", MOTOR_RFHR_ID));
    ui->TW_MOTOR->item(4, 0)->setText(QString().sprintf("0x%x", MOTOR_RFHP_ID));
    ui->TW_MOTOR->item(5, 0)->setText(QString().sprintf("0x%x", MOTOR_RFKP_ID));
    ui->TW_MOTOR->item(6, 0)->setText(QString().sprintf("0x%x", MOTOR_LBHR_ID));
    ui->TW_MOTOR->item(7, 0)->setText(QString().sprintf("0x%x", MOTOR_LBHP_ID));
    ui->TW_MOTOR->item(8, 0)->setText(QString().sprintf("0x%x", MOTOR_LBKP_ID));
    ui->TW_MOTOR->item(9, 0)->setText(QString().sprintf("0x%x", MOTOR_RBHR_ID));
    ui->TW_MOTOR->item(10, 0)->setText(QString().sprintf("0x%x", MOTOR_RBHP_ID));
    ui->TW_MOTOR->item(11, 0)->setText(QString().sprintf("0x%x", MOTOR_RBKP_ID));
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