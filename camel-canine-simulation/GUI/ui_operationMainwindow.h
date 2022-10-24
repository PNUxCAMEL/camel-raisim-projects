
#ifndef RAISIM_UI_SIMULATIONMAINWINDOW_H
#define RAISIM_UI_SIMULATIONMAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *pushButton;
    QCustomPlot *widget_pos0;
    QCustomPlot *widget_pos1;
    QCustomPlot *widget_pos2;
    QCustomPlot *widget_vel0;
    QCustomPlot *widget_vel1;
    QCustomPlot *widget_vel2;
    QCustomPlot *widget_tor1;
    QCustomPlot *widget_tor2;
    QLabel *kneeTorMin;
    QLabel *kneeTorMax;
    QLabel *kneeTorMinValue;
    QLabel *kneeTorMaxValue;
    QLabel *hipTorMin;
    QLabel *hipTorMax;
    QLabel *hipTorMinValue;
    QLabel *hipTorMaxValue;

    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(972, 972);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(840, 610, 121, 51));

        kneeTorMin = new QLabel(centralWidget);
        kneeTorMin->setObjectName(QString::fromUtf8("kneeTorMin"));
        kneeTorMin->setGeometry(QRect(840,690,131,17));

        kneeTorMinValue = new QLabel(centralWidget);
        kneeTorMinValue->setObjectName(QString::fromUtf8("kneeTorMinValue"));
        kneeTorMinValue->setGeometry(QRect(840,710,91,17));

        kneeTorMax = new QLabel(centralWidget);
        kneeTorMax->setObjectName(QString::fromUtf8("kneeTorMax"));
        kneeTorMax->setGeometry(QRect(840,730,131,17));

        kneeTorMaxValue = new QLabel(centralWidget);
        kneeTorMaxValue->setObjectName(QString::fromUtf8("kneeTorMaxValue"));
        kneeTorMaxValue->setGeometry(QRect(840,750,91,17));

        hipTorMin = new QLabel(centralWidget);
        hipTorMin->setObjectName(QString::fromUtf8("hipTorMin"));
        hipTorMin->setGeometry(QRect(840,780,131,17));

        hipTorMinValue = new QLabel(centralWidget);
        hipTorMinValue->setObjectName(QString::fromUtf8("hipTorMinValue"));
        hipTorMinValue->setGeometry(QRect(840,800,91,17));

        hipTorMax = new QLabel(centralWidget);
        hipTorMax->setObjectName(QString::fromUtf8("hipTorMax"));
        hipTorMax->setGeometry(QRect(840,820,131,17));

        hipTorMaxValue = new QLabel(centralWidget);
        hipTorMaxValue->setObjectName(QString::fromUtf8("hipTorMaxValue"));
        hipTorMaxValue->setGeometry(QRect(840,840,91,17));

        widget_pos0 = new QCustomPlot(centralWidget);
        widget_pos0->setObjectName(QString::fromUtf8("widget_pos0"));
        widget_pos0->setGeometry(QRect(10, 10, 311, 291));

        widget_pos1 = new QCustomPlot(centralWidget);
        widget_pos1->setObjectName(QString::fromUtf8("widget_pos1"));
        widget_pos1->setGeometry(QRect(330, 10, 311, 291));

        widget_pos2 = new QCustomPlot(centralWidget);
        widget_pos2->setObjectName(QString::fromUtf8("widget_pos2"));
        widget_pos2->setGeometry(QRect(650, 10, 311, 291));

        widget_vel0 = new QCustomPlot(centralWidget);
        widget_vel0->setObjectName(QString::fromUtf8("widget_vel0"));
        widget_vel0->setGeometry(QRect(10, 310, 311, 291));

        widget_vel1 = new QCustomPlot(centralWidget);
        widget_vel1->setObjectName(QString::fromUtf8("widget_vel1"));
        widget_vel1->setGeometry(QRect(330, 310, 311, 291));

        widget_vel2 = new QCustomPlot(centralWidget);
        widget_vel2->setObjectName(QString::fromUtf8("widget_vel2"));
        widget_vel2->setGeometry(QRect(650, 310, 311, 291));

        widget_tor1 = new QCustomPlot(centralWidget);
        widget_tor1->setObjectName(QString::fromUtf8("widget_tor1"));
        widget_tor1->setGeometry(QRect(10, 610, 410, 291));

        widget_tor2 = new QCustomPlot(centralWidget);
        widget_tor2->setObjectName(QString::fromUtf8("widget_tor2"));
        widget_tor2->setGeometry(QRect(420, 610, 410, 291));

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 972, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        pushButton->setText(QApplication::translate("MainWindow", "Run", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE



#endif //RAISIM_UI_SIMULATIONMAINWINDOW_H
