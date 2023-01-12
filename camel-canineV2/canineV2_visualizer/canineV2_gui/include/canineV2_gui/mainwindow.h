#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidget>
#include <QTimer>
#include <QPen>
#include "canineV2_util/SharedMemory.hpp"
#include "canineV2_util/RobotDescription.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void DisplayUpdate();
    void GraphUpdate();
    void GraphInitialize();
    void on_BT_MOTOR_ON_clicked();
    void on_BT_MOTOR_OFF_clicked();
    void on_BT_CAN_ON_clicked();
    void on_BT_VISUAL_ON_clicked();
    void on_BT_HOME_clicked();
    void on_BT_PD_CONTROL_clicked();
    void on_BT_CUSTOM1_clicked();
    void on_BT_CUSTOM2_clicked();

private:
    Ui::MainWindow *ui;
    QTimer      *displayTimer;
    QTimer		*graphTimer;
    double graphOffset;
    void InitTable(QTableWidget *table);
    void InitLineEdit();
};
#endif // MAINWINDOW_H
