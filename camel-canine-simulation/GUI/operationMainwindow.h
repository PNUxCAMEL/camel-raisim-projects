//
// Created by jaehoon on 22. 7. 19.
//

#ifndef RAISIM_OPERATIONMAINWINDOW_H
#define RAISIM_OPERATIONMAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    bool button1 = false;
    double yMinWidgetPos0;
    double yMaxWidgetPos0;
    double yMinWidgetPos1;
    double yMaxWidgetPos1;
    double yMinWidgetPos2;
    double yMaxWidgetPos2;
    double yMinWidgetVel0;
    double yMaxWidgetVel0;
    double yMinWidgetVel1;
    double yMaxWidgetVel1;
    double yMinWidgetVel2;
    double yMaxWidgetVel2;
    double yMinWidgetTor1;
    double yMaxWidgetTor1;
    double yMinWidgetTor2;
    double yMaxWidgetTor2;
    QTimer dataTimer;

public slots:
    void plotWidget_pos0();
    void plotWidget_pos1();
    void plotWidget_pos2();
    void plotWidget_vel0();
    void plotWidget_vel1();
    void plotWidget_vel2();
    void plotWidget_tor1();
    void plotWidget_tor2();
    void updateKneeTorMin();
    void updateKneeTorMax();
    void updateHipTorMin();
    void updateHipTorMax();
    void realtimeDataSlot();

private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
};


#endif //RAISIM_OPERATIONMAINWINDOW_H
