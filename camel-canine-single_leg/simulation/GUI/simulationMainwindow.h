//
// Created by jaehoon on 22. 4. 26.
//

#ifndef RAISIM_SIMULATIONMAINWINDOW_H
#define RAISIM_SIMULATIONMAINWINDOW_H

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
    bool isSimulationEnd = true;
    double yMinWidget1;
    double yMaxWidget1;
    double yMinWidget2;
    double yMaxWidget2;
    double yMinWidget3;
    double yMaxWidget3;
    double intervalTime = 5.0;
    QTimer dataTimer;

public slots:
    void plotWidget1();
    void plotWidget2();
    void plotWidget3();
    void realtimeDataSlot();

private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
};

#endif //RAISIM_SIMULATIONMAINWINDOW_H
