#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    bool button1 = false;
    double yMinWidget1;
    double yMaxWidget1;
    double yMinWidget2;
    double yMaxWidget2;
    double yMinWidget3;
    double yMaxWidget3;

    double yMinWidget4;
    double yMaxWidget4;
    double yMinWidget5;
    double yMaxWidget5;
    double yMinWidget6;
    double yMaxWidget6;

    double yMinWidget7;
    double yMaxWidget7;
    double xMinWidget7;
    double xMaxWidget7;

    double xMinWidget8;
    double xMaxWidget8;
    double yMinWidget8;
    double yMaxWidget8;

    double xMinWidget9;
    double xMaxWidget9;
    double yMinWidget9;
    double yMaxWidget9;

    QTimer dataTimer;
    int gaitIdx=0;
    bool gaitChanged=0;

public slots:
    void plotWidget1();
    void plotWidget2();
    void plotWidget3();

    void plotWidget4();
    void plotWidget5();
    void plotWidget6();

    void plotWidget7();
    void plotWidget8();
    void plotWidget9();
    void realtimeDataSlot();

private slots:
    void on_pushButton_clicked();
    void on_gaitBox_currentIndexChanged(int index);

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
