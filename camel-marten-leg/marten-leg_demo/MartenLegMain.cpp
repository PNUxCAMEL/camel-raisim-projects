//
// Created by jh on 22. 11. 21.
//

#include <marten-leg_gui/mainwindow.h>
#include <marten-leg_fsm/MainFSM.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    StartFSM();

    w.show();
    return a.exec();
}