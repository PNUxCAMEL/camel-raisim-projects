//
// Created by hs on 22. 8. 21.
//

#include <canineV2_gui/mainwindow.h>
#include <canineV2_fsm/MainFSM.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    StartFSM();

    w.show();
    return a.exec();
}