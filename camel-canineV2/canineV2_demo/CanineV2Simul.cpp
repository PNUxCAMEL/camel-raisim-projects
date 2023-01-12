//
// Created by hs on 22. 10. 27.
//

#include <canineV2_gui/mainwindow.h>
#include <canineV2_simulation/SimulMain.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    StartSimulation();

    w.show();
    return a.exec();
}