//
// Created by hs on 22. 10. 27.
//

#include <marten-leg_gui/mainwindow.h>
#include <marten-leg_simulation/SimulMain.hpp>

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    StartSimulation();

    w.show();
    return a.exec();
}