//
// Created by hs on 22. 10. 27.
//

#include <canine-leg-left_gui/mainwindow.h>
#include <canine-leg-left_simulation/SimulMain.hpp>

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    StartSimulation();

    w.show();
    return a.exec();
}