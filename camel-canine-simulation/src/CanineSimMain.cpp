
#include "CanineSimMain.hpp"

void realTimePlot()
{
    sharedMemory->simTime = 0;
    sharedMemory->PositionHip = 0;
    sharedMemory->PositionKnee = 0;
    sharedMemory->PositionBase = 0;
    sharedMemory->DesiredPositionHip = 0;
    sharedMemory->DesiredPositionKnee = 0;
    sharedMemory->DesiredPositionBase = 0;
    sharedMemory->VelocityHip = 0;
    sharedMemory->VelocityKnee = 0;
    sharedMemory->VelocityBase = 0;
    sharedMemory->DesiredVelocityHip = 0;
    sharedMemory->DesiredVelocityKnee = 0;
    sharedMemory->DesiredVelocityBase = 0;
    sharedMemory->TorqueHip = 0;
    sharedMemory->TorqueKnee = 0;
    sharedMemory->DesiredTorqueHip = 0;
    sharedMemory->DesiredTorqueKnee = 0;
}

void *rt_simulation_thread(void *arg) {
    std::cout << "entered #rt_simulation_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(dT * 1e6);

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        realTimePlot();
//        raisimSimulation();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
            std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                << " ms" << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    sharedMemory = (pSHM) malloc(sizeof(SHM));
    int thread_id_simulation = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 1, 99,
        NULL);
    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    w.show();

    return a.exec();
}