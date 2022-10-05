#include "SingleLeggedSimulation.hpp"

extern MainWindow* MainUI;
pthread_t thread_simulation;
pSHM sharedMemory;

std::string urdfPath = "camel_single_leg_left/camel_single_leg.urdf";
std::string name = "single_leg";
raisim::World world;

double simulationDuration = 1.0;
double dT = 0.005;

double oneCycleSimTime = 0;
struct timespec TIME_TIC;
struct timespec TIME_TOC;

SingleLeggedSimulation sim = SingleLeggedSimulation(&world, dT);
SingleLeggedRobot robot = SingleLeggedRobot(&world, urdfPath, name);

//SingleLeggedPDController controller = SingleLeggedPDController(&robot, dT);
//SingleLeggedIDController controller = SingleLeggedIDController(&robot, dT);
SingleLeggedMPCController controller = SingleLeggedMPCController(&robot, dT, 7);

void resetSimVarialbes()
{
    oneCycleSimTime = 0;
}

void resetSharedMemory()
{
    sharedMemory->localTime = 0;
    sharedMemory->positionZ = 0;
    sharedMemory->desiredPositionZ = 0;
    sharedMemory->velocityZ = 0;
    sharedMemory->desiredVelocityZ = 0;
    sharedMemory->jointPosition[0] = 0;
    sharedMemory->jointVelocity[0] = 0;
    sharedMemory->jointTorque[0] = 0;
    sharedMemory->jointPosition[1] = 0;
    sharedMemory->jointVelocity[1] = 0;
    sharedMemory->jointTorque[1] = 0;
    sharedMemory->GRF = 0;
}

void raisimSimulation()
{
    if ((MainUI->button1) && (oneCycleSimTime < simulationDuration))
    {
        oneCycleSimTime += dT;
        clock_gettime(CLOCK_REALTIME, &TIME_TIC);
        controller.DoControl();
        clock_gettime(CLOCK_REALTIME, &TIME_TOC);
        world.integrate();
    }
    else if (oneCycleSimTime >= simulationDuration)
    {
        MainUI->button1 = false;
        MainUI->isSimulationEnd = true;
        resetSimVarialbes();
    }
}

void* rt_simulation_thread(void* arg)
{
    std::cout << "entered #rt_simulation_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;

    const long PERIOD_US = long(dT * 1e6);

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;
    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        raisimSimulation();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "RT Deadline Miss, Simulation thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                << " ms" << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    sharedMemory = (pSHM)malloc(sizeof(SHM));
    resetSharedMemory();
    sim.SetGroundProperty("wheat");
    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 7, 99,
        NULL);
    w.show();

    return a.exec();
}

