//
// Created by hs on 22. 10. 27.
//

#include <canine_simulation/SimulMain.hpp>

pthread_t RTThreadController;
pthread_t RTThreadStateEstimator;
pthread_t NRTThreadCommand;
pthread_t NRTThreadIMU;

pUI_COMMAND sharedCommand;
pSHM sharedMemory;

raisim::World world;
raisim::RaisimServer server(&world);
raisim::ArticulatedSystem* robot = world.addArticulatedSystem(std::string(URDF_RSC_DIR)+"/canine/urdf/canineV1.urdf");

SimulCommand userCommand;
SimulVisualizer Visualizer(&world, robot, &server);
SimulControlPanel ControlPanel(&world, robot);
SimulStateEstimator StateEstimator(robot);
T265 TrackingCam;

void* NRTCommandThread(void* arg)
{
    std::cout << "entered #Command_NRT_thread" << std::endl;
    while (true)
    {
        userCommand.commandFunction();
        usleep(CMD_dT * 1e6);
    }
}

void* NRTImuThread(void* arg)
{
    std::cout << "entered #Command_NRTImuThread" << std::endl;
    std::string serial;
    rs2::pipeline pipe;
    rs2::config cfg;
    if(!serial.empty())
        cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_POSE,RS2_FORMAT_6DOF);
    auto callback = [&](const rs2::frame& frame) {
        if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
            rs2_pose pose_data = fp.get_pose_data();
            auto now = std::chrono::system_clock::now().time_since_epoch();
            double now_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
            double pose_time_ms = fp.get_timestamp();
            float dt_s = static_cast<float>(std::max(0., (now_ms - pose_time_ms) / 1000.));
            rs2_pose predicted_pose = TrackingCam.predict_pose(pose_data, dt_s);
            sharedMemory->baseQuartPosition[0];
            sharedMemory->baseQuartPosition[1];
            sharedMemory->baseQuartPosition[2];
            sharedMemory->baseQuartPosition[3];
            std::cout << predicted_pose.velocity.x << std::endl;
        }
    };
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void *RTControllerThread(void *arg)
{
    std::cout << "entered #Controller_RT_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(CONTROL_dT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;

    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            ControlPanel.ControllerFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, Controller thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }

    }
}

void* RTStateEstimator(void* arg)
{
    std::cout << "entered #StateEsitimator_RT_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(ESTIMATOR_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        if (sharedMemory->visualState != STATE_VISUAL_STOP)
        {
            StateEstimator.StateEstimatorFunction();
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "RT Deadline Miss, State estimator thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void clearSharedMemory()
{
    sharedMemory->newCommand = false;
    sharedMemory->can1Status = false;
    sharedMemory->can2Status = false;
    sharedMemory->motorStatus = false;
    sharedMemory->controlState = STATE_CONTROL_STOP;
    sharedMemory->visualState = STATE_VISUAL_STOP;
    sharedMemory->gaitState = STAND;
    sharedMemory->can1State = CAN_NO_ACT;
    sharedMemory->can2State = CAN_NO_ACT;
    sharedMemory->localTime = 0;
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        sharedMemory->motorErrorStatus[index] = 0;
        sharedMemory->motorTemp[index] = 0;
        sharedMemory->motorPosition[index] = 0;
        sharedMemory->motorVelocity[index] = 0;
        sharedMemory->motorTorque[index] = 0;
        sharedMemory->motorDesiredTorque[index] = 0;
        sharedMemory->motorVoltage[index] = 0;
    }
    sharedMemory->basePosition.setZero();
    sharedMemory->baseVelocity.setZero();
    for (int index = 0; index < 3; index++)
    {
        sharedMemory->baseEulerPosition[index] = 0;
        sharedMemory->baseEulerVelocity[index] = 0;
    }

    sharedMemory->baseQuartPosition[0] = 1.0;
    sharedMemory->baseQuartPosition[1] = 0.0;
    sharedMemory->baseQuartPosition[2] = 0.0;
    sharedMemory->baseQuartPosition[3] = 0.0;

    sharedMemory->gaitIteration = 0;
}

void StartSimulation()
{
    sharedCommand = (pUI_COMMAND)malloc(sizeof(UI_COMMAND));
    sharedMemory = (pSHM)malloc(sizeof(SHM));
    clearSharedMemory();

    server.launchServer(8080);

    int thread_id_rt1 = generate_rt_thread(RTThreadController, RTControllerThread, "rt_thread1", 6, 99, NULL);
    int thread_id_rt2 = generate_rt_thread(RTThreadStateEstimator, RTStateEstimator, "rt_thread2", 7, 99,NULL);

    int thread_id_nrt1 = generate_nrt_thread(NRTThreadCommand, NRTCommandThread, "nrt_thread1", 1, NULL);
    int thread_id_nrt3 = generate_nrt_thread(NRTThreadIMU, NRTImuThread, "nrt_thread3", 2, NULL);

}