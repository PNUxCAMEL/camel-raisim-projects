#include "camel-tools/Sensor/RE22SC.hpp"

#include <QApplication>
#include <cmath>

#define DIVIDER 10
#define REFRESH 10
#define DT 0.005

extern MainWindow *MainUI;
pthread_t thread_plotting;
pthread_t thread_encoder;

double oneCyclePlotingTime = 0;
double Time = 0;
int iteration = 0;

RE22SC encoder;

void dataPloting() {
    if ((MainUI->button1) && (oneCyclePlotingTime < REFRESH)) {
        oneCyclePlotingTime = iteration * DT;
        Time = Time + DT;
        if (iteration % DIVIDER == 0) {
            MainUI->data_x[MainUI->data_idx] = Time;
            MainUI->data_y1[MainUI->data_idx] = encoder.getRawReadedData();
            MainUI->data_y1_desired[MainUI->data_idx] = encoder.getFilteredData();
            MainUI->data_y2[MainUI->data_idx] = encoder.getRawDegreeData();
            MainUI->data_y2_desired[MainUI->data_idx] = encoder.getFilteredDegree();
            MainUI->data_y3_blue[MainUI->data_idx] = encoder.getRawRadianData();
            MainUI->data_y3_red[MainUI->data_idx] = encoder.getFilteredRadian();
            MainUI->data_idx += 1;
            MainUI->plotWidget1();
            MainUI->plotWidget2();
            MainUI->plotWidget3();
        }
        iteration++;
    } else if (oneCyclePlotingTime >= REFRESH) {
        iteration = 0;
        oneCyclePlotingTime = 0;
        MainUI->data_idx = 0;
    }
}

void *rt_simulation_thread(void *arg) {
    std::cout << "entered #rt_time_checker_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(DT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        dataPloting();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

void *rt_encoder_thread(void *arg){
    while(true){
        encoder.readFilteredData();
    }
}

int main (int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    int thread_id_timeChecker = generate_rt_thread(thread_plotting, rt_simulation_thread, "simulation_thread", 0, 99,
                                                   NULL);
    int thread_id_encoder = generate_rt_thread(thread_encoder, rt_encoder_thread, "encoder_thread", 1, 98,
                                                   NULL);

    w.show();

    return a.exec();
}
