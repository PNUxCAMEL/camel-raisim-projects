//
// Created by jaehoon on 22. 6. 9.
//
#include "camel-tools/sensor.hpp"

int main()
{
    LoadCell sensorLoadcell;

    int rawData = 0;
    double force = 0.0;
    double weight = 0.0;

    while (true)
    {
        sensorLoadcell.ReadData();
        force = sensorLoadcell.GetSensoredForce();
        weight = sensorLoadcell.GetSensoredWeight();
        rawData = sensorLoadcell.GetRawData();
        std::cout << "Sensored force[N] : " << force << std::endl;
        std::cout << "Sensored weight[g] : " << weight << std::endl;
        std::cout << "Raw data : " << rawData << std::endl;
    }
}