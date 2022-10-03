#ifndef RAISIM_SINGLELEGGEDSHAREDMEMORY_HPP
#define RAISIM_SINGLELEGGEDSHAREDMEMORY_HPP

typedef struct _SHM_
{
    double localTime;
    double positionZ;
    double desiredPositionZ;
    double velocityZ;
    double desiredVelocityZ;
    double jointPosition[2];
    double jointVelocity[2];
    double jointTorque[2];
    double GRF;
}SHM, *pSHM;

#endif //RAISIM_SINGLELEGGEDSHAREDMEMORY_HPP
