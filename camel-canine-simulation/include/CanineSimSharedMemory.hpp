
#ifndef RAISIM_CANINESIMSHAREDMEMORY_HPP
#define RAISIM_CANINESIMSHAREDMEMORY_HPP

typedef struct _SHM_
{
    double simTime;
    double PositionHip;
    double PositionKnee;
    double PositionBase;
    double DesiredPositionHip;
    double DesiredPositionKnee;
    double DesiredPositionBase;
    double VelocityHip;
    double VelocityKnee;
    double VelocityBase;
    double DesiredVelocityHip;
    double DesiredVelocityKnee;
    double DesiredVelocityBase;
    double TorqueHip;
    double TorqueKnee;
    double DesiredTorqueHip;
    double DesiredTorqueKnee;
}SHM, *pSHM;

#endif //RAISIM_CANINESIMSHAREDMEMORY_HPP
