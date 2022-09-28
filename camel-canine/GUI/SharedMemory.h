//
// Created by hs on 22. 7. 21.
//

#ifndef RAISIM_SHAREDMEMORY_H
#define RAISIM_SHAREDMEMORY_H

typedef struct _SHM_
{
    double simTime;
    double GetPosX;
    double GetPosY;
    double GetPosZ;
    double DesPosX;
    double DesPosY;
    double DesPosZ;

    double GetRotX;
    double GetRotY;
    double GetRotZ;
    double DesRotX;
    double DesRotY;
    double DesRotZ;

    double footPointX[4];
    double footPointY[4];
    double footPointZ[4];


}SHM, *pSHM;

#endif //RAISIM_SHAREDMEMORY_H
