//
// Created by hs on 22. 12. 12.
//

#ifndef RAISIM_CONTROLLER_HPP
#define RAISIM_CONTROLLER_HPP

#include <vector>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <canine_util/RobotDescription.hpp>
#include <canine_util/SharedMemory.hpp>
#include <canine_util/EigenTypes.hpp>
#include <canine_util/RobotMath.hpp>

class WholeBodyController{
public:
    WholeBodyController();
    ~WholeBodyController();
    void DoControl();
private:
    void updateState();
    void setControlInput();
    void setLegControl();
    void getJointPos(const double& x, const double& z, Vec3<double>& pos);
private:
    const int mTorqueLimit;
    const std::string mUrdfPath;
    RigidBodyDynamics::Model* mModel;
    RigidBodyDynamics::Math::VectorNd mQ;
    RigidBodyDynamics::Math::VectorNd mQdot;
    RigidBodyDynamics::Math::VectorNd mQddot;
    RigidBodyDynamics::Math::VectorNd mTau;

    Vec3<double> mBasePosition;
    Vec3<double> mBaseVelocity;
    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];
    Vec3<double> mBodyFootPosition[4];
    Vec3<double> mGlobalFootPosition[4];
    Vec3<double> mDesiredFootPosition[4];
    Vec3<double> mSwingFootPosition;
    Vec4<double> mBaseQuaternion;
    Vec3<double> mBaseEulerVelocity;
    Vec3<double> mBaseEulerPosition;

    Vec3<double> mTorque[4];

    Vec3<double> mStandJointPos;
    Vec3<double> mStandJointVel;
    const double mTranslationGain[2];
    const double mRotationGain[2];
    const double mLegGain[6];
};


#endif //RAISIM_CONTROLLER_HPP
