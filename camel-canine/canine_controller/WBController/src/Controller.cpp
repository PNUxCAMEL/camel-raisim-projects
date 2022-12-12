#include <WBController/Controller.hpp>

extern pSHM sharedMemory;

WholeBodyController::WholeBodyController()
    : mUrdfPath(robotURDF)
    , mTorqueLimit(35)
{
    mModel = new RigidBodyDynamics::Model();
    bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(mUrdfPath.c_str(), mModel, true);
    if (modelLoaded)
    {
        std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(*mModel);
        std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(*mModel);
        std::cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(*mModel);
        mQ = RigidBodyDynamics::Math::VectorNd::Zero(mModel->dof_count);
        mQdot = RigidBodyDynamics::Math::VectorNd::Zero(mModel->dof_count);
        mQddot = RigidBodyDynamics::Math::VectorNd::Zero(mModel->dof_count);
        mTau = RigidBodyDynamics::Math::VectorNd::Zero(mModel->dof_count);
    }
    else
    {
        std::cout << "RBDL Model Load Failed!" << std::endl;
    }
}

WholeBodyController::~WholeBodyController()
{
    delete mModel;
}

void WholeBodyController::DoControl()
{
    updateState();
    setLegControl();
    setControlInput();
}

void WholeBodyController::updateState()
{
    for (int idx=0; idx<3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
    }

    for (int idx=0; idx<4; idx++)
    {
        mBaseQuaternion[idx] = sharedMemory->baseQuartPosition[idx];
        mBodyFootPosition[idx] = sharedMemory->bodyFootPosition[idx];
        mGlobalFootPosition[idx] = sharedMemory->globalFootPosition[idx];
        for (int mt=0; mt<3; mt++)
        {
            mMotorPosition[idx][mt] = sharedMemory->motorPosition[idx*3+mt];
            mMotorVelocity[idx][mt] = sharedMemory->motorVelocity[idx*3+mt];

        }
    }
}

void WholeBodyController::setLegControl()
{
    mQ << 0,0,0.35,
          0,0,0,
          0,0.7,-1.4,
          0,0.7,-1.4,
          0,0.7,-1.4,
          0,0.7,-1.4;
    RigidBodyDynamics::InverseDynamics(*mModel, mQ, mQdot, mQddot, mTau);

    std::cout << mTau.transpose() << std::endl;

    for(int idx=0; idx<4; idx++)
    {
        mTorque[idx][0] = mTau[idx*3+6];
        mTorque[idx][1] = mTau[idx*3+7];
        mTorque[idx][2] = mTau[idx*3+8];
    }
}

void WholeBodyController::setControlInput()
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int motor = 0; motor < 3; motor++)
        {
            if (mTorque[leg][motor] > mTorqueLimit)
            {
                mTorque[leg][motor] = mTorqueLimit;
            }
            else if (mTorque[leg][motor] < -mTorqueLimit)
            {
                mTorque[leg][motor] = -mTorqueLimit;
            }
            sharedMemory->motorDesiredTorque[leg*3+motor] = mTorque[leg][motor];
        }
    }
}