#include <WBController/Controller.hpp>

extern pSHM sharedMemory;

WholeBodyController::WholeBodyController()
    : mUrdfPath(robotURDF)
    , mTorqueLimit(35)
    , mTranslationGain{100, 2}
    , mRotationGain{100, 2}
    , mLegGain{30,1}
{
    mModel = new RigidBodyDynamics::Model();
    bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(mUrdfPath.c_str(), mModel, true);
    if (modelLoaded)
    {
        std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(*mModel);
        std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(*mModel);
        mQ = RigidBodyDynamics::Math::VectorNd::Zero(mModel->q_size);
        mQdot = RigidBodyDynamics::Math::VectorNd::Zero(mModel->q_size);
        mQddot = RigidBodyDynamics::Math::VectorNd::Zero(mModel->q_size);
        mTau = RigidBodyDynamics::Math::VectorNd::Zero(mModel->q_size);
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
//    setControlInput();
}

void WholeBodyController::updateState()
{
    for (int idx=0; idx<3; idx++)
    {
        mBasePosition[idx] = sharedMemory->basePosition[idx];
        mBaseVelocity[idx] = sharedMemory->baseVelocity[idx];
        mBaseEulerVelocity[idx] = sharedMemory->baseEulerVelocity[idx];
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
    mQ << mBasePosition[0], mBasePosition[1], mBasePosition[2],
          mBaseQuaternion[1],mBaseQuaternion[2],mBaseQuaternion[3],
          mMotorPosition[1][0], mMotorPosition[1][1], mMotorPosition[1][2],
          mMotorPosition[3][0], mMotorPosition[3][1], mMotorPosition[3][2],
          mMotorPosition[0][0], mMotorPosition[0][1], mMotorPosition[0][2],
          mMotorPosition[2][0], mMotorPosition[2][1], mMotorPosition[2][2],
          mBaseQuaternion[2];
    mQdot << mBaseVelocity[0], mBaseVelocity[1], mBaseVelocity[2],
             mBaseEulerVelocity[0], mBaseEulerVelocity[1], mBaseEulerVelocity[2],
             mMotorVelocity[1][0], mMotorVelocity[1][1], mMotorVelocity[1][2],
             mMotorVelocity[3][0], mMotorVelocity[3][1], mMotorVelocity[3][2],
             mMotorVelocity[0][0], mMotorVelocity[0][1], mMotorVelocity[0][2],
             mMotorVelocity[2][0], mMotorVelocity[2][1], mMotorVelocity[2][2],
             0;
}

void WholeBodyController::getJointPos(const double& x, const double& z, Vec3<double>& pos)
{
    double d = sqrt(pow(x,2)+pow(z,2));
    double phi = acos(abs(x)/ d);
    double psi = acos(pow(d,2)/(2*LEN_THI*d));

    if (x < 0)
        pos[1] = 1.57 - phi + psi;
    else if(x == 0)
        pos[1] = psi;
    else
        pos[1] = phi + psi - 1.57;
    pos[2] = -acos((pow(d,2)-2*pow(LEN_CAL,2)) / (2*LEN_CAL*LEN_CAL));
}

void WholeBodyController::setLegControl()
{
    for (int i=0; i<3; i++)
    {
        mQddot[i] = sharedMemory->baseDesiredAcceleration[i]
                  + mTranslationGain[0] * (sharedMemory->baseDesiredPosition[i] - mQ[i])
                  + mTranslationGain[1] * (sharedMemory->baseDesiredVelocity[i] - mQdot[i]);
        mQddot[i+3] = mRotationGain[0] * (0 - mQ[i+3])
                      + mRotationGain[1] * (0 - mQdot[i+3]);
    }
    for (int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            mQddot[i*3+6+j] = sharedMemory->legDesiredAcceleration[j]
                          + mLegGain[0] * (sharedMemory->legDesiredPosition[j] - mQ[i*3+6+j])
                          + mLegGain[1] * (sharedMemory->legDesiredVelocity[j] - mQdot[i*3+6+j]);
        }
    }

    std::cout << "========================================================================" << std::endl;
    std::cout << "========================================================================" << std::endl;
    std::cout << mQ.transpose() << std::endl;
    std::cout << mQdot.transpose() << std::endl;
    std::cout << mQddot.transpose() << std::endl;
    RigidBodyDynamics::InverseDynamics(*mModel, mQ, mQdot, mQddot, mTau);

    std::cout << mTau.transpose() << std::endl;
    for (int i=0; i<4; i++)
    {
        mTorque[i][0] = mQddot[i*3+6];
        mTorque[i][1] = mQddot[i*3+7];
        mTorque[i][2] = mQddot[i*3+8];
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