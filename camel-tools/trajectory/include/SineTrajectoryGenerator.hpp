#ifndef RAISIM_SINCURVETRAJECTORYGENERATOR_H
#define RAISIM_SINCURVETRAJECTORYGENERATOR_H
#define PI 3.141592

class SineTrajectoryGenerator{
public:
    void updateTrajectory(double currentPosition, double currentTime, double amplitude, double frequency);
    double getPositionTrajectory(double currentTime);
    double getVelocityTrajectory(double currentTime);

private:
    double mReferencePose;
    double mReferenceTime;
    double mAmplitude;
    double mFrequency;
};

#endif //RAISIM_SINCURVETRAJECTORYGENERATOR_H
