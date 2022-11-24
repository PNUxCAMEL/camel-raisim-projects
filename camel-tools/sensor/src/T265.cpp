
#include "../include/T265.hpp"

T265::T265()
{
}

void T265::SetConfig()
{
    if(!mSerial.empty())
        mCfg.enable_device(mSerial);
    mCfg.enable_stream(RS2_STREAM_POSE,RS2_FORMAT_6DOF);
    std::cout << "t265 setconfig" << std::endl;
}

void T265::ParseData() {
    std::cout << "t265 partdata" << std::endl;
    auto callback = [&](const rs2::frame &frame){///200hz
        rs2::pose_frame fp = frame.as<rs2::pose_frame>();
        rs2_pose pose_data = fp.get_pose_data();
        auto now = std::chrono::system_clock::now().time_since_epoch();
        double now_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
        double pose_time_ms = fp.get_timestamp();
        float dt_s = static_cast<float>(std::max(0., (now_ms - pose_time_ms)/1000.));
        rs2_pose predicted_pose = predict_pose(pose_data, dt_s);
        mT265quat = predicted_pose.rotation;
        mT265translation = predicted_pose.translation;
        mT265vel = predicted_pose.velocity;
        mT265acc = predicted_pose.acceleration;
        mT265angAcc = predicted_pose.angular_acceleration;
        mT265angVel = predicted_pose.angular_velocity;
    };
    rs2::pipeline_profile profile = mPipe.start(mCfg, callback);
}


rs2_pose T265::predict_pose(rs2_pose & pose, float dt_s)
{
    rs2_pose P = pose;
    P.translation.x = dt_s * (dt_s/2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
    P.translation.y = dt_s * (dt_s/2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
    P.translation.z = dt_s * (dt_s/2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
    rs2_vector W = {
        dt_s * (dt_s/2 * pose.angular_acceleration.x + pose.angular_velocity.x),
        dt_s * (dt_s/2 * pose.angular_acceleration.y + pose.angular_velocity.y),
        dt_s * (dt_s/2 * pose.angular_acceleration.z + pose.angular_velocity.z),
    };
    P.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
    return P;
}

rs2_quaternion T265::quaternion_multiply(rs2_quaternion a, rs2_quaternion b)
{
    rs2_quaternion Q = {
        a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
        a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
        a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
    return Q;
}

rs2_quaternion T265::quaternion_exp(rs2_vector v)
{
    float x = v.x/2, y = v.y/2, z = v.z/2, th2, th = sqrtf(th2 = x*x + y*y + z*z);
    float c = cosf(th), s = th2 < sqrtf(120*FLT_EPSILON) ? 1-th2/6 : sinf(th)/th;
    rs2_quaternion Q = { s*x, s*y, s*z, c };
    return Q;
}

rs2_quaternion T265::GetT265quat() const
{
    return mT265quat;
}
rs2_vector T265::GetT265vel() const
{
    return mT265vel;
}

rs2_vector T265::GetT265pos() const
{
    return mT265translation;
}