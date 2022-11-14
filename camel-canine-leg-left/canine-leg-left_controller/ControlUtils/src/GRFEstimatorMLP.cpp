//
// Created by jh on 22. 11. 3.
//

#include <ControlUtils/GRFEstimatorMLP.hpp>

extern pSHM sharedMemory;

GRFEstimatorMLP::GRFEstimatorMLP()
{
    mRunOption = TF_NewBufferFromString("", 0);
    mSessionOptions = TF_NewSessionOptions();
    mGraph = TF_NewGraph();
    mStatus = TF_NewStatus();

    std::array<char const*, 1> tags{ "serve" };
//    const char* modelPath = "/home/jh/AnacondaWorkspace/tensorflow260/DynamicBehaviorIdentification/RITA_models/model15";
    const char* modelPath = "/home/jh/AnacondaWorkspace/tensorflow260/DynamicBehaviorIdentification/RITA_models/model_v3";
    mSession = TF_LoadSessionFromSavedModel(mSessionOptions, mRunOption,
        modelPath,
        tags.data(),
        tags.size(),
        mGraph, nullptr, mStatus);
    if (TF_GetCode(mStatus) != TF_OK)
    {
        std::cout << TF_Message(mStatus) << '\n';
    }

    mInputOperation = TF_GraphOperationByName(mGraph, "serving_default_dense_input");
//    mInputOperation = TF_GraphOperationByName(mGraph, "serving_default_dense_10_input");

    if (mInputOperation == nullptr)
    {
        std::cout << "Failed to find graph operation\n";
    }

    mOutputOperation = TF_GraphOperationByName(mGraph, "StatefulPartitionedCall");
    if (mOutputOperation == nullptr)
    {

        std::cout << "Failed to find graph operation\n";
    }


    mInputOps = { TF_Output{ mInputOperation, 0 }};
    mOutputOps = { TF_Output{ mOutputOperation, 0 }};

    std::array<float, 14> x{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
    std::vector<std::array<float, 14>> inputs{ x };

    // input dimension
    std::array<int64_t, 2> const dims{ static_cast<int64_t>(inputs.size()), static_cast<int64_t>(x.size()) };
    void* data = (void*)inputs.data();
    std::size_t const ndata = inputs.size() * x.size() * TF_DataTypeSize(TF_FLOAT);

    auto const deallocator = [](void*, std::size_t, void*)
    {}; // unused deallocator because of RAII

    auto* input_tensor = TF_NewTensor(TF_FLOAT, dims.data(), dims.size(), data, ndata, deallocator, nullptr);
    std::array<TF_Tensor*, 1> input_values{ input_tensor };
    std::array<TF_Tensor*, 1> output_values{ };

    TF_SessionRun(mSession,
        mRunOption,
        mInputOps.data(), input_values.data(), mInputOps.size(),
        mOutputOps.data(), output_values.data(), mOutputOps.size(),
        nullptr, 0,
        nullptr,
        mStatus);
    if (TF_GetCode(mStatus) != TF_OK)
    {
        std::cout << TF_Message(mStatus) << '\n';
    }
    auto* output_tensor = static_cast<std::array<float, 1>*>(TF_TensorData(output_values[0]));
    std::vector<std::array<float, 1>> outputs{ output_tensor, output_tensor + inputs.size() };
}

void GRFEstimatorMLP::Estimate()
{
    //pos hip, knee /vel hip, knee
//    sharedMemory->GRFNetInputs[0] = sharedMemory->bufMotorPosition[0][0];
//    sharedMemory->GRFNetInputs[1] = sharedMemory->bufMotorPosition[0][4];
//    sharedMemory->GRFNetInputs[2] = sharedMemory->bufMotorPosition[0][8];
//    sharedMemory->GRFNetInputs[3] = sharedMemory->bufMotorPosition[1][0];
//    sharedMemory->GRFNetInputs[4] = sharedMemory->bufMotorPosition[1][4];
//    sharedMemory->GRFNetInputs[5] = sharedMemory->bufMotorPosition[1][8];
//    sharedMemory->GRFNetInputs[6] = sharedMemory->bufMotorVelocity[0][0];
//    sharedMemory->GRFNetInputs[7] = sharedMemory->bufMotorVelocity[0][4];
//    sharedMemory->GRFNetInputs[8] = sharedMemory->bufMotorVelocity[0][8];
//    sharedMemory->GRFNetInputs[9] = sharedMemory->bufMotorVelocity[1][0];
//    sharedMemory->GRFNetInputs[10] = sharedMemory->bufMotorVelocity[1][4];
//    sharedMemory->GRFNetInputs[11] = sharedMemory->bufMotorVelocity[1][8];
//    sharedMemory->GRFNetInputs[12] = sharedMemory->motorDesiredTorque[0];
//    sharedMemory->GRFNetInputs[13] = sharedMemory->motorDesiredTorque[1];

    sharedMemory->GRFNetInputs[0] = sharedMemory->bufMotorPosition[0][0];
    sharedMemory->GRFNetInputs[1] = sharedMemory->bufMotorPosition[0][20];
    sharedMemory->GRFNetInputs[2] = sharedMemory->bufMotorPosition[0][40];
    sharedMemory->GRFNetInputs[3] = sharedMemory->bufMotorPosition[1][0];
    sharedMemory->GRFNetInputs[4] = sharedMemory->bufMotorPosition[1][20];
    sharedMemory->GRFNetInputs[5] = sharedMemory->bufMotorPosition[1][40];
    sharedMemory->GRFNetInputs[6] = sharedMemory->bufMotorVelocity[0][0];
    sharedMemory->GRFNetInputs[7] = sharedMemory->bufMotorVelocity[0][20];
    sharedMemory->GRFNetInputs[8] = sharedMemory->bufMotorVelocity[0][40];
    sharedMemory->GRFNetInputs[9] = sharedMemory->bufMotorVelocity[1][0];
    sharedMemory->GRFNetInputs[10] = sharedMemory->bufMotorVelocity[1][20];
    sharedMemory->GRFNetInputs[11] = sharedMemory->bufMotorVelocity[1][40];
    sharedMemory->GRFNetInputs[12] = sharedMemory->motorDesiredTorque[0];
    sharedMemory->GRFNetInputs[13] = sharedMemory->motorDesiredTorque[1];

    std::array<float, 14> x{ sharedMemory->GRFNetInputs[0], sharedMemory->GRFNetInputs[1], sharedMemory->GRFNetInputs[2],
                             sharedMemory->GRFNetInputs[3], sharedMemory->GRFNetInputs[4], sharedMemory->GRFNetInputs[5],
                             sharedMemory->GRFNetInputs[6], sharedMemory->GRFNetInputs[7], sharedMemory->GRFNetInputs[8],
                             sharedMemory->GRFNetInputs[9], sharedMemory->GRFNetInputs[10], sharedMemory->GRFNetInputs[11],
                             sharedMemory->GRFNetInputs[12], sharedMemory->GRFNetInputs[13] };
    std::vector<std::array<float, 14>> inputs{ x };

    // input dimension
    std::array<int64_t, 2> const dims{ static_cast<int64_t>(inputs.size()), static_cast<int64_t>(x.size()) };
    void* data = (void*)inputs.data();
    std::size_t const ndata = inputs.size() * x.size() * TF_DataTypeSize(TF_FLOAT);

    auto const deallocator = [](void*, std::size_t, void*)
    {}; // unused deallocator because of RAII

    auto* input_tensor = TF_NewTensor(TF_FLOAT, dims.data(), dims.size(), data, ndata, deallocator, nullptr);
    std::array<TF_Tensor*, 1> input_values{ input_tensor };
    std::array<TF_Tensor*, 1> output_values{ };

    TF_SessionRun(mSession,
        mRunOption,
        mInputOps.data(), input_values.data(), mInputOps.size(),
        mOutputOps.data(), output_values.data(), mOutputOps.size(),
        nullptr, 0,
        nullptr,
        mStatus);
    if (TF_GetCode(mStatus) != TF_OK)
    {
        std::cout << TF_Message(mStatus) << '\n';
    }
    auto* output_tensor = static_cast<std::array<float, 1>*>(TF_TensorData(output_values[0]));
    std::vector<std::array<float, 1>> outputs{ output_tensor, output_tensor + inputs.size() };
    sharedMemory->estimatedGRFMLP = outputs[0][0];
}