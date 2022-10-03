//
// Created by user on 22. 6. 30.
//

#include "RE22SC.h"

void RE22SC::readData() {
    mReadData = 0;
    while (true) {
        mNumBytes = read(mSerialPort, &mReadBuf, sizeof(mReadBuf));
        if (mNumBytes == 1) {
            if (mReadBuf[0] == mLineFeedCode) {
                break;
            } else if (mReadBuf[0] == mCarriageReturnCode) {
                mIsDataStore = false;
            } else if (mIsDataStore) {
                mRead[mIdx] = (mReadBuf[0] - 48);
                mIdx++;
            }
        }
    }
    for (int i = 0; i < mIdx; i++) {
        mReadData += mRead[i] * pow(10.0, mIdx - i - 1);
    }
    mRawData = mReadData;

    // reset
    mIdx = 0;
    mIsDataStore = true;
}

void RE22SC::lowPassFilter() {
    if(abs(mPreviousData - mReadData) > REVOLUTE){
        mFilteredData = mReadData;
    }else{
        mFilteredData = BETA * mPreviousData + (1 - BETA) * mReadData;
    }
    mPreviousData = mFilteredData;
}

void RE22SC::initializing() {
    for(int i = 0; i < N_INITIALIZING; i++)
    {
        readData();
    }
    while(abs(mPreviousData - mReadData) > DIFFERENCE_INITIALIZING){
        readData();
        mPreviousData = mReadData;
    }
    std::cout<<"finish initializing RE22SC data"<<std::endl;
}

void RE22SC::readFilteredData() {
    readData();
    lowPassFilter();
}

double RE22SC::getRawDegreeData() {
    return mRawData*ENC2DEG;
}

double RE22SC::getRawRadianData() {
    return mRawData*ENC2RAD;
}

double RE22SC::getFilteredDegree() {
    return mFilteredData*ENC2DEG;
}

double RE22SC::getFilteredRadian() {
    return mFilteredData*ENC2RAD;
}