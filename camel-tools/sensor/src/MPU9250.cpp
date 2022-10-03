//
// Created by user on 22. 6. 23.
//

#include "MPU9250.h"

void MPU9250::readData() {
    mReadedData = 0;
    while (true) {
        mNumBytes = read(mSerialPort, &mReadBuf, sizeof(mReadBuf));
        if (mNumBytes == 1) {
            if (mReadBuf[0] == mLineFeedCode) {
                mGyroIdx++;
                break;
            } else if (mReadBuf[0] == mCommaCode) {
                mIsDataStore = false;
                mGyroIdx++;
                break;
            } else if (mReadBuf[0] == mCarriageReturnCode) {
                mCarriageReturnCode = false;
            } else if (mReadBuf[0] == mDotCode) {
                mIsDataStore = false;
                mDotIdx = mIdx;
            } else if (mReadBuf[0] == mNegativeValueCode) {
                mIsNegativeValue = true;
            } else if (mIsDataStore) {
                mReaded[mIdx] = (mReadBuf[0] - 48);
                mIdx++;
            }
        }
        //reset
        mIsDataStore = true;
    }
    for (int i = 0; i < mIdx; i++) {
        mReadedData += mReaded[i] * pow(10.0, mDotIdx-i-1);
    }
    if (mIsNegativeValue) {
        mReadedData = -1.0 * mReadedData;
    }
    mGyroXYZ[mGyroIdx-1] = mReadedData;

    // reset
    mIdx = 0;
    mIsDataStore = true;
    mIsNegativeValue = false;
}

void MPU9250::getGyroXYZ() {
    mGyroIdx = 0;
    while(mGyroIdx<3){
        readData();
    }
    std::cout << "mGyroXYZ" << std::endl;
    std::cout << mGyroXYZ[0] <<"   \t"<< mGyroXYZ[1] <<"   \t"<< mGyroXYZ[2] << std::endl;
}

void MPU9250::flushData(int num) {
    while(true){
        getGyroXYZ();
    }
}