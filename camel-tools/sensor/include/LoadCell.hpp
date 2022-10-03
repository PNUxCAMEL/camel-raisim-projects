#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

class LoadCell
{
public:
    LoadCell();

    int GetRawData() const;
    double GetSensoredWeight() const;
    double GetSensoredForce() const;
    void ReadData();

private:
    void initUSBComm();
    void nulling();
    void flushData(int num);

private:
    struct termios mTty;
    char mReadBuf[1];
    bool mbIsConnected;
    bool mbIsTtySet;
    bool mbIsReaded;
    bool mbIsDataStore;
    bool mbIsNegativeValue;
    const int mLineFeedCode = 10;
    const int mSplitCode = 46;
    const int mNegativeValueCode = 45;
    int mSerialPort;
    int mNumBytes;
    int mReadedData;
    int mIdx;
    int mReaded[10];
    double mInclineWeight;
    double mBiasWeight;
    double mInclineForce;
    double mBiasForce;
    double mSensoredWeight;

    double mSensoredForce;
};