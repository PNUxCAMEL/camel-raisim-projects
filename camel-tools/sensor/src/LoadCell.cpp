#include "../include/LoadCell.hpp"

LoadCell::LoadCell()
{
    mbIsConnected = true;
    mbIsTtySet = true;
    mbIsReaded = true;
    mbIsDataStore = true;
    mbIsNegativeValue = false;
    mReadedData = 0;
    mIdx = 0;
    mInclineWeight = 0.01081112031;
    mBiasWeight = -288.7621365;
    mInclineForce = 0.0001060570902;
    mBiasForce = -2.832756559;
    mSensoredWeight = 0;
    mSensoredForce = 0;
    initUSBComm();
    flushData(10);
    nulling();
}

int LoadCell::GetRawData() const
{
    return mReadedData;
}

double LoadCell::GetSensoredWeight() const
{
    return mSensoredWeight;
}

double LoadCell::GetSensoredForce() const
{
    return mSensoredForce;
}

void LoadCell::ReadData()
{
    mReadedData = 0;
    while (true)
    {
        mNumBytes = read(mSerialPort, &mReadBuf, sizeof(mReadBuf));
        if (mNumBytes == 1)
        {
            if (mReadBuf[0] == mLineFeedCode)
            {
                break;
            }
            else if (mReadBuf[0] == mSplitCode)
            {
                mbIsDataStore = false;
            }
            else if (mReadBuf[0] == mNegativeValueCode)
            {
                mbIsNegativeValue = true;
            }
            else if (mbIsDataStore)
            {
                mReaded[mIdx] = (mReadBuf[0] - 48);
                mIdx++;
            }
        }
    }

    for (int i = 0; i < mIdx; i++)
    {
        mReadedData += mReaded[i] * pow(10.0, mIdx - i - 1);
    }
    if (mbIsNegativeValue)
    {
        mReadedData = -1.0 * mReadedData;
    }
    mSensoredWeight = mReadedData * mInclineWeight + mBiasWeight;
    mSensoredForce = mReadedData * mInclineForce + mBiasForce;

    // reset
    mIdx = 0;
    mbIsDataStore = true;
    mbIsNegativeValue = false;
}

void LoadCell::initUSBComm()
{
    mSerialPort = open("/dev/ttyUSB0", O_RDWR);
    if (tcgetattr(mSerialPort, &mTty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        mbIsConnected = false;
    }
    std::cout << "USB serial communication is successfully connected." << std::endl;
    sleep(1);
    mTty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    mTty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    mTty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    mTty.c_cflag |= CS8; // 8 bits per byte (most common)
    mTty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    mTty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    mTty.c_lflag &= ~ICANON;
    mTty.c_lflag &= ~ECHO; // Disable echo
    mTty.c_lflag &= ~ECHOE; // Disable erasure
    mTty.c_lflag &= ~ECHONL; // Disable new-line echo
    mTty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    mTty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    mTty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
        ICRNL); // Disable any special handling of received bytes

    mTty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    mTty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    mTty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    mTty.c_cc[VMIN] = 0;

    cfsetispeed(&mTty, B115200);
    cfsetospeed(&mTty, B115200);
    if (tcsetattr(mSerialPort, TCSANOW, &mTty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        mbIsTtySet = false;
    }
    std::cout << "tty setting is successfully set." << std::endl;
    sleep(1);
    memset(&mReadBuf, '\0', sizeof(mReadBuf));
    mNumBytes = read(mSerialPort, &mReadBuf, sizeof(mReadBuf));

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (mNumBytes < 0)
    {
        printf("Error reading: %s", strerror(errno));
        mbIsReaded = false;
    }
    std::cout << "Success to initialize USB serial COM communication for LoadCell." << std::endl;
}

void LoadCell::nulling()
{
    // it takes 2 seconds.
    std::cout << "Nulling Loadcell." << std::endl;
    double tempSumedForce = 0;
    double tempSumedWeight = 0;
    double tempBiasForce = 0;
    double tempBiasWeight = 0;
    for (int i = 0; i < 400; i++)
    {
        ReadData();
        tempSumedForce += GetSensoredForce();
        tempSumedWeight += GetSensoredWeight();
    }
    std::cout << tempSumedWeight << std::endl;
    tempBiasForce = tempSumedForce / 400.0;
    tempBiasWeight = tempSumedWeight / 400.0;

    std::cout << mBiasWeight << std::endl;
    mBiasForce -= tempBiasForce;
    mBiasWeight -= tempBiasWeight;
    std::cout << mBiasWeight << std::endl;
}

void LoadCell::flushData(int num)
{
    for (int i = 0; i < num; i++)
    {
        ReadData();
    }
}