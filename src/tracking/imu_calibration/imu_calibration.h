class correctionData {
    public:
        double A[3][3] = {0};
        double b[3] = {0};
};

class CalibratedIMUData {
    public:
        float x;
        float y;
        float z;
};

void readCalibrationData(float x, float y, float z, int i);
void calibrate();
void sendCalculated();
void hardIronCorrection(float x, float y, float z);
CalibratedIMUData calibratedMagData(float x, float y, float z);
CalibratedIMUData calibratedAccData(float x, float y, float z);
float tiltCompensatedHeading(float x, float y, float z, float pitch, float roll);