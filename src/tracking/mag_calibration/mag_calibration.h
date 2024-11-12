class correctionData {
    public:
        double A[3][3] = {0};
        double b[3] = {0};
};

class CalibratedMagData {
    public:
        float x;
        float y;
        float z;
};

void readCalibrationData(float x, float y, float z, int i);
void calibrate();
void sendCalculated();
void hardIronCorrection(float x, float y, float z);
CalibratedMagData calibratedData(float x, float y, float z);
