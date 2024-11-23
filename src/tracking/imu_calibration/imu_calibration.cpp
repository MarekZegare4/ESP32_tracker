#include <Arduino.h>
#include "imu_calibration.h"
// #include <ArduinoEigen.h>

// using Eigen::EigenSolver;
// using Eigen::MatrixXd;
// using Eigen::VectorXd;

// VectorXd calibrationX(300);
// VectorXd calibrationY(300);
// VectorXd calibrationZ(300);

// MatrixXd A_1{
//     {0.7763, 0.0341, -0.0172},
//     {0.0341, 0.7662, -0.0100},
//     {-0.0172, -0.01, 0.7285}};
// VectorXd b{
//     -48.1067,
//      36.9905,
//     -39.3537};
// MatrixXd D(10, 300);
// MatrixXd C{{-1, 1, 1, 0, 0, 0},
//            {1, -1, 1, 0, 0, 0},
//            {1, 1, -1, 0, 0, 0},
//            {0, 0, 0, -4, 0, 0},
//            {0, 0, 0, 0, -4, 0},
//            {0, 0, 0, 0, 0, -4}};
// MatrixXd S(10, 10);

// /*
//     @brief Read uncalibrated data from the magnetometer into the matrix [3x300]
//     @param x: x-axis data
//     @param y: y-axis data
//     @param z: z-axis data
//     @param i: index of the data
// */
// void readCalibrationData(float x, float y, float z, int i)
// {
//     calibrationX(i) = x;
//     calibrationY(i) = y;
//     calibrationZ(i) = z;
// }

// void calibrate()
// {

//     D.row(0) = calibrationX.array().square();
//     D.row(1) = calibrationY.array().square();
//     D.row(2) = calibrationZ.array().square();
//     D.row(3) = 2 * calibrationY.array() * calibrationZ.array();
//     D.row(4) = 2 * calibrationX.array() * calibrationZ.array();
//     D.row(5) = 2 * calibrationX.array() * calibrationY.array();
//     D.row(6) = 2 * calibrationX;
//     D.row(7) = 2 * calibrationY;
//     D.row(8) = 2 * calibrationZ;
//     D.row(9) = VectorXd::Ones(300);

//     S = D * D.transpose();
//     MatrixXd S_11 = S.block<6, 6>(0, 0);
//     MatrixXd S_12 = S.block<6, 4>(0, 6);
//     MatrixXd S_21 = S.block<4, 6>(6, 0);
//     MatrixXd S_22 = S.block<4, 4>(6, 6);

//     MatrixXd E = C.inverse() * (S_11 - (S_12 * (S_22.inverse() * S_12.transpose())));
//     EigenSolver<MatrixXd> eigenValue(E);
//     MatrixXd E_v = eigenValue.eigenvectors().real();
//     VectorXd E_w = eigenValue.eigenvalues().real();

//     Eigen::Index maxRow, maxCol;
//     double max = E_w.maxCoeff(&maxRow, &maxCol);
//     VectorXd v_1 = E_v.col(maxCol);
//     if (v_1(0) < 0)
//     {
//         v_1 = -v_1;
//     }
//     VectorXd v_2 = (-S_22.inverse() * S_21) * v_1;

//     MatrixXd M{
//         {v_1(0), v_1(5), v_1(4)},
//         {v_1(5), v_1(1), v_1(3)},
//         {v_1(4), v_1(3), v_1(2)}};

//     MatrixXd N{
//         {v_2(0)},
//         {v_2(1)},
//         {v_2(2)}};

//     double d = v_2(3);

//     b = -M.inverse() * N;

//     MatrixXd M_sqrt = (M.array() + sqrt(M.determinant())) / (sqrt(M.transpose().array() + 2 * sqrt(M.determinant())));

//     A_1 = ((1 / (sqrt((N.transpose() * (M.inverse() * N)).array() - d))) * M_sqrt.array()).real();
// }

// void sendCalculated()
// {
//     Serial.println("Matrix A_1:");
//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             Serial.print(A_1(i, j));
//             Serial.print(" ");
//         }
//         Serial.println();
//     }
//     Serial.println("Vector b:");
//     for (int i = 0; i < 3; i++)
//     {
//         Serial.print(b(i));
//         Serial.print(" ");
//     }
//     Serial.println();
// }

// Correction matrices for Hard and Soft Iron calibration
/*
@brief Outputs the tilt compensated heading
@param mag_x: x-axis magnetometer data 
@param mag_y: y-axis magnetometer data
@param mag_z: z-axis magnetometer data
@param pitch: pitch angle in radians
@param roll: roll angle in radians
*/
float tiltCompensatedHeading(float mag_x, float mag_y, float mag_z, float pitch, float roll)
{   
    pitch = radians(pitch);
    roll = radians(roll);
    float xh = mag_x * cos(pitch) + mag_z * sin(pitch);
    float yh = mag_x * sin(roll) * sin(pitch) + mag_y * cos(roll) - mag_z * sin(roll) * cos(pitch);
    float heading = degrees(atan2(yh, xh));
    return heading;
}

float A_mag[3][3] = {
    {0.984763744413713, 0.04122669970682054, 0.00913880286025970},
    {0.0412266997068205, 1.12509760850787, 0.00334996597668393},
    {0.00913880286025970, 0.00334996597668393, 0.936866818746867}};

float b_mag[3] = {
    -32.710997189811046,
    44.6055600163019,
    -61.7232070724625};

float A_acc[3][3] = {
    {0.9898, -0.002, 0.0018},
    {-0.002, 0.9945, 0.0019},
    {0.0018, 0.0019, 0.9961}};

float b_acc[3] = {
    -0.0588,
    0.0512,
    0.3196};
// float A_mag[3][3] = {
//     {1, 0, 0},
//     {0, 1, 0},
//     {0, 0, 1}};

// float b[3] = {0, 0, 0};

/*
@brief Outputs corrected magnetometer data
@param x: x-axis data in uT
@param y: y-axis data in uT
@param z: z-axis data in uT
*/
CalibratedIMUData calibratedMagData(float x, float y, float z)
{
    CalibratedIMUData data;
    data.x = A_mag[0][0] * x + A_mag[0][1] * y + A_mag[0][2] * z - b_mag[0];
    data.y = A_mag[1][0] * x + A_mag[1][1] * y + A_mag[1][2] * z - b_mag[1];
    data.z = A_mag[2][0] * x + A_mag[2][1] * y + A_mag[2][2] * z - b_mag[2];
    return data;
}

CalibratedIMUData calibratedAccData(float x, float y, float z)
{
    CalibratedIMUData data;
    data.x = A_acc[0][0] * x + A_acc[0][1] * y + A_acc[0][2] * z - b_acc[0];
    data.y = A_acc[1][0] * x + A_acc[1][1] * y + A_acc[1][2] * z - b_acc[1];
    data.z = A_acc[2][0] * x + A_acc[2][1] * y + A_acc[2][2] * z - b_acc[2];
    return data;
}
/*
@brief Function for updating the Hard Iron correction matrix
@param x: x-axis data in uT
@param y: y-axis data in uT
@param z: z-axis data in uT
*/
void HardIronCorrection(float x, float y, float z)
{
    b_mag[0] = (x + b_mag[0]) / 2;
    b_mag[1] = (y + b_mag[1]) / 2;
    b_mag[2] = (z + b_mag[2]) / 2;
}