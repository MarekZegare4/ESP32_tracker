#include <Arduino.h>
#include "mag_calibration.h"
//#include <ArduinoEigen.h>

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
float A_1[3][3] = {
    {0.784, 0.0318, -0.0075},
    {0.0318, 0.7886, -0.0168},
    {-0.0075, -0.0168, 0.7407}};

float b[3] = {
    -47.0339,
    36.4925,
    -38.8863};

// float A_1[3][3] = {
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
CalibratedMagData calibratedData(float x, float y, float z)
{
    CalibratedMagData data;
    data.x = A_1[0][0] * x + A_1[0][1] * y + A_1[0][2] * z - b[0];
    data.y = A_1[1][0] * x + A_1[1][1] * y + A_1[1][2] * z - b[1];
    data.z = A_1[2][0] * x + A_1[2][1] * y + A_1[2][2] * z - b[2];
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
    b[0] = (x + b[0]) / 2;
    b[1] = (y + b[1]) / 2;
    b[2] = (z + b[2]) / 2;
}