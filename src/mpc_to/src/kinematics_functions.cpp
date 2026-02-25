#include <math.h>
#include <eigen3/Eigen/Dense>

Eigen::MatrixXf get_cpose(float q1, float q2, float q3, float q4, float q5, float q6){
Eigen::MatrixXf mat(3,7);
mat <<
-0.0679*sin(q1),0.2125*cos(q1)*cos(q2) - 0.13585*sin(q1),0.425*cos(q1)*cos(q2) - 0.08405*sin(q1),0.425*cos(q1)*cos(q2) - 0.01615*sin(q1) + 0.1294*cos(q1)*cos(q2)*cos(q3) - 0.1294*cos(q1)*sin(q2)*sin(q3),0.425*cos(q1)*cos(q2) - 0.01615*sin(q1) + 0.2589*cos(q1)*cos(q2)*cos(q3) - 0.2589*cos(q1)*sin(q2)*sin(q3),0.425*cos(q1)*cos(q2) - 0.06265*sin(q1) + 0.39225*cos(q1)*cos(q2)*cos(q3) - 0.39225*cos(q1)*sin(q2)*sin(q3),0.425*cos(q1)*cos(q2) - 0.10915*sin(q1) - 0.0411*cos(q5)*sin(q1) + 0.0411*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - 0.09465*cos(q2 + q3)*cos(q1)*sin(q4) - 0.09465*sin(q2 + q3)*cos(q1)*cos(q4) + 0.39225*cos(q1)*cos(q2)*cos(q3) - 0.39225*cos(q1)*sin(q2)*sin(q3),
0.0679*cos(q1),0.13585*cos(q1) + 0.2125*cos(q2)*sin(q1),0.08405*cos(q1) + 0.425*cos(q2)*sin(q1),0.01615*cos(q1) + 0.425*cos(q2)*sin(q1) - 0.1294*sin(q1)*sin(q2)*sin(q3) + 0.1294*cos(q2)*cos(q3)*sin(q1),0.01615*cos(q1) + 0.425*cos(q2)*sin(q1) - 0.2589*sin(q1)*sin(q2)*sin(q3) + 0.2589*cos(q2)*cos(q3)*sin(q1),0.06265*cos(q1) + 0.425*cos(q2)*sin(q1) - 0.39225*sin(q1)*sin(q2)*sin(q3) + 0.39225*cos(q2)*cos(q3)*sin(q1),0.10915*cos(q1) + 0.0411*cos(q1)*cos(q5) + 0.425*cos(q2)*sin(q1) - 0.39225*sin(q1)*sin(q2)*sin(q3) + 0.0411*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - 0.09465*cos(q2 + q3)*sin(q1)*sin(q4) - 0.09465*sin(q2 + q3)*cos(q4)*sin(q1) + 0.39225*cos(q2)*cos(q3)*sin(q1),
0.089159000000000002139621813057602,0.089159000000000002139621813057602 - 0.2125*sin(q2),0.089159000000000002139621813057602 - 0.425*sin(q2),0.089159000000000002139621813057602 - 0.425*sin(q2) - 0.1294*sin(q2 + q3),0.089159000000000002139621813057602 - 0.425*sin(q2) - 0.2589*sin(q2 + q3),0.089159000000000002139621813057602 - 0.425*sin(q2) - 0.39225*sin(q2 + q3),0.09465*sin(q2 + q3)*sin(q4) - 0.425*sin(q2) - sin(q5)*(0.0411*cos(q2 + q3)*sin(q4) + 0.0411*sin(q2 + q3)*cos(q4)) - 0.09465*cos(q2 + q3)*cos(q4) - 0.39225*sin(q2 + q3) + 0.089159000000000002139621813057602;
    return mat;
}



Eigen::MatrixXf get_velocity(float q1, float q2, float q3, float q4, float q5, float q6,
                             float u_1, float u_2, float u_3, float u_4, float u_5, float u_6){
    Eigen::MatrixXf mat(21,1);
mat <<
-0.0679*u_1*cos(q1),
-0.0679*u_1*sin(q1),
0,
- 1.0*u_1*(0.13585*cos(q1) + 0.2125*cos(q2)*sin(q1)) - 0.2125*u_2*cos(q1)*sin(q2),
- 1.0*u_1*(0.13585*sin(q1) - 0.2125*cos(q1)*cos(q2)) - 0.2125*u_2*sin(q1)*sin(q2),
-0.2125*u_2*cos(q2),
- 1.0*u_1*(0.08405*cos(q1) + 0.425*cos(q2)*sin(q1)) - 0.425*u_2*cos(q1)*sin(q2),
- 1.0*u_1*(0.08405*sin(q1) - 0.425*cos(q1)*cos(q2)) - 0.425*u_2*sin(q1)*sin(q2),
-0.425*u_2*cos(q2),
- 1.0*u_1*(0.01615*cos(q1) + 0.425*cos(q2)*sin(q1) - 0.1294*sin(q1)*sin(q2)*sin(q3) + 0.1294*cos(q2)*cos(q3)*sin(q1)) - 0.1294*u_3*sin(q2 + q3)*cos(q1) - 0.0002*u_2*cos(q1)*(647.0*sin(q2 + q3) + 2125.0*sin(q2)),
- 1.0*u_2*(0.425*sin(q1)*sin(q2) + 0.1294*cos(q2)*sin(q1)*sin(q3) + 0.1294*cos(q3)*sin(q1)*sin(q2)) - 1.0*u_1*(0.01615*sin(q1) - 0.425*cos(q1)*cos(q2) - 0.1294*cos(q1)*cos(q2)*cos(q3) + 0.1294*cos(q1)*sin(q2)*sin(q3)) - 1.0*u_3*(0.1294*cos(q2)*sin(q1)*sin(q3) + 0.1294*cos(q3)*sin(q1)*sin(q2)),
- 1.0*u_2*(0.1294*cos(q2 + q3) + 0.425*cos(q2)) - 0.1294*u_3*cos(q2 + q3),
- 1.0*u_1*(0.01615*cos(q1) + 0.425*cos(q2)*sin(q1) - 0.2589*sin(q1)*sin(q2)*sin(q3) + 0.2589*cos(q2)*cos(q3)*sin(q1)) - 0.2589*u_3*sin(q2 + q3)*cos(q1) - 0.0001*u_2*cos(q1)*(2589.0*sin(q2 + q3) + 4250.0*sin(q2)),
- 1.0*u_2*(0.425*sin(q1)*sin(q2) + 0.2589*cos(q2)*sin(q1)*sin(q3) + 0.2589*cos(q3)*sin(q1)*sin(q2)) - 1.0*u_1*(0.01615*sin(q1) - 0.425*cos(q1)*cos(q2) - 0.2589*cos(q1)*cos(q2)*cos(q3) + 0.2589*cos(q1)*sin(q2)*sin(q3)) - 1.0*u_3*(0.2589*cos(q2)*sin(q1)*sin(q3) + 0.2589*cos(q3)*sin(q1)*sin(q2)),
- 1.0*u_2*(0.2589*cos(q2 + q3) + 0.425*cos(q2)) - 0.2589*u_3*cos(q2 + q3),
- 1.0*u_1*(0.06265*cos(q1) + 0.425*cos(q2)*sin(q1) - 0.39225*sin(q1)*sin(q2)*sin(q3) + 0.39225*cos(q2)*cos(q3)*sin(q1)) - 0.00025*u_2*cos(q1)*(1569.0*sin(q2 + q3) + 1700.0*sin(q2)) - 0.39225*u_3*sin(q2 + q3)*cos(q1),
- 1.0*u_2*(0.425*sin(q1)*sin(q2) + 0.39225*cos(q2)*sin(q1)*sin(q3) + 0.39225*cos(q3)*sin(q1)*sin(q2)) - 1.0*u_1*(0.06265*sin(q1) - 0.425*cos(q1)*cos(q2) - 0.39225*cos(q1)*cos(q2)*cos(q3) + 0.39225*cos(q1)*sin(q2)*sin(q3)) - 1.0*u_3*(0.39225*cos(q2)*sin(q1)*sin(q3) + 0.39225*cos(q3)*sin(q1)*sin(q2)),
- 1.0*u_2*(0.39225*cos(q2 + q3) + 0.425*cos(q2)) - 0.39225*u_3*cos(q2 + q3),
u_5*(0.0411*sin(q1)*sin(q5) + 0.0411*cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - 1.0*u_1*(0.10915*cos(q1) + 0.0411*cos(q1)*cos(q5) + 0.425*cos(q2)*sin(q1) - 0.39225*sin(q1)*sin(q2)*sin(q3) + 0.0411*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - 0.09465*cos(q2 + q3)*sin(q1)*sin(q4) - 0.09465*sin(q2 + q3)*cos(q4)*sin(q1) + 0.39225*cos(q2)*cos(q3)*sin(q1)) - 0.00015*u_4*cos(q1)*(274.0*sin(q2 + q3 + q4)*sin(q5) + 631.0*cos(q2 + q3)*cos(q4) - 631.0*sin(q2 + q3)*sin(q4)) - 0.00015*u_3*cos(q1)*(631.0*cos(q2 + q3 + q4) + 137.0*cos(q2 + q3 + q4 - 1.0*q5) - 137.0*cos(q2 + q3 + q4 + q5) + 2615.0*sin(q2 + q3)) - 0.00005*u_2*cos(q1)*(1893.0*cos(q2 + q3 + q4) + 411.0*cos(q2 + q3 + q4 - 1.0*q5) - 411.0*cos(q2 + q3 + q4 + q5) + 7845.0*sin(q2 + q3) + 8500.0*sin(q2)),
- 1.0*u_1*(0.10915*sin(q1) - 0.425*cos(q1)*cos(q2) + 0.0411*cos(q5)*sin(q1) - 0.0411*cos(q2 + q3 + q4)*cos(q1)*sin(q5) + 0.09465*cos(q2 + q3)*cos(q1)*sin(q4) + 0.09465*sin(q2 + q3)*cos(q1)*cos(q4) - 0.39225*cos(q1)*cos(q2)*cos(q3) + 0.39225*cos(q1)*sin(q2)*sin(q3)) - 0.0411*u_5*(cos(q1)*sin(q5) - 1.0*cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - 0.00015*u_4*sin(q1)*(274.0*sin(q2 + q3 + q4)*sin(q5) + 631.0*cos(q2 + q3)*cos(q4) - 631.0*sin(q2 + q3)*sin(q4)) - 0.00015*u_3*sin(q1)*(631.0*cos(q2 + q3 + q4) + 137.0*cos(q2 + q3 + q4 - 1.0*q5) - 137.0*cos(q2 + q3 + q4 + q5) + 2615.0*sin(q2 + q3)) - 0.00005*u_2*sin(q1)*(1893.0*cos(q2 + q3 + q4) + 411.0*cos(q2 + q3 + q4 - 1.0*q5) - 411.0*cos(q2 + q3 + q4 + q5) + 7845.0*sin(q2 + q3) + 8500.0*sin(q2)),
u_4*(0.09465*sin(q2 + q3 + q4) - 0.0411*cos(q2 + q3 + q4)*sin(q5)) - 1.0*u_3*(0.39225*cos(q2 + q3) - 0.09465*sin(q2 + q3 + q4) + 0.0411*cos(q2 + q3 + q4)*sin(q5)) - 1.0*u_2*(0.39225*cos(q2 + q3) + 0.425*cos(q2) - 0.09465*cos(q2 + q3)*sin(q4) - 0.09465*sin(q2 + q3)*cos(q4) + 0.0411*cos(q2 + q3)*cos(q4)*sin(q5) - 0.0411*sin(q2 + q3)*sin(q4)*sin(q5)) - 0.0411*u_5*sin(q2 + q3 + q4)*cos(q5);
    return mat;
}
