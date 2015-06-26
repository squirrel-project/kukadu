
#include <cstdio>
#include <iostream>
#include <fstream>

#include "../include/kukadu.h"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>

#define PI 3.14

double * log(const tf::Quaternion quat);
tf::Quaternion exp(const double* logQuat);
double distQuat(tf::Quaternion q1, tf::Quaternion q2);

int main(int argc, char** args) {
    return 0;
}

void func(double t, const double* y, double* f, void* params){
    // TODO: remove equation for z' and merge the first two equations
    // y' = z / tau
    // z' = 1 / tau * ( az * (bz * (g - y) - z) + f);
    // x' = -ax / tau * x

    for(int i = 0; i < odeSystemSizeMinOne / 2; i = i + 2) {

        double yPlusOne = y[i + 1];

        int currentSystem = (int) (i / 2);
        f[i] = yPlusOne * oneDivTau;
        double g = gs(currentSystem);
        arma::vec currentCoeffs = dmpCoeffs.at(currentSystem);

        if(t <= (dmp.getTmax() - 1)) {

            double addTerm = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
            f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm)  + this->addTerm(t, y, i / 2, controlQueue);

        } else {
            f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne));
        }

    }

    double logG = log(tf::Quaternion(gs(3), gs(4), gs(5), gs(6)) * tf::Quaternion(y[6], y[8], y[10], y[12]).inverse());




}


double * log(tf::Quaternion quat) {

    double logQuat[3];
    double modU = sqrt(quat.x() * quat.x()  + quat.y() * quat.y() + quat.z() * quat.z());

    if (modU > 0) {

        logQuat[0]= quat.x() / modU;
        logQuat[1] = quat.y() / modU;
        logQuat[2] = quat.z() / modU;

    } else {

        logQuat[0] = 0.0;
        logQuat[1] = 0.0;
        logQuat[2] = 0.0;

    }

    return logQuat;

}

tf::Quaternion exp(const double* logQuat) {

    double x, y, z, w;
    double modR = sqrt(logQuat[0] * logQuat[0] + logQuat[1] * logQuat[1] + logQuat[2] * logQuat[2]);

    if (modR > 0) {

        w = cos(modR);
        x = sin(modR) * logQuat[0] / modR;
        y = sin(modR) * logQuat[1] / modR;
        z = sin(modR) * logQuat[2] / modR;

    } else {

        w = 0.0;
        x = 0.0;
        y = 0.0;
        z = 0.0;

    }

    return tf::Quaternion(x, y, z, w);

}


double distQuat(tf::Quaternion q1, tf::Quaternion q2){

    double d;
    const tf::Quaternion q = q1 * q2.inverse();
    double *logQuat = log(q);

    if ((q.x() == 0) && (q.y() == 0) && (q.z() == 0) && (q.w() == -1)) d = 2 * PI;

    else {

        d = 2 * sqrt(logQuat[0] * logQuat[0] + logQuat[1] * logQuat[1] + logQuat[2] * logQuat[2]);

    }

    return d;

}


/*geometry_msgs::Vector3 log(const geometry_msgs::Quaternion quat) {

    geometry_msgs::Vector3 logQuat;
    double modU = sqrt(quat.x*quat.x  + quat.y*quat.y + quat.z*quat.z);

    if (modU > 0) {

        logQuat.x = quat.x/modU;
        logQuat.y = quat.y/modU;
        logQuat.z = quat.z/modU;

    } else {

        logQuat.x = 0.0;
        logQuat.y = 0.0;
        logQuat.z = 0.0;

    }

    return logQuat;

}




geometry_msgs::Quaternion exp(geometry_msgs::Vector3 logQuat) {

    geometry_msgs::Quaternion quat;
    double modR = sqrt(logQuat.x*logQuat.x + logQuat.y*logQuat.y + logQuat.z*logQuat.z);

    if (modR > 0) {

        quat.w = cos(modR);
        quat.x = sin(modR)*logQuat.x/modR;
        quat.y = sin(modR)*logQuat.y/modR;
        quat.z = sin(modR)*logQuat.z/modR;

    } else {

        quat.w = 0.0;
        quat.x = 0.0;
        quat.y = 0.0;
        quat.z = 0.0;

    }

    return quat;

}*/


