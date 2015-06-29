
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

arma::mat learnerDMP (arma::mat y,arma::mat y_1, arma::mat g, arma::mat pq, double az, double bz, double tau, mat D, arma:: vec T) {

    arma::mat data(6, y.n_cols);
    arma::mat eta(3, y.n_cols);
    arma::mat deta(3, y.n_cols);
    arma::mat omega(3, y.n_cols);
    arma::mat domega(3, y.n_cols);



    //for quaternions
    //y= tau * omega
    //y_1= tau * omega'
    // q' = 0.5 * omega * q

    for (int j = 0; j < y.n_cols -1; j++) {

       double * logL= log(tf::Quaternion(pq(3, j), pq(4, j), pq(5, j), pq(6, j)) * tf::Quaternion(pq(3, j + 1), pq(4, j + 1), pq(5, j + 1), pq(6, j + 1)).inverse());
       for (int i = 0; i < 3; i++) omega(i, j) = 2 * logL[i];
    }
    eta = tau * omega;

    for (int j = 1; j < eta.n_cols -1; j++) {

       double * logL= log(tf::Quaternion(pq(3, j), pq(4, j), pq(5, j), pq(6, j)) * tf::Quaternion(pq(3, j + 1), pq(4, j + 1), pq(5, j + 1), pq(6, j + 1)).inverse());
       for (int i = 0; i < 3; i++) domega(i, j) = (omega(i, j + 1) - omega(i, j)) / (T(j+1) - T(j));
    }

    deta = tau * domega;

    for (int j = 0; j < y.n_cols; j++) {
       double * logL= log(tf::Quaternion(g(3, j), g(4, j), g(5, j), g(6, j)) * tf::Quaternion(pq(3, j), pq(4, j), pq(5, j), pq(6, j)).inverse());
      // for (int i = 3; i < 7; i++) data(i, j) = D (i, j) * (tau * y_1 (i, j) - az * (bz * 2 * logL[i-3] - y(i, j)));
       for (int i = 3; i < 7; i++) data(i, j) = D (i, j) * (tau * deta (i-3, j) - az * (bz * 2 * logL[i-3] - eta(i-3, j)));
    }

    return data;
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


