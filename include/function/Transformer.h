#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

/**
  * @author YuelinZOU, Tianjin University.
  */

namespace transform {
    double DegToRad(double angle);
    double RadToDeg(double angle);

    Eigen::Vector3d QuaternionToEulerAngles(Eigen::Quaterniond q);
    Eigen::Matrix3d QuaternionToMatrix(Eigen::Quaterniond q);

    Eigen::Quaterniond EulerAngle2Quat(double rx, double ry, double rz);
    Eigen::Quaterniond EulerAngle2Quat(Eigen::Vector3d angle);
    Eigen::Matrix3d EulerAngle2Matrix(double rx, double ry, double rz);
    Eigen::Matrix3d EulerAngle2Matrix(Eigen::Vector3d angle);

    Eigen::Vector3d MatrixToEulerAngles(Eigen::Matrix3d mat);
    Eigen::Quaterniond MatrixToQuaternion(Eigen::Matrix3d mat);

}

#endif // TRANSFORMER_H
