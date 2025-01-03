#include <iostream>
#include <Eigen/Dense>

using vector3f = Eigen::Vector3f;
using matrix3f = Eigen::Matrix3f;
using vector4f = Eigen::Vector3f;
using matrix4f = Eigen::Matrix3f;
using vector6f = Eigen::Vector6f;
using matrix6f = Eigen::Matrix6f;

matrix3f getSkewSymMatrix(const vector3f& vec) { 
    // Create the skew-symmetric matrix
    matrix3f skewMatrix;
    skewMatrix <<  0,       -vec.z(),  vec.y(),
                   vec.z(),  0,       -vec.x(),
                  -vec.y(),  vec.x(),  0;
    return skewMatrix;
}

matrix3f getSO3ExpMap(const vector3f& omega, double theta){
    matrix3f omega_hat = getSkewSymMatrix(omega);
    matrix3f exp_matrix = matrix3f::Identity() +
                          sin(theta) * omega_hat +
                          (1 - cos(theta)) * omega_hat * omega_hat;
    return exp_matrix;
}

matrix4f getSE3ExpMap(const vector6f& twist, double theta){
    vector3f omega << twist(0),twist(1),twist(2);
    vector3f velocity << twist(3),twist(4),twist(5);
    matrix3f omega_hat = getSkewSymMatrix(omega);
    matrix3f rotation = getSO3ExpMap(omega,theta);
    vector3f translation = (matrix3f::Identity() +
                          (theta-sin(theta)) * omega_hat * omega_hat +
                          (1 - cos(theta)) * omega_hat)*velocity;
    matrix4f transformation = matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = rotation;  // Top-left 3x3 block is the rotation
    transformation.block<3, 1>(0, 3) = translation;  // Top-right 3x1 block is the translation
    return transformation;
}

int main() {
    vector3f vec(1.0f, 2.0f, 3.0f);  // Example vector
    matrix3f skewMatrix = getSkewSymMatrix(vec);

    std::cout << "Skew-symmetric matrix:\n" << skewMatrix << std::endl;
    return 0;
}
