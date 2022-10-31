#include <iostream>
#include "lab2/hTransform.hpp"
#include <math.h>

#define PI 3.14159265

using namespace Eigen;



void HomTrans::createTMatrix(Vector3d anglesZYX, Vector3d translation) {
    float alpha = anglesZYX(0);
    float beta = anglesZYX(1);
    float gamma = anglesZYX(2);

    if ( alpha > 0 ) {              // Rotation around z-axis
        R = HomTrans::getRotationZ(alpha);
    }
    else if ( beta > 0 ) {          // Rotation around y-axis
        R = HomTrans::getRotationY(beta);        
    }
    else if ( gamma > 0 ) {         // Rotation around x-axis
        R = HomTrans::getRotationX(gamma);
    }
    // else {
    //     std::cout << "Angle vector empty/Angles undefined" << std::endl;
    //     R << 
    //     1, 0, 0,
    //     0, 1, 0,
    //     0, 0, 1;
    // }
    t = translation;
    T <<
    R, t,
    0, 0, 0, 1;
}

void HomTrans::inverseT() {
    invT <<
    R.transpose(), -R.transpose()*t,
    0, 0, 0, 1;
}

Vector4d HomTrans::transformPoint(Vector4d p) {
    Vector4d newPoint;
    newPoint <<
    T*newPoint;
    return newPoint;
}

HomTrans HomTrans::transformFrame(HomTrans frame) {
    HomTrans Tnew;

    Tnew.T = T*frame.T;
    Tnew.R = Tnew.T.block<3,3>(0,0);
    Tnew.t = Tnew.T.block<3,1>(0,3);
    
    return Tnew;
}

Matrix4d HomTrans::identityTransform() {
    Matrix4d iT;
    iT <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

    return iT;
}

void HomTrans::eulerTransform(Vector3d angles, Vector3d translation) {
    // Matrix3d Rz, Ry, Rx, eR;
    // Rz <<
    //     1, 0, 0,
    //     0, 1, 0,
    //     0, 0, 1;
    // Ry <<
    //     1, 0, 0,
    //     0, 1, 0,
    //     0, 0, 1;
    // Rx <<
    //     1, 0, 0,
    //     0, 1, 0,
    //     0, 0, 1;
    // Rz = getRotationZ(angles(2));
    // Ry = getRotationY(angles(1));
    // Rx = getRotationX(angles(0));
    // eR = Rz * Ry * Rx;
    
    Matrix3d eR;
    eR <<
        cos(angles(2))*cos(angles(1)), cos(angles(2))*sin(angles(1))*sin(angles(0)) - sin(angles(2))*cos(angles(0)), cos(angles(2))*sin(angles(1))*cos(angles(0)) + sin(angles(2))*sin(angles(0)),
        sin(angles(2))*cos(angles(1)), sin(angles(2))*sin(angles(1))*sin(angles(0)) + cos(angles(2))*cos(angles(0)), sin(angles(2))*sin(angles(1))*cos(angles(0)) - cos(angles(2))*sin(angles(0)),
        -sin(angles(1)),               cos(angles(1))*sin(angles(0)),                                                cos(angles(1))*cos(angles(0));

    this->T <<
    eR, translation,
    0, 0, 0, 1;

    this->R = eR;
    this->t = translation;
}

Vector3d HomTrans::getRPY() {
    Vector3d rpy;

    rpy <<
    atan2(R(1,0), R(0,0)),
    atan2(-R(2,0), sqrt(pow(R(2,1),2) + pow(R(2,2),2))),
    atan2(R(2,1), R(2,2));

    return rpy;
}

Vector4d HomTrans::toAngleAxis() {
    Vector4d angleAxis;
    double theta = acos((R(0,0) + R(1,1) + R(2,2) -1)/2.0);

    angleAxis <<
    theta,
    (R(2,1) - R(1,2))/(2*sin(theta)),
    (R(0,2) - R(2,0))/(2*sin(theta)),
    (R(1,0) - R(0,1))/(2*sin(theta));

    return angleAxis;
}

void HomTrans::fromAngleAxis(Vector3d axis, double angle) {
    R <<
    pow(axis(0), 2)*(1-cos(angle))+cos(angle), axis(0)*axis(1)*(1-cos(angle))-axis(2)*sin(angle), axis(0)*axis(2)*(1-cos(angle))+axis(2)*sin(angle),
    axis(0)*axis(1)*(1-cos(angle))+axis(0)*sin(angle), pow(axis(1), 2)*(1-cos(angle))+cos(angle), axis(1)*axis(2)*(1-cos(angle))-axis(0)*sin(angle),
    axis(0)*axis(2)*(1-cos(angle))-axis(1)*sin(angle), axis(1)*axis(2)*(1-cos(angle))+axis(1)*sin(angle), axis(2)*axis(2)*(1-cos(angle))+cos(angle);
}

Vector4d HomTrans::toQuaternion() {
    Vector4d angleAxis;
    Vector4d q;
    // double eta;
    // eta = 1/2*sqrt(R(0,0) + R(1,1) + R(2,2) + 1);
    // q <<
    // eta,
    // ((R(2,1)-R(1,2))>0)-((R(2,1)-R(1,2))<0)*sqrt(R(0,0) - R(1,1) - R(2,2) + 1),
    // ((R(0,2)-R(2,0))>0)-((R(0,2)-R(2,0))<0)*sqrt(R(0,0) - R(1,1) - R(2,2) + 1),
    // ((R(1,0)-R(0,1))>0)-((R(1,0)-R(0,1))<0)*sqrt(R(0,0) - R(1,1) - R(2,2) + 1);
    angleAxis = this->toAngleAxis();
    q <<
    cos(angleAxis(0) / 2),
    sin(angleAxis(0) / 2) * angleAxis(1),
    sin(angleAxis(0) / 2) * angleAxis(2),
    sin(angleAxis(0) / 2) * angleAxis(3);
    return q;
}

void HomTrans::fromQuaternion(Vector4d q) {
    R <<
    2*(q(3)*q(3) + q(0)*q(0)) - 1, 2*(q(0)*q(1) - q(3)*q(2)), 2*(q(0)*q(2) + q(3)*q(1)),
    2*(q(0)*q(1) + q(3)*q(2)), 2*(q(3)*q(3) + q(1)*q(1)) - 1, 2*(q(1)*q(2) - q(3)*q(0)),
    2*(q(0)*q(2) - q(3)*q(1)), 2*(q(1)*q(2) + q(3)*q(0)), 2*(q(3)*q(3) + q(2)*q(2)) - 1;            
}

Matrix3d HomTrans::getRotationX(double gamma) {
    Matrix3d R;
    R <<
    1,          0,           0,
    0, cos(gamma), -sin(gamma),
    0, sin(gamma),  cos(gamma);

    return R;
}
Matrix3d HomTrans::getRotationY(double beta) {
    Matrix3d R;
    R <<
    cos(beta),  0, sin(beta),
            0,  1,         0,
    -sin(beta), 0, cos(beta);

    return R;
}
Matrix3d HomTrans::getRotationZ(double alpha) {
    Matrix3d R;
    R <<
    cos(alpha), -sin(alpha), 0,
    sin(alpha),  cos(alpha), 0,
             0,           0, 1;

    return R;
}


HomTrans::HomTrans(){}

HomTrans::~HomTrans(){}

