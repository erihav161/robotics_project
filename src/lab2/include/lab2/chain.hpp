#pragma once
#ifndef chain_hpp
#define chain_hpp
#endif

#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include "lab2/hTransform.hpp"
#include "lab2/joint.hpp"

using namespace Eigen;

class Joint;

class Chain {
    private:
    // data
    public:
    std::vector<Joint*> jointList;
    void addJoint(Vector3d offset, Vector3d rpy);
    void printJoints();
    Matrix4d getTransform();
    void updateJointAngle(int jointID, double angle);


    Chain();
    ~Chain();

};