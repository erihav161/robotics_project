#pragma once
#ifndef joint_hpp
#define joint_hpp
#endif

#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include "lab2/hTransform.hpp"
#include "lab2/chain.hpp"

using namespace Eigen;


class Joint {
    private:
    // data
    public:
    int jointID;
    // Joint* parentJoint;
    // Joint* childJoint;
    double angle;
    Vector3d offset;
    Vector3d rpy;
    HomTrans state;
    // void setParentJoint(Joint& parentJoint);
    // void setChildJoint(Joint& childJoint);
    void updateState(Vector3d rpy, Vector3d offset);

    Joint(Vector3d offset = Vector3d(0, 0, 0), Vector3d rpy = Vector3d(0, 0, 0), int ID = 0);
    ~Joint();

};