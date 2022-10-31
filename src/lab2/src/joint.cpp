#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include "lab2/hTransform.hpp"
#include "lab2/joint.hpp"


using namespace Eigen;

// void Joint::setParentJoint(Joint* parent) {
//     this->parentJoint = parent;
//     parent->childJoint = this;
// }

// void Joint::setChildJoint(Joint& child) {
//     this->childJoint = *child;
//     child->parentJoint = this;
// }


// Makes a new transformation object from the angle and offset, transforms the old state by the new

 Joint::Joint(Vector3d offset, Vector3d rpy, int ID) {
        this->offset = offset;
        this->rpy = rpy;
        this->jointID = ID;
        this->state.eulerTransform(rpy, offset);
    }
void Joint::updateState(Vector3d rpy, Vector3d offset) {
    HomTrans update;
    update.eulerTransform(rpy, offset);
    this->state = this->state.transformFrame(update);
}

Joint::~Joint(){}

