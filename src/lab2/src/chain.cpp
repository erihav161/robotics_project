#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include "lab2/chain.hpp"
#include "lab2/joint.hpp"
#include "lab2/hTransform.hpp"


using namespace Eigen;
using namespace std;

Chain::Chain() {

}

void Chain::addJoint(Vector3d offset, Vector3d rpy) {
    int jointID = this->jointList.size();
    Joint* j = new Joint(offset, rpy, jointID);
    this->jointList.push_back(j);
}

void Chain::printJoints() {
    for (Joint* i : this->jointList) {
        // std::cout << "Joint: " << i.jointID << " with parent " << i.parentJoint.jointID << " and child joint " << i.childJoint.jointID << std::endl;
        cout << "Joint: " << i->jointID << endl;
        cout << "with offset: " << endl << i->offset << endl;
        cout << "and orientation: " << endl << i->rpy <<  endl << endl;
        cout << "Transform: " << endl << i->state.T << endl << endl;
        cout << &i->state.T << endl << endl;
    }
    cout << endl;
}

Matrix4d Chain::getTransform() {
    HomTrans updatedFrame;
    Vector3d rpy, currentTranslation;  
    Matrix4d totalTransform;
    totalTransform <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    for (Joint* i : this->jointList) {
        rpy = i->state.getRPY();
        cout << "RPY: " << endl << rpy << endl;
        rpy = Vector3d(rpy(2), rpy(1), rpy(0) + i->angle);
        cout << "Updated RPY: " << endl << rpy << endl;

        currentTranslation = i->state.T.block<3,1>(0,3);
        cout << "Translation: " << endl << currentTranslation << endl;

        updatedFrame.eulerTransform(rpy, currentTranslation);
        cout << "Applied frame: " << endl << updatedFrame.T << endl << endl;

        totalTransform = totalTransform * updatedFrame.T;
        cout << "Total transform: " << i->jointID << endl << totalTransform << endl;

    }
    return totalTransform;
}

void Chain::updateJointAngle(int jointID, double angle) {
    // HomTrans h;
    // h.eulerTransform(Vector3d(0, 0, angle), Vector3d(0, 0, 0));
    // this->jointList.at(jointID)->state = this->jointList.at(jointID)->state.transformFrame(h);
    this->jointList.at(jointID)->angle = angle;
    }

Chain::~Chain() {}



