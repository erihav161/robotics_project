#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdlib.h>
#include <time.h>
#include "lab2/hTransform.hpp"
#include "lab2/joint.hpp"
#include "lab2/chain.hpp"

using namespace std;
using namespace Eigen;


int main(int argc, char** argv) {
    cout << "Kinematics node running... " << endl << endl;

    ros::init(argc, argv, "tf2_publisher");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped endEffectorFrame;


    Vector3d link_lengths;
    Matrix4d endFrame;
    link_lengths <<
        0.5,
        0.3,
        0.2;
    
    Chain c;

    c.addJoint(Vector3d(0, 0, 0.05), Vector3d(0, 0, (double)M_PI_4));
    c.addJoint(Vector3d(link_lengths(0), 0, 0), Vector3d(0, 0, (double)-M_PI_4/2));
    c.addJoint(Vector3d(link_lengths(1), 0, 0), Vector3d(0, 0, (double)-M_PI_4/2));
    c.addJoint(Vector3d(link_lengths(2), 0, 0), Vector3d(0, 0, 0));

    c.printJoints();
    endFrame = c.getTransform();
    cout << "End effector transform: " << endl << endFrame << endl << endl;

    cout << "Kinematics node terminating. " << endl;
    return 0;
}