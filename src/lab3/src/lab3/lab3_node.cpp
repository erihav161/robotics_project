#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <ros/ros.h>
#include <kdl/tree.hpp>
#include "lab3/KDLSubscriber.hpp"

using namespace KDL;

int main(int argc, char** argv) {
    // KDL::Tree my_tree;
    // ros::NodeHandle nh;
    // std::string robot_description;
    // nh.param("robot_description", robot_description, std::string());
    // if (!kdl_parser::treeFromString(robot_description, my_tree)) {
    //     ROS_ERROR("Failed to construct KDL tree");
    //     return false;
    // }
    // unsigned int nj = my_tree.getNrOfJoints();
    // unsigned int js = my_tree.getNrOfSegments();
    // ROS_INFO("NrOfJoints =%d   ||  NrOfSegments= %d \n", nj,js);
    ros::init(argc, argv, "lab3");
    KDLSubscriber *sub = new KDLSubscriber();
    ros::spin();
    return 0;
}