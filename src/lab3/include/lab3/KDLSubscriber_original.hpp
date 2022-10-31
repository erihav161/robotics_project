#ifndef KDLSubscriber_hpp
#define KDLSubscriber_hpp

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class KDLSubscriber {
    private:
    // data
    public:
    KDL::Tree my_tree;
    KDL::TreeFkSolverPos_recursive *fkSolver;
    ros::NodeHandle node;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    KDL::TreeJntToJacSolver *jacobiSolver;
    KDL::JntArray qc;
    sensor_msgs::JointState jointStateMessage;

    float randomGuess();
    void randomPose();
    void callBack(const sensor_msgs::JointState &jointState);
    void solveIK();
    void calculateEndFrame(const sensor_msgs::JointState &jointState);
    Eigen::VectorXd calculateEndFrame(KDL::JntArray q);
    Eigen::MatrixXd calcualteJacobian(const sensor_msgs::JointState &jointState);
    Eigen::MatrixXd calcualteJacobian(KDL::JntArray q);

    KDLSubscriber();
    ~KDLSubscriber();
   




};


#endif