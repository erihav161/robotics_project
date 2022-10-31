#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdlib.h>
#include <time.h>
#include "lab1/hTransform.hpp"
#include "lab1/test.hpp"



tf2::Quaternion getQuaternion(geometry_msgs::TransformStamped* t) {
  tf2::Quaternion Q;

  Q.setX(t->transform.rotation.x);
  Q.setY(t->transform.rotation.y);
  Q.setZ(t->transform.rotation.z);
  Q.setW(t->transform.rotation.w);

  return Q;
}


int main(int argc, char** argv){
  TestHomTrans Tester;
  Tester.T1.createTMatrix(Eigen::Vector3d(M_PI_2, 0, 0), Eigen::Vector3d(5, 0, 0));
  Tester.T1.createTMatrix(Eigen::Vector3d(M_PI_4, 0, 0), Eigen::Vector3d(10, 0, 0));
  Tester.eT1.matrix() = Tester.T1.T;
  Tester.eT2.matrix() = Tester.T2.T;
  std::cout << "HT: " << std::endl << Tester.T1.T << std::endl << std::endl;
  std::cout << "ET: " << std::endl << Tester.eT1.matrix() << std::endl << std::endl; 
  Tester.testAll();

  srand(time(NULL));
  ros::init(argc, argv, "tf2_publisher");
  ros::NodeHandle nh;
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped frame1, frame2;
  tf2::Quaternion Q1, Q2;
  // ros::Publisher topic_pub = nh.advertise<std_msgs::String>("tutorial", 1000);


  frame1.header.frame_id = "world";
  frame1.child_frame_id = "Frame 1";

  frame2.header.frame_id = "world";
  frame2.child_frame_id = "Frame 2";

  // Random frame generator

  // frame1.transform.translation.x = ((double) rand()/RAND_MAX);
  // frame1.transform.translation.y = ((double) rand()/RAND_MAX);
  // frame1.transform.translation.z = ((double) rand()/RAND_MAX);
  // Q1.setRPY(rand() % 6, rand() % 6, rand() % 6);
  // frame1.transform.rotation.x = Q1.x();
  // frame1.transform.rotation.y = Q1.y();
  // frame1.transform.rotation.z = Q1.z();
  // frame1.transform.rotation.w = Q1.w();
  
  // frame2.transform.translation.x = ((double) rand()/RAND_MAX);
  // frame2.transform.translation.y = ((double) rand()/RAND_MAX);
  // frame2.transform.translation.z = ((double) rand()/RAND_MAX);
  // Q2.setRPY(rand() % 6, rand() % 6, rand() % 6);
  // frame2.transform.rotation.x = Q2.x();
  // frame2.transform.rotation.y = Q2.y();
  // frame2.transform.rotation.z = Q2.z();
  // frame2.transform.rotation.w = Q2.w();

  // Easy fixed frames
  frame1.transform.translation.x = 1;
  frame1.transform.translation.y = 1;
  frame1.transform.translation.z = 1;

  
  frame2.transform.translation.x = 2;
  frame2.transform.translation.y = 2;
  frame2.transform.translation.z = 2;


  HomTrans F0_1, F1_0, F0_2, F1_2;
  Eigen::Vector4d q1, q2, angleAxis;
  Eigen::Vector3d t1, t2, empty;

  empty <<
    0,
    0,
    0;
  
  q1 <<
    0,
    0,
    0.9797958971,
    0.2;
  frame1.transform.rotation.x = q1(0);
  frame1.transform.rotation.y = q1(1);
  frame1.transform.rotation.z = q1(2);
  frame1.transform.rotation.w = q1(3);
  
  
  q2 <<
    0,
    0,
    0.866,
    0.5;
  frame2.transform.rotation.x = q2(0);
  frame2.transform.rotation.y = q2(1);
  frame2.transform.rotation.z = q2(2);
  frame2.transform.rotation.w = q2(3);

  t1 <<
    frame1.transform.translation.x,
    frame1.transform.translation.y,
    frame1.transform.translation.z;
  
  t2 <<
    frame2.transform.translation.x,
    frame2.transform.translation.y,
    frame2.transform.translation.z;

  F0_1.fromQuaternion(q1);
  F0_1.createTMatrix(empty, t1);
  F0_1.inverseT();
  
  F1_0.T = F0_1.invT;

  F0_2.fromQuaternion(q2);
  F0_2.createTMatrix(empty, t2);

  F1_2 = F1_0.transformFrame(F0_2);
  
  
  std::cout << "Transform from frame 0 to frame 1: " << std::endl;
  std::cout << F0_1.T << std::endl << std::endl;
  std::cout << "Transform from frame 1 to frame 0: " << std::endl;
  std::cout << F1_0.T << std::endl << std::endl;
  std::cout << "Transform from frame 0 to frame 2: " << std::endl;
  std::cout << F0_2.T << std::endl << std::endl;
  std::cout << "Transform from frame 1 to frame 2: " << std::endl;
  std::cout << F1_2.T << std::endl << std::endl;
  std::cout << "Transform F0_1 * F1_2: " << std::endl;
  std::cout << F0_1.T * F1_2.T << std::endl << std::endl;
  std::cout << "Frame 1 quaternion: " << std::endl;
  std::cout << "x: " << getQuaternion(&frame1).x() << std::endl;
  std::cout << "y: " << getQuaternion(&frame1).y() << std::endl; 
  std::cout << "z: " << getQuaternion(&frame1).z() << std::endl;
  std::cout << "w: " << getQuaternion(&frame1).w() << std::endl << std::endl;
  
  angleAxis = F1_2.toAngleAxis();
  tf2Scalar angle = angleAxis(0) / 100;
  tf2::Vector3 vec = tf2::Vector3((float)angleAxis(1), (float)angleAxis(2), (float)angleAxis(3));

  tf2::Quaternion q1_2;
  tf2::Quaternion q0_1 = getQuaternion(&frame1);
  tf2::Quaternion resultingQuaternion;

  Eigen::Vector3d deltaTrans;
  deltaTrans = (t2 - t1) / 100;

  tf2Scalar deltaAngle = angleAxis(0) / 100;

  int count = 0;


  ros::Rate loop_rate(10.0);

  while(nh.ok()) {
    frame1.header.stamp = ros::Time::now();
    frame2.header.stamp = ros::Time::now();

    std::cout << "Transform 1 to 2 axis vector and angle: " << std::endl;
    std::cout << "x: " << vec.x() << std::endl;
    std::cout << "y: " << vec.y() << std::endl; 
    std::cout << "z: " << vec.z() << std::endl;
    std::cout << "angle: " << angle << std::endl << std::endl; 
    q1_2.setRotation(vec, angle);
    std::cout << "Transform 1 to 2 quaternion: " << std::endl;
    std::cout << "x: " << q1_2.x() << std::endl;
    std::cout << "y: " << q1_2.y() << std::endl; 
    std::cout << "z: " << q1_2.z() << std::endl;
    std::cout << "w: " << q1_2.w() << std::endl << std::endl; 
    
    if (count < 100) {

      frame1.transform.translation.x += deltaTrans(0);
      frame1.transform.translation.y += deltaTrans(1);
      frame1.transform.translation.z += deltaTrans(2);

      resultingQuaternion = q0_1 * q1_2;
      std::cout << "Resulting quaternion: " << std::endl;
      std::cout << "x: " << resultingQuaternion.x() << std::endl;
      std::cout << "y: " << resultingQuaternion.y() << std::endl; 
      std::cout << "z: " << resultingQuaternion.z() << std::endl;
      std::cout << "w: " << resultingQuaternion.w() << std::endl << std::endl;

      frame1.transform.rotation.x = resultingQuaternion.x();
      frame1.transform.rotation.y = resultingQuaternion.y();
      frame1.transform.rotation.z = resultingQuaternion.z();
      frame1.transform.rotation.w = resultingQuaternion.w();
      std::cout << "Frame 1 quaternion: " << std::endl;
      std::cout << "x: " << getQuaternion(&frame1).x() << std::endl;
      std::cout << "y: " << getQuaternion(&frame1).y() << std::endl; 
      std::cout << "z: " << getQuaternion(&frame1).z() << std::endl;
      std::cout << "w: " << getQuaternion(&frame1).w() << std::endl << std::endl;

      br.sendTransform(frame1);
      br.sendTransform(frame2);
      angle += deltaAngle;

      count++;

    }
    loop_rate.sleep();
  }
  return 0;
};