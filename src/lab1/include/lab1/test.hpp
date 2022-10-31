#ifndef test_hpp
#define test_hpp
#include "hTransform.hpp"
#include <iostream>

class TestHomTrans {
    private:
    // data
    public:
        HomTrans T1;
        HomTrans T2;
        Eigen::Transform<double, 3, Eigen::Affine> eT1;
        Eigen::Transform<double, 3, Eigen::Affine> eT2;
        Eigen::Vector3d point1;
        Eigen::Vector3d point2;
        Eigen::Vector3d point3;



        int testInverse();
        int testPointTransform();
        int testFrameTransform();
        int testAngleAxis();
        int testQuaternion();

        void testAll();

        TestHomTrans();
        ~TestHomTrans();
};

#endif