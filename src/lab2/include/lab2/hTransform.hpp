#pragma once
#ifndef hTransform_hpp
#define hTransform_hpp


#include <Eigen/Dense>

using namespace Eigen;

class HomTrans {
    private:
    // data
    public:
        Matrix4d T;                     // Four dimensional homogeneous transformation matrix
        Matrix3d R;                     // Three dimensional rotation matrix
        Vector3d t;                     // Three dimensional translation vector
        Matrix4d invT;                  // Inverse matrix of T


        void createTMatrix(Vector3d anglesZYX, Vector3d translation);   // AnglesZYX should be a 3D vector with angles phi, theta, psi
        void inverseT();
        Vector4d transformPoint(Vector4d p);
        HomTrans transformFrame(HomTrans frame);
        Matrix4d identityTransform();
        void eulerTransform(Vector3d angles, Vector3d translation);
        Vector3d getRPY();
        Vector4d toAngleAxis();
        void fromAngleAxis(Vector3d axis, double angle);
        Vector4d toQuaternion();
        void fromQuaternion(Vector4d q);
        Matrix3d getRotationX(double gamma);
        Matrix3d getRotationY(double beta);
        Matrix3d getRotationZ(double alpha);
        
        HomTrans();
        ~HomTrans();
       
};

#endif