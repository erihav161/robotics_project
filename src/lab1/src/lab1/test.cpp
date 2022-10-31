#include "lab1/test.hpp"

using namespace std;

int TestHomTrans::testInverse() {
    Eigen::Transform<double, 3, Eigen::Affine> invET;
    this->T1.inverseT();
    invET = this->eT1.inverse();

    if (!this->T1.invT.isApprox(invET.matrix())) {
        cout << "Inverse test failed!" << endl << endl;
        return -1;
    }

    return 0;
}

int TestHomTrans::testPointTransform() {
    this->point2 = this->T1.transformPoint(this->point1);
    this->point3 = this->eT1 * this->point1;
    cout << "p2: " << endl << this->point2 << endl << endl;
    cout << "p3: " << endl << this->point3 << endl << endl;

    if (this->point2 != this->point3) {
        cout << "Point transform test failed!" << endl << endl;
        return -2;
    }
    return 0;
}

int TestHomTrans::testFrameTransform() {
    this->T2 = this->T1.transformFrame(this->T2);
    this->eT2 = this->eT1 * this->eT2;

    if (!this->T2.T.isApprox(this->eT2.matrix())) {
        cout << "Frame transform test failed!" << endl << endl;
        return -3;
    }
    return 0;
}

int TestHomTrans::testAngleAxis() {
    Eigen::Vector4d htAA;
    Eigen::AngleAxisd eAA(this->eT1.linear());
    htAA = this->T1.toAngleAxis();

    if (htAA(0) != eAA.angle() || htAA.block<3,1>(1,0) != eAA.axis()) {
        cout << "Angle axis test failed!" << endl << endl;
        return -4;
    }
    
    return 0;
}

int TestHomTrans::testQuaternion() {
    Eigen::Vector4d htQ;
    htQ = this->T1.toQuaternion();
    Eigen::Quaterniond eQ(this->eT1.linear());

    if (htQ(0) != eQ.w() || htQ.block<3,1>(1,0) != Eigen::Vector3d(eQ.x(), eQ.y(), eQ.z())) {
        cout << "Quaternion test failed" << endl << endl;
        return -5;
    }
    return 0;
}

void TestHomTrans::testAll() {
    cout << endl << "Starting tests!" << endl << endl;

    int tests = 0;
    tests += this->testInverse();
    tests += this->testPointTransform();
    tests += this->testFrameTransform();
    tests += this->testAngleAxis();
    tests += this->testQuaternion();

    cout << "Tests finished!" << endl << endl;
    cout << "Test result (<0 is a fail): " << tests << endl;
}


TestHomTrans::TestHomTrans() {
    this->T1.setIdentityT();
    this->T2.setIdentityT();
    this->eT1.setIdentity();
    this->eT2.setIdentity();
    this->point1 <<
        1,
        1,
        1;
}

TestHomTrans::~TestHomTrans(){}
