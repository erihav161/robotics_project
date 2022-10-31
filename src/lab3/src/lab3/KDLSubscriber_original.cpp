#include <lab3/KDLSubscriber.hpp>
#define FLOAT_MIN -1.9
#define FLOAT_MAX 1.9
#include <random>
#include <iomanip>

using namespace std;
using namespace Eigen;

float KDLSubscriber::randomGuess(){
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<float> distr(FLOAT_MIN, FLOAT_MAX);
  return distr(eng);
}

void KDLSubscriber::randomPose() {
    cout << "initial qc:";
    for (int i = 0; i < this->my_tree.getNrOfJoints(); i++) {
        this->qc(i) = this->randomGuess();
        cout << " " << this->qc(i);
    }
    cout << endl;
}

void KDLSubscriber::callBack(const sensor_msgs::JointState &jointState) {
    this->jointStateMessage = jointState;
    // this->calculateEndFrame(jointState);
    this->solveIK();
}

void KDLSubscriber::solveIK() {
    int nrJoints = this->my_tree.getNrOfJoints();
    KDL::JntArray qGoal(nrJoints);
    KDL::Frame goalFrame;
    VectorXd goalVector(6,1), currentVector(6,1), qCurrent(nrJoints,1), deltaVel(6,1);
    MatrixXd pseudoInverseJ(6,nrJoints), JJT(6,6);
    
    for (int i = 0; i < nrJoints; i++) {
        qGoal(i) = 0;
    }
    goalVector = this->calculateEndFrame(qGoal);
    currentVector = this->calculateEndFrame(this->qc);
    deltaVel = goalVector - currentVector;


    while ((deltaVel.block<3,1>(0,0)).norm() > 0.1) {
        // cout << "Dleta velocity:" << endl << deltaVel << endl;
        // cout << "qc: " << this->qc(0) << " " << this->qc(1) << " " << this->qc(2) << endl;
        pseudoInverseJ = this->calcualteJacobian(this->qc);
        // cout << "Jacobian: " << endl << pseudoInverseJ << endl;
        JJT = pseudoInverseJ * pseudoInverseJ.transpose();
        JJT.diagonal().array() = 0.0001;
        pseudoInverseJ = pseudoInverseJ.transpose()*(JJT.inverse());
        // cout << "Pseudo inverse: " << endl << pseudoInverseJ << endl;

        deltaVel *= pow(10, -3);
        for (int i = 0; i < nrJoints; i++) {
            qCurrent(i) = qc(i);
        }
        qCurrent = qCurrent + pseudoInverseJ * deltaVel;
        // cout << "qc: " << endl << qCurrent << endl;
        for (int i = 0; i < nrJoints; i++) {
            qc(i) = qCurrent(i);
            this->jointStateMessage.position[i] = qc(i);
        }

        this->jointStateMessage.header.stamp = ros::Time::now();
        this->publisher.publish(jointStateMessage);
        // cout << "Current vector: " << endl << this->calculateEndFrame(this->qc) << endl;
        deltaVel = goalVector - (this->calculateEndFrame(this->qc));
    }
    cout << "Finished!" << endl;
    this->randomPose();
    sleep(2);
}

void KDLSubscriber::calculateEndFrame(const sensor_msgs::JointState &jointState) {
    KDL::JntArray array(this->my_tree.getNrOfJoints());
    KDL::Frame endEffectorFrame;

    for (int i = 0; i < this->my_tree.getNrOfJoints(); i++) {
        array(i) = jointState.position[i];
    }
    if (this->fkSolver->JntToCart(array, endEffectorFrame, "three_dof_planar_eef") != 0) {
        ROS_ERROR("fkSolverPos failed");
    }
    cout << " x: " << endEffectorFrame.p(0) << " y: " << endEffectorFrame.p(1) << " z: " << endEffectorFrame.p(2) << endl;
    cout << "M: " << endl;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cout << endEffectorFrame.M(i,j) << " ";
        }
        cout << endl;
    } 

}

VectorXd KDLSubscriber::calculateEndFrame(KDL::JntArray q) {
    Transform<double, 3, Affine> T;
    T.setIdentity();
    Matrix3d R;
    Vector3d rpy;
    VectorXd endFrameVector(6,1);
    KDL::Frame frame;
    if (this->fkSolver->JntToCart(q, frame, "three_dof_planar_eef") != 0) {
        ROS_ERROR("fkSolverPos failed");
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R(i,j) = frame.M(i,j);
        }
    }

    T.linear() = R;
    rpy = T.linear().eulerAngles(0, 1, 2);
    endFrameVector <<
        frame.p(0),
        frame.p(1),
        frame.p(2),
        rpy(0),
        rpy(1),
        rpy(2);
    return endFrameVector;
}



MatrixXd KDLSubscriber::calcualteJacobian(const sensor_msgs::JointState &jointState) {
    int n = this->my_tree.getNrOfJoints();
    MatrixXd jacobi(6,n);
    KDL::JntArray array(this->my_tree.getNrOfJoints());
    KDL::Jacobian J(this->my_tree.getNrOfJoints());

    for (int i = 0; i < this->my_tree.getNrOfJoints(); i++) {
        array(i) = jointState.position[i];
    }
    if (this->jacobiSolver->JntToJac(array, J, "three_dof_planar_eef") != 0) {
        ROS_ERROR("jacobiSolver failed");
    }
    // cout << "Jacobi matrix: " << endl <<  J.data << endl;

    return jacobi;
}

MatrixXd KDLSubscriber::calcualteJacobian(KDL::JntArray q) {
    MatrixXd jacobi(6,this->my_tree.getNrOfJoints());
    KDL::Jacobian J(this->my_tree.getNrOfJoints());

    if (this->jacobiSolver->JntToJac(q, J, "three_dof_planar_eef") != 0) {
        ROS_ERROR("jacobiSolver failed");
    }
    jacobi = J.data;
    return jacobi;
}


KDLSubscriber::KDLSubscriber() {
    std::string robot_desc_string;
    this->node.param("robot_description", robot_desc_string, std::string());
    if(!kdl_parser::treeFromString(robot_desc_string, this->my_tree)) {
        ROS_ERROR("Failed to construct kdl_tree");
    }
    this->fkSolver = new KDL::TreeFkSolverPos_recursive(this->my_tree);
    this->jacobiSolver = new KDL::TreeJntToJacSolver(this->my_tree);
    this->subscriber = node.subscribe("/joint_states", 1000, &KDLSubscriber::callBack, this);
    this->publisher = node.advertise<sensor_msgs::JointState>("my_joint_states", 1000);
    this->qc = KDL::JntArray(this->my_tree.getNrOfJoints());
    this->randomPose();
}

KDLSubscriber::~KDLSubscriber() {

}