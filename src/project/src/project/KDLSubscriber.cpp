#include <project/KDLSubscriber.hpp>
#define FLOAT_MIN -1.7
#define FLOAT_MAX 1.7
#include <random>
#include <iomanip>
#define LAMBDA pow(10, -5)

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
        if (i == 3) {
            this->qc(i) = -1;
            continue;
        }
        else if (i == 5) {
            this->qc(i) = 1;
            continue;
        }
        this->qc(i) = this->randomGuess();
        cout << " " << this->qc(i);
    }
    cout << endl;
}

VectorXd KDLSubscriber::getVelocity(KDL::JntArray qc, KDL::JntArray qg) {
    VectorXd v(6,1);
    Transform<double, 3, Affine> Tc, Tg;
    Tc = this->calculateEndFrame(qc);
    Tg = this->calculateEndFrame(qg);

    Tc = Tc.inverse();
    Tc = Tc * Tg;

    v.block<3,1>(0,0) = Tc.translation();
    v.block<3,1>(3,0) = Tc.linear().eulerAngles(0, 1, 2);
    
    return v;
}

void KDLSubscriber::callBack(const sensor_msgs::JointState &jointState) {
    this->jointStateMessage = jointState;
    // this->calculateEndFrame(jointState);
    this->solveIK();
}

void KDLSubscriber::solveIK() {
    int nrJoints = this->my_tree.getNrOfJoints();
    KDL::JntArray qGoal(nrJoints);
    VectorXd qCurrent(nrJoints,1), deltaVelocity(6,1), u0(nrJoints, 1);
    MatrixXd pseudoInverseJ(6,nrJoints), JJT(6,6), I(nrJoints, nrJoints), J(6,nrJoints);

    I.setIdentity();


    
    for (int i = 0; i < nrJoints; i++) {
        if (nrJoints > 3) {
            qGoal(i) = this->jntsPanda.at(i);
            u0(i) = qGoal(i);
        }
        else {
            qGoal(i) = this->jnts3dof.at(i);
        }
    }
   
    
    deltaVelocity = this->getVelocity(this->qc, qGoal);
    
    while (deltaVelocity.norm() > 0.001) {

        // cout << deltaVelocity.block<3,1>(0,0).norm() << endl;
        cout << deltaVelocity << endl << endl;

        J = this->calcualteJacobian(this->qc);

        JJT = J * J.transpose();
        JJT.diagonal().array() += 0.01;
        pseudoInverseJ = J.transpose()*(JJT.inverse());

        deltaVelocity *= pow(10, 1);
        for (int i = 0; i < nrJoints; i++) {
            qCurrent(i) = this->qc(i);
        }

        
        qCurrent = qCurrent + LAMBDA * pseudoInverseJ * deltaVelocity + (I - pseudoInverseJ*J)*u0;

        for (int i = 0; i < nrJoints; i++) {
            this->qc(i) = qCurrent(i);
            this->jointStateMessage.position[i] = this->qc(i);
        }

        this->jointStateMessage.header.stamp = ros::Time::now();
        this->publisher.publish(jointStateMessage);
        deltaVelocity = this->getVelocity(this->qc, qGoal);
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
    if (this->fkSolver->JntToCart(array, endEffectorFrame, ROBOT_DESC) != 0) {
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

Transform<double, 3, Affine> KDLSubscriber::calculateEndFrame(KDL::JntArray q) {
    Transform<double, 3, Affine> T;
    T.setIdentity();
    Matrix3d R;
    Vector3d rpy;
    VectorXd endFrameVector(6,1);
    KDL::Frame frame;
    if (this->fkSolver->JntToCart(q, frame, ROBOT_DESC) != 0) {
        ROS_ERROR("fkSolverPos failed");
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R(i,j) = frame.M(i,j);
        }
    }

    T.linear() = R;
    T.translation() = Vector3d(frame.p(0), frame.p(1), frame.p(2));

    return T;
}



MatrixXd KDLSubscriber::calcualteJacobian(const sensor_msgs::JointState &jointState) {
    int n = this->my_tree.getNrOfJoints();
    MatrixXd jacobi(6,n);
    KDL::JntArray array(this->my_tree.getNrOfJoints());
    KDL::Jacobian J(this->my_tree.getNrOfJoints());

    for (int i = 0; i < this->my_tree.getNrOfJoints(); i++) {
        array(i) = jointState.position[i];
    }
    if (this->jacobiSolver->JntToJac(array, J, ROBOT_DESC) != 0) {
        ROS_ERROR("jacobiSolver failed");
    }

    return jacobi;
}

MatrixXd KDLSubscriber::calcualteJacobian(KDL::JntArray q) {
    MatrixXd jacobi(6,this->my_tree.getNrOfJoints());
    KDL::Jacobian J(this->my_tree.getNrOfJoints());

    if (this->jacobiSolver->JntToJac(q, J, ROBOT_DESC) != 0) {
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
    this->jnts3dof = {0, 0, 0};
    this->jntsPanda = {0, 0, 0, -1, 0, 0, 0};
}

KDLSubscriber::~KDLSubscriber() {

}
