#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr2(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    // Matrici di guadagno
    Matrix6d matrix_KP;
    Matrix6d matrix_KD; 
    matrix_KP.setZero();
    matrix_KD.setZero();

    for (int i = 0; i < 3; ++i) {
        matrix_KP(i, i) = _Kpp;  
        matrix_KD(i, i) = _Kdp;  
    }
    for (int i = 3; i < 6; ++i) {
        matrix_KP(i, i) = _Kpo;  
        matrix_KD(i, i) = _Kdo;  
    }

    KDL::Frame cartpos = robot_->getEEFrame();
    KDL::Twist current_velocity = robot_->getEEVelocity();

    Vector6d position_error;
    Vector6d velocity_error;
    
    Eigen::VectorXd desAcc_vec(6);
    desAcc_vec << _desAcc.vel.x(), _desAcc.vel.y(), _desAcc.vel.z(),
                  _desAcc.rot.x(), _desAcc.rot.y(), _desAcc.rot.z();

    computeErrors(_desPos, cartpos, _desVel, current_velocity,position_error, velocity_error);

    Eigen::VectorXd torques;
    Eigen::MatrixXd Jacobian = robot_->getEEJacobian().data;
    Eigen::VectorXd JacDotqDot = robot_->getEEJacDotqDot();

    torques = robot_->getJsim() * pseudoinverse(Jacobian) * (desAcc_vec + matrix_KP * position_error + matrix_KD * velocity_error - JacDotqDot) + robot_->getCoriolis() + robot_->getGravity();    
    return torques;
}