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

Eigen::VectorXd KDLController::velocity_control_null(KDL::Frame &_desPos,
                                                    // KDL::Twist &_desVel,
                                                    // KDL::Twist &_desAcc,
                                                    double Kpp)
{
    // Stato
    Eigen::VectorXd q = robot_->getJntValues();      // n×1
    const int n = q.size();

    // Errore di posizione (solo traslazione)
    KDL::Frame cartpos = robot_->getEEFrame();
    KDL::Vector d = _desPos.p - cartpos.p;
    Eigen::Vector3d e_p(d.x(), d.y(), d.z());        // 3×1


    // Jacobiano (6×n) e parte lineare (3×n)
    KDL::Jacobian J_kdl = robot_->getEEJacobian();
    Eigen::MatrixXd J6 = J_kdl.data;    //Eigen::MatrixXd J_dag = J.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd J = J6.block(0, 0, 3, n);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd J_dag = J.transpose()*(J*J.transpose()).inverse();


    // Limiti
    Eigen::MatrixXd limits = robot_->getJntLimits();
    Eigen::VectorXd qmin = limits.col(0);
    Eigen::VectorXd qmax = limits.col(1);


    const double lambda = 100;

    Eigen::VectorXd gradient;
    gradient.resize(n);

    std::cout << "J_dag: " << J_dag.rows() << "x" << J_dag.cols() << std::endl;
    std::cout << "J: " << J.rows() << "x" << J.cols() << std::endl;
    std::cout << "e_p: " << e_p.size() << std::endl;
   

    
    for(unsigned int i = 0; i < n; i++) {
        if (q(i,0) < qmin(i) || q(i,0) > qmax(i)) {
            std::cout << "joint " << i << " limits violated. Value = "<< q(i,0)*180.0/M_PI << "\n"; 
        }
            gradient(i,0) = 1.0/lambda * std::pow((qmax(i) - qmin(i)),2)* (2*q(i,0) - qmax(i) - qmin(i))/(std::pow((qmax(i)-q(i,0)),2)*std::pow((q(i,0)-qmin(i)),2));
    } 
    Eigen::VectorXd q0_dot; 
    q0_dot = gradient; 
    Eigen::VectorXd q_dot = J_dag*Kpp*e_p +(I-J_dag*J)*q0_dot;

    return q_dot;
}



    



