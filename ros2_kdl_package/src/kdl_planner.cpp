#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius){
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd){
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}


void KDLPlanner::trapezoidal_vel(double t, double tc, double& s, double& s_dot, double& s_ddot) {

    double tf = trajDuration_;

  
    double ddot_sc = 1.0 / (tc * (tf - tc));

    if (t <= tc) {
        s = 0.5 * ddot_sc * t * t;
        s_dot = ddot_sc * t;
        s_ddot = ddot_sc;
    }

    else if (t <= tf - tc) {
        s = ddot_sc * tc * (t - 0.5 * tc);
        s_dot = ddot_sc * tc;
        s_ddot = 0.0;
    }

    else if (t <= tf) {
        s = 1.0 - 0.5 * ddot_sc * (tf - t) * (tf - t);
        s_dot = ddot_sc * (tf - t);
        s_ddot = -ddot_sc;
    }
}


void KDLPlanner::cubic_polynomial(double t, double& s, double& s_dot, double& s_ddot){
    double tf=trajDuration_;
    // s(0)=0; s_dot(0)=0; s(tf)=1; s_dot(tf)=0
    double a0=0.0;
    double a1=0.0;
    double a2=3.0/(tf*tf);
    double a3=-2.0/(tf*tf*tf);
    
    //postion
    s=a3*(t*t*t)+a2*(t*t)+a1*t+a0;
    //velocity
    s_dot=3.0*a3*(t*t)+2.0*a2*t+a1;
    //acceleration
    s_ddot=6.0*a3*t+2.0*a2;
}

trajectory_point KDLPlanner::compute_trajectory_circle(double t, double tc) {

    trajectory_point traj;
    double s, s_dot, s_ddot;


    if (tc == 0) {
        cubic_polynomial(t, s, s_dot, s_ddot);
    } else {
        trapezoidal_vel(t, tc, s, s_dot, s_ddot);
    }
    
    double angle = 2 * M_PI * s; 
    traj.pos[0] = trajInit_[0]; // x  costant
    traj.pos[1] = trajInit_[1] - trajRadius_ * std::cos(angle); // y
    traj.pos[2] = trajInit_[2] - trajRadius_ * std::sin(angle); // z

    // velocity
    double angle_dot = 2 * M_PI * s_dot; 
    traj.vel[0] = 0.0; // x
    traj.vel[1] = trajRadius_ * angle_dot * std::sin(angle); // y
    traj.vel[2] = -trajRadius_ * angle_dot * std::cos(angle); // z

    // acceleration
    double angle_ddot = 2 * M_PI * s_ddot; 
    traj.acc[0] = 0.0; // x
    traj.acc[1] = trajRadius_ * (angle_ddot * std::sin(angle) + angle_dot * angle_dot * std::cos(angle)); // y
    traj.acc[2] = -trajRadius_ * (angle_ddot * std::cos(angle) - angle_dot * angle_dot * std::sin(angle)); // z

    return traj;
}

trajectory_point KDLPlanner::compute_trajectory_linear(double t, double tc) {

    trajectory_point traj;
    double s, s_dot, s_ddot;

    if (tc == 0) {
        cubic_polynomial(t, s, s_dot, s_ddot);
    } else {
        trapezoidal_vel(t, tc, s, s_dot, s_ddot);
    }

    Eigen::Vector3d diff_EI = trajEnd_ - trajInit_;

    traj.pos = trajInit_ + s * (diff_EI);
    traj.vel = s_dot * (diff_EI);
    traj.acc = s_ddot * (diff_EI);

    return traj;
}
