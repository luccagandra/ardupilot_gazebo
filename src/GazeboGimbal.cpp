#include "GazeboGimbal.h"

namespace gazebo {

  enum {
    ROLL,
    PITCH,
    YAW,
};

  GazeboGimbal::GazeboGimbal() {}

  GazeboGimbal::~GazeboGimbal() {}

  void GazeboGimbal::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("Cannot load Gimbal plugin, ROS node for Gazebo not initialized");
      return;
    }

    joints_.resize(3);
    joints_[ROLL] = _model->GetJoint("gimbal_roll_joint");
    joints_[PITCH] = _model->GetJoint("gimbal_pitch_joint");
    joints_[YAW] = _model->GetJoint("gimbal_yaw_joint");
    count = 0;

    link = _model->GetLink("gimbal_link_1");
    world = _model;
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboGimbal::Update, this));
  }

  void GazeboGimbal::Update() {
    ignition::math::Pose3d pose = link->WorldPose();

    double x, y, z, w;
    x = pose.Rot().X();
    y = pose.Rot().Y();
    z = pose.Rot().Z();
    w = pose.Rot().W();

    double pitch_c = M_PI/2 - std::acos(2*x*z - 2*y*w);
    double roll_c = std::acos(2*y*z + 2*x*w) - M_PI/2;

    joints_[YAW]->SetPosition(0, 0);
    joints_[PITCH]->SetPosition(0, pitch_c);
    joints_[ROLL]->SetPosition(0, roll_c);

  }

GZ_REGISTER_MODEL_PLUGIN(GazeboGimbal);
}