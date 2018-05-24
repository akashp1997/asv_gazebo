#include <algorithm>
#include <assert.h>
#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/math/gzmath.hh>

namespace gazebo 
{
  class GazeboThrusterController : public ModelPlugin 
  {
  /*  ////////////////////////////////////////////////////////////////////////////////
    // Constructor
  public: void GazeboThrusterConroller()
    {
    }

////////////////////////////////////////////////////////////////////////////////
// Destructor
public: void ~GazeboThrusterConroller()
{
  this->update_connection_.reset();

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}*/

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();

  this->model_ = _model;

  this->left_msg_ = 1500;
  this->right_msg_ = 1500;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _model->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n",this->link_name_.c_str());
    return;
}

  if (!_sdf->HasElement("LeftTopicName")||!_sdf->HasElement("RightTopicName"))
  {
    ROS_FATAL_NAMED("accel", "accel plugin missing <LeftTopicName> and/or <RightTopicName>, cannot proceed");
    return;
  }
  else
    this->left_topic_name_ = _sdf->GetElement("LeftTopicName")->Get<std::string>();
    this->right_topic_name_ = _sdf->GetElement("RightTopicName")->Get<std::string>();


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("accel", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  ros::SubscribeOptions so_left = ros::SubscribeOptions::create<std_msgs::Float64>(
    this->left_topic_name_,1,
    boost::bind(&gazebo::GazeboThrusterController::UpdateObjectLeftAccel,this,_1),
    ros::VoidPtr(), &this->queue_);
  ros::SubscribeOptions so_right = ros::SubscribeOptions::create<std_msgs::Float64>(
    this->right_topic_name_,1,
    boost::bind(&gazebo::GazeboThrusterController::UpdateObjectRightAccel,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_left_ = this->rosnode_->subscribe(so_left);
  this->sub_right_ = this->rosnode_->subscribe(so_right);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &gazebo::GazeboThrusterController::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&gazebo::GazeboThrusterController::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
public: void UpdateObjectLeftAccel(const std_msgs::Float64::ConstPtr& _msg)
{
  //ROS_INFO("Left");
  this->left_msg_ = _msg->data;
}
public: void UpdateObjectRightAccel(const std_msgs::Float64::ConstPtr& _msg)
{
  //ROS_INFO("Right");
  this->right_msg_ = _msg->data;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
public: void UpdateChild()
{
  this->lock_.lock();
  this->link_->ResetPhysicsStates();
  double force_left, force_right;
  //The Idea is to calculate Resultant Forces in global frame by calculating and generating transforms.
  //Then convert these global frame forces to the frame of the world by calculating their components in respective axis.
  //The idea for torque is already there, as torque is global in nature.
  double max_forward = 400;
  double max_backward = 320;
  if(this->left_msg_<1500) {
    force_left = max_backward*(this->left_msg_-1500)/400;
  }
  else {
    force_left = max_forward*(this->left_msg_-1500)/400;
  }
  force_left = force_left>max_forward?max_forward:force_left;
  force_left = force_left<-max_backward?-max_backward:force_left;

  if(this->right_msg_<1500) {
    force_right = max_backward*(this->right_msg_-1500)/400;
  }
  else {
    force_right = max_forward*(this->right_msg_-1500)/400;    
  }
  force_right = force_right>max_forward?max_forward:force_right;
  force_right = force_right<-max_backward?-max_backward:force_right;
  //ROS_INFO("%f %f", force_left, force_right);
  ignition::math::Vector3d linear1(force_left,0,0);
  ignition::math::Vector3d linear2(force_right,0,0);
  //Correct for the 3DOF which are unrequired
  //ignition::math::Vector3d ang(0,0,this->link_->GetWorldAngularVel()[2]);
  ignition::math::Vector3d lin1(-0.1,-0.3,-0.1);
  ignition::math::Vector3d lin2(-0.1,0.3,-0.1);

  ignition::math::Vector3d force = linear1+linear2;
  ignition::math::Vector3d torque = linear1.Cross(lin1)+linear2.Cross(lin2);
  //ROS_INFO("%f", torque[2]);
  //ignition::math::Vector3d newtorque(torque[2], torque[1], torque[0]);
  //ROS_INFO("%.2f %.2f %.2f | %.2f %.2f %.2f", force[0], force[1], force[2], torque[0], torque[1], torque[2]);
  //ROS_INFO("%f", ang[0]);
  //this->link_->SetWorldTwist(lin, ang);
  //Works on Set and Add functions

  //These functions work good. However, the force here is relative to world frame and needs to be converted to local boat frame.
  this->link_->AddRelativeForce(force);
  this->link_->AddTorque(torque);  
  //this->link_->AddTorqueAtRelativePosition(angular, lin);
  this->lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
private: void QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}


  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  private: physics::ModelPtr model_;

  /// \brief A pointer to the Link, where force is applied
  private: physics::LinkPtr link_;
  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_left_;
  private: ros::Subscriber sub_right_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;

  /// \brief ROS Wrench topic name inputs
  private: std::string left_topic_name_;
  private: std::string right_topic_name_;
  /// \brief The Link this plugin is attached to, and will exert forces on.
  private: std::string link_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;
  /// \brief Container for the wrench force that this plugin exerts on the body.
  private: int left_msg_;
  private: int right_msg_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

};
  GZ_REGISTER_MODEL_PLUGIN(GazeboThrusterController)
}