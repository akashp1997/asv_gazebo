#include <algorithm>
#include <assert.h>
#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

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

      this->twist_msg_.linear.x = 0;
      this->twist_msg_.linear.y = 0;
      this->twist_msg_.linear.z = 0;
      this->twist_msg_.angular.x = 0;
      this->twist_msg_.angular.y = 0;
      this->twist_msg_.angular.z = 0;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("accel", "accel plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _model->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("accel", "gazebo_Thuster_Controller plugin error: link named: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("accel", "accel plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("accel", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
    this->topic_name_,1,
    boost::bind(&gazebo::GazeboThrusterController::UpdateObjectAccel,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);

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
public: void UpdateObjectAccel(const geometry_msgs::Twist::ConstPtr& _msg)
{
  this->twist_msg_.linear.x = _msg->linear.x;
  this->twist_msg_.linear.y = _msg->linear.y;
  this->twist_msg_.linear.z = _msg->linear.z;
  this->twist_msg_.angular.x = _msg->angular.x;
  this->twist_msg_.angular.y = _msg->angular.y;
  this->twist_msg_.angular.z = _msg->angular.z;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
public: void UpdateChild()
{
  this->lock_.lock();
  ignition::math::Vector3d linear(this->twist_msg_.linear.x,this->twist_msg_.linear.y,this->twist_msg_.linear.z);
  ignition::math::Vector3d offset(this->twist_msg_.angular.x,this->twist_msg_.angular.y,this->twist_msg_.angular.z);
  this->link_->AddLinkForce(linear);
  this->link_->AddRelativeTorque(offset);
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

  /// \brief A pointer to the Link, where force is applied
  private: physics::LinkPtr link_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;

  /// \brief ROS Wrench topic name inputs
  private: std::string topic_name_;
  /// \brief The Link this plugin is attached to, and will exert forces on.
  private: std::string link_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;
  /// \brief Container for the wrench force that this plugin exerts on the body.
  private: geometry_msgs::Twist twist_msg_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

};
  GZ_REGISTER_MODEL_PLUGIN(GazeboThrusterController)
}