/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_ROS_TACTILE_H
#define GAZEBO_ROS_TACTILE_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sys/time.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/String.h>

#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
#include <tactile_msgs/TactileState.h>

#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/common/Plugin.hh>
#if GAZEBO_MAJOR_VERSION >= 7
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#endif

#include <urdf_tactile/sensor.h>

#define TACT_PLUGIN_DEFAULT_MINFORCE 0.0
#define TACT_PLUGIN_DEFAULT_MAXDISTANCE 0.005
#define TACT_PLUGIN_DEFAULT_UPDATE_RATE 10.0
#define TACT_PLUGIN_DEFAULT_ANGLE_THRESHOLD 0.52  // 30.0

namespace gazebo
{
/// \brief A Bumper controller
class GazeboRosTactile : public SensorPlugin
{
  /// Constructor
public:
  GazeboRosTactile();

  /// Destructor
  ~GazeboRosTactile();

  /// \brief Load the plugin
  /// \param take in SDF root element
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// Update the controller
private:
  void OnContact();

  /// \brief pointer to ros node
  ros::NodeHandle* rosnode_;

#ifdef PUB_DEBUG_CONTACT_STATE
  ros::Publisher contact_pub_;
#endif
  ros::Publisher tactile_pub_;
  sensors::ContactSensorPtr parentSensor;

  /// \brief set topic name of broadcast
  std::string bumper_topic_name_;
  std::string tactile_topic_name_;

  physics::WorldPtr world_;
  physics::LinkPtr local_link_;  //!< link to which the sensor is attached (and in which force and torques are given)
  physics::LinkPtr my_link_;     //!< user selected frame/link for the result in which all data should be transformed
  std::string frame_name_;       //!< name of user selected frame for the result in which all data should be transformed
  std::string frame_id_name_;    //!< same as frame_name_ but resolved for tf_prefix
  std::string local_name_;
  double update_rate_;
  common::Time update_period_;
  common::Time last_update_time_;
  std::string collision_name_;  //!< collision name matching the local_link (user selected in the sdf) and might differ
                                //!< from local_name
  bool skip_local_transform_;

#ifdef PUB_DEBUG_CONTACT_STATE
  /// \brief broadcast some string for now.
  gazebo_msgs::ContactsState contact_state_msg_;
#endif
  // new Message where to write in
  tactile_msgs::TactileState tactile_state_msg_;

  /// \brief for setting ROS name space
  std::string robot_namespace_;
  ros::CallbackQueue contact_queue_;
  void ContactQueueThread();
  boost::thread callback_queue_thread_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;
  bool is_initialized_;
  bool use_gaussianDistribution_;
  void TransformFrameInit();

  // for Parsing
  urdf::tactile::SensorMap sensors;
#if GAZEBO_MAJOR_VERSION >= 7
  std::vector<std::vector<ignition::math::Vector3d>> taxelNormals;
  std::vector<std::vector<ignition::math::Vector3d>> taxelPositions;
#else
  std::vector<std::vector<gazebo::math::Vector3>> taxelNormals;
  std::vector<std::vector<gazebo::math::Vector3>> taxelPositions;
#endif
  unsigned int numOfSensors;
  std::vector<unsigned int> numOfTaxels;
  double forceSensitivity;
  double gaussianDistanceCoefficient_;  // gaussian parameter of the distribution of force to neighbouring cells
  double gaussianAngleCoefficient_;     // gaussian parameter of the distribution of force to neighbouring cells
  double maxDistance_;                  // binary parameter of the distribution of force to neighbouring cells
  double minAngleProjection_;           // binary parameter of the distribution of force to neighbouring cells
  double minForce_;                     // parameter of the filtering;
};
}  // namespace gazebo

#endif  // GAZEBO_ROS_TACTILE_H
