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

#ifndef GAZEBO_ROS_TACTILE_HH
#define GAZEBO_ROS_TACTILE_HH

#include <string>

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

#include <urdf_tactile/tactile.h>
#include <urdf/sensor.h>

namespace gazebo
{
  /// \brief A Bumper controller
  class GazeboRosTactile : public SensorPlugin
  {
    /// Constructor
    public: GazeboRosTactile();

    /// Destructor
    public: ~GazeboRosTactile();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// Update the controller
    private: void OnContact();

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher contact_pub_; //TODO delete?
    
    //my publisher
    private: ros::Publisher tactile_pub_;

    private: sensors::ContactSensorPtr parentSensor;

    /// \brief set topic name of broadcast
    private: std::string bumper_topic_name_;
    private: std::string tactile_topic_name_;

    private: physics::WorldPtr world_;
    private: physics::LinkPtr local_link_;  //!< link to which the sensor is attached (and in which force and torques are given)
    private: physics::LinkPtr my_link_; //!< user selected frame/link for the result in which all data should be transformed
    private: std::string frame_name_;
    private: std::string local_name_;
    private: std::string collision_name_;  //!< collision name matching the local_link (user selected in the sdf) and might differ from local_name
    private: bool skip_local_transform_;

    /// \brief broadcast some string for now.
    private: gazebo_msgs::ContactsState contact_state_msg_;
    // new Message where to write in
    private: tactile_msgs::TactileState tactile_state_msg_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue contact_queue_;
    private: void ContactQueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;
    
    //for Parsing
    
    private: urdf::SensorMap sensors;
    
    std::vector<std::vector<gazebo::math::Vector3>> taxelNormals;
    std::vector<std::vector<gazebo::math::Vector3>> taxelPositions;
    
    unsigned int numOfSensors;
    std::vector<unsigned int> numOfTaxels;
  };
}

#endif

