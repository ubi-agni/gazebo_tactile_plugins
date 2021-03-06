/*
 * Copyright 2013 Open Source Robotics Foundation
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

/*
 * Desc: Contact controller
 * Author: Lars Oetermann
 * original of bumper controller Author: Nate Koenig
 * Date: 09 Sept. 2008
 */

/*
 * FRAME info
 * Gazebo provides positions and normals of contact in world frame
 * BUT Gazebo provides force and torques in link frame
 */

#include <map>
#include <string>

#include <gazebo/common/Exception.hh>
#include <gazebo/gazebo_config.h>
#if GAZEBO_MAJOR_VERSION >= 7
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#else
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>
#endif
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <sdf/Param.hh>
#include <sdf/sdf.hh>

#include <tf/tf.h>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_ros_contact/gazebo_ros_contact.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosContact)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosContact::GazeboRosContact() : SensorPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosContact::~GazeboRosContact()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosContact::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parentSensor = dynamic_pointer_cast<sensors::ContactSensor>(_parent);
  if (!this->parentSensor)
  {
    ROS_ERROR("Contact sensor parent is not of type ContactSensor");
    return;
  }

#if GAZEBO_MAJOR_VERSION >= 7
  std::string worldName = _parent->WorldName();
#else
  std::string worldName = _parent->GetWorldName();
#endif
  this->world_ = physics::get_world(worldName);

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  // "publishing contact/collisions to this topic name: "
  //   << this->bumper_topic_name_ << std::endl;
  this->bumper_topic_name_ = "bumper_states";
  if (_sdf->HasElement("bumperTopicName"))
    this->bumper_topic_name_ = _sdf->GetElement("bumperTopicName")->Get<std::string>();

  // "publishing tactile contact to this topic name: "
  //   << this->tactile_topic_name_ << std::endl;
  this->tactile_topic_name_ = "tactile_states";
  if (_sdf->GetElement("tactileTopicName"))
    this->tactile_topic_name_ = _sdf->GetElement("tactileTopicName")->Get<std::string>();

// "get the body (link) name to which the sensor is attached"
#if GAZEBO_MAJOR_VERSION >= 7
  std::string parentName = _parent->ParentName();
#else
  std::string parentName = _parent->GetParentName();
#endif
  local_name_ = parentName.substr(parentName.find_last_of(':') + 1);
  ROS_DEBUG_STREAM("contact plugin belongs to link named: " << local_name_);

  // by default use collision_name as parentName:local_name_ _collision
  collision_name_ = parentName + "::" + local_name_ + "_collision";

  // try access the real collision name used for the contact sensor (one level up from the plugin sdf)
  if (_sdf->GetParent()->HasElement("contact"))
  {
     if (_sdf->GetParent()->GetElement("contact")->HasElement("collision"))
     {
       collision_name_ = _sdf->GetParent()->GetElement("contact")->GetElement("collision")->Get<std::string>();
       collision_name_ = parentName + "::" + collision_name_;
     }
  }

  // "transform contact/collisions pose, forces to this body (link) name: "
  //   << this->frame_name_ << std::endl;
  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("bumper plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  // Because forces and torques are in local frame, a transformation is needed
  // Because position and normals are given in world another transformation is needed

  // Two frames are needed. The local frame, of the current link
  // and the frame of the user selected frame

  // lock in case a model is being spawned
  // boost::recursive_mutex::scoped_lock lock(*gazebo::Simulator::Instance()->GetMRMutex());
#if GAZEBO_MAJOR_VERSION >= 8
  physics::Model_V all_models = world_->Models();
#else
  physics::Model_V all_models = world_->GetModels();
#endif

  // if frameName specified is "world", "/map" or "map" report back
  // inertial values in the gazebo world.
  if (this->my_link_ == NULL && this->frame_name_ != "world" && this->frame_name_ != "/map" &&
      this->frame_name_ != "map")
  {
    // look through all models in the world, search for body
    // name that matches frameName
    for (physics::Model_V::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
    {
      if (*iter)
        this->my_link_ = boost::dynamic_pointer_cast<physics::Link>((*iter)->GetLink(this->frame_name_));
      if (this->my_link_)
        break;
    }

    // not found
    if (!this->my_link_)
    {
      ROS_INFO("gazebo_ros_bumper plugin: frameName: %s does not exist"
               " using world",
               this->frame_name_.c_str());
    }
  }

  if (this->local_link_ == NULL)
  {
    // look through all models in the world, search for body
    // name that matches local link name
    for (physics::Model_V::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
    {
      if (*iter)
        this->local_link_ = boost::dynamic_pointer_cast<physics::Link>((*iter)->GetLink(this->local_name_));
      if (this->local_link_)
        break;
    }

    // not found
    if (!this->local_link_)
    {
      ROS_FATAL("gazebo_ros_bumper plugin: local link: %s does not exist", this->local_name_.c_str());
      return;
    }
  }

  // if local name is the same as the desired frame name, skip force/torque transform
  skip_local_transform_ = false;
  if (this->frame_name_ == this->local_name_)
    skip_local_transform_ = true;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_id_name_ = tf::resolve(prefix, this->frame_name_);

  this->tactile_pub_ =
    this->rosnode_->advertise<tactile_msgs::TactileContact>(std::string(this->tactile_topic_name_), 1);

  this->contact_pub_ = this->rosnode_->advertise<gazebo_msgs::ContactsState>(std::string(this->bumper_topic_name_), 1);

  // Initialize
  // start custom queue for contact bumper
  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosContact::ContactQueueThread, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = this->parentSensor->ConnectUpdated(boost::bind(&GazeboRosContact::OnContact, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosContact::OnContact()
{
  if (this->contact_pub_.getNumSubscribers() <= 0 && this->tactile_pub_.getNumSubscribers() <= 0)
    return;

  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();

  // define frame_name and stamp
  this->tactile_contact_msg_.header.frame_id = this->frame_id_name_;
  this->contact_state_msg_.header.frame_id = this->frame_id_name_;

  ros::Time contact_time;
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time gazebotime = this->world_->SimTime();
#else
  common::Time gazebotime = this->world_->GetSimTime();
#endif
  common::Time meastime = this->parentSensor->LastMeasurementTime();
  ROS_DEBUG_STREAM("sim time " << gazebotime.sec << "," << gazebotime.nsec);
  ROS_DEBUG_STREAM("measurement time " << meastime.sec << "," << meastime.nsec);
  msgs::Time contacts_time = contacts.time();
  ROS_DEBUG_STREAM("contacts time via msgs " << contacts_time.sec() << "," << contacts_time.nsec());
  ROS_DEBUG_STREAM("global contact time " << contacts.time().sec() << "," << contacts.time().nsec());
  contact_time = ros::Time(meastime.sec, meastime.nsec);

  // get reference frame (body(link)) pose and subtract from it to get
  // relative force, torque, position and normal vectors
#if GAZEBO_MAJOR_VERSION >= 7
  ignition::math::Pose3d pose, frame_pose, local_pose;
  ignition::math::Quaterniond rot, frame_rot, local_rot;
  ignition::math::Vector3d pos, frame_pos, local_pos;
#else
  math::Pose pose, frame_pose, local_pose;
  math::Quaternion rot, frame_rot, local_rot;
  math::Vector3 pos, frame_pos, local_pos;
#endif
  

  // Get local link orientation
  if (local_link_)
  {
#if GAZEBO_MAJOR_VERSION >= 7
 #if GAZEBO_MAJOR_VERSION >= 8
    local_pose = local_link_->WorldPose();
 #else
    local_pose = local_link_->GetWorldPose().Ign();
 #endif
    local_pos = local_pose.Pos();
    local_rot = local_pose.Rot();
#else
    local_pose = local_link_->GetWorldPose();
    local_pos = local_pose.pos;
    local_rot = local_pose.rot;
#endif
  }
  // Get frame orientation if frame_id is given
  if (my_link_)
  {
#if GAZEBO_MAJOR_VERSION >= 7
 #if GAZEBO_MAJOR_VERSION >= 8
    frame_pose = my_link_->WorldPose();
 #else
    frame_pose = my_link_->GetWorldPose().Ign(); // -this->myBody->GetCoMPose();->GetDirtyPose();
 #endif
    frame_pos = frame_pose.Pos();
    frame_rot = frame_pose.Rot();
#else
    frame_pose = my_link_->GetWorldPose();
    frame_pos = frame_pose.pos;
    frame_rot = frame_pose.rot;
#endif
  }
  else
  {
    // no specific frames specified, use identity pose, keeping
    // relative frame at inertial origin
#if GAZEBO_MAJOR_VERSION >= 7
    frame_pos = ignition::math::Vector3d(0, 0, 0);
    frame_rot = ignition::math::Quaterniond(1, 0, 0, 0);  // gazebo u,x,y,z == identity
    frame_pose = ignition::math::Pose3d(frame_pos, frame_rot);
#else
    frame_pos = math::Vector3(0, 0, 0);
    frame_rot = math::Quaternion(1, 0, 0, 0);  // gazebo u,x,y,z == identity
    frame_pose = math::Pose(frame_pos, frame_rot);
#endif
  }

  geometry_msgs::Wrench total_wrench;
  total_wrench.force.x = 0;
  total_wrench.force.y = 0;
  total_wrench.force.z = 0;
  total_wrench.torque.x = 0;
  total_wrench.torque.y = 0;
  total_wrench.torque.z = 0;

  geometry_msgs::Point total_position;
  total_position.x = 0;
  total_position.y = 0;
  total_position.z = 0;
  double total_force_lengths = 0;
  double total_normal_lengths = 0;

  geometry_msgs::Point average_position;
  average_position.x = 0;
  average_position.y = 0;
  average_position.z = 0;
  geometry_msgs::Vector3 total_normal;
  total_normal.x = 0;
  total_normal.y = 0;
  total_normal.z = 0;

  this->contact_state_msg_.states.clear();

  // Loop over Collisions
  // GetContacts returns all contacts on the collision body
  unsigned int contactsPacketSize = contacts.contact_size();
  for (unsigned int i = 0; i < contactsPacketSize; ++i)
  {
    // Create a ContactState
    gazebo_msgs::ContactState state;
    gazebo::msgs::Contact contact = contacts.contact(i);

    state.collision1_name = contact.collision1();
    state.collision2_name = contact.collision2();
    std::ostringstream stream;
    stream << "Debug:  i:(" << i << "/" << contactsPacketSize << ")     my geom:" << state.collision1_name
           << "   other geom:" << state.collision2_name
           << "         time:" << ros::Time(contact.time().sec(), contact.time().nsec()) << std::endl;
    state.info = stream.str();

    // is body 1 the collision body attached to the sensor plugin ?
    bool switch_body = false;
    if(state.collision2_name == collision_name_)
      switch_body = true;


    state.wrenches.clear();
    state.contact_positions.clear();
    state.contact_normals.clear();
    state.depths.clear();

    // contact_time = ros::Time(contact.time().sec(), contact.time().nsec());
    ROS_DEBUG_STREAM("local contact time " << contact.time().sec() << "," << contact.time().nsec());
    // Loop over Contacts
    unsigned int contactGroupSize = contact.position_size();
    for (unsigned int j = 0; j < contactGroupSize; ++j)
    {
      // Get force, torque. They are in local frame already.
      // forward transform them to world and then
      // and rotate into user specified frame.
      // frame_rot is identity if world is used (default for now)

      ROS_DEBUG_STREAM("body1 name: " << contact.wrench(j).body_1_name() << ", body2 name: " << contact.wrench(j).body_2_name());

      // select the correct body
      gazebo::msgs::Wrench source_wrench;
      if (switch_body)
      {
        ROS_DEBUG("using body2");
        source_wrench = contact.wrench(j).body_2_wrench();
      }
      else
        source_wrench = contact.wrench(j).body_1_wrench();

#if GAZEBO_MAJOR_VERSION >= 7
      ignition::math::Vector3d force;
      ignition::math::Vector3d torque;
      // apply transform to force/torque only if needed
      if (skip_local_transform_)
      {
        force = ignition::math::Vector3d(source_wrench.force().x(), source_wrench.force().y(), source_wrench.force().z());
        torque = ignition::math::Vector3d(source_wrench.torque().x(), source_wrench.torque().y(), source_wrench.torque().z());
      }
      else
      {
        force = frame_rot.RotateVectorReverse(local_rot.RotateVector(
          ignition::math::Vector3d(source_wrench.force().x(), source_wrench.force().y(), source_wrench.force().z())));
        torque = frame_rot.RotateVectorReverse(local_rot.RotateVector(
          ignition::math::Vector3d(source_wrench.torque().x(), source_wrench.torque().y(), source_wrench.torque().z())));
      }
#else
      math::Vector3 force;
      math::Vector3 torque;
      // apply transform to force/torque only if needed
      if (skip_local_transform_)
      {
        force = math::Vector3(source_wrench.force().x(), source_wrench.force().y(), source_wrench.force().z());
        torque = math::Vector3(source_wrench.torque().x(), source_wrench.torque().y(), source_wrench.torque().z());
      }
      else
      {
        force = frame_rot.RotateVectorReverse(local_rot.RotateVector(
          math::Vector3(source_wrench.force().x(), source_wrench.force().y(), source_wrench.force().z())));
        torque = frame_rot.RotateVectorReverse(local_rot.RotateVector(
          math::Vector3(source_wrench.torque().x(), source_wrench.torque().y(), source_wrench.torque().z())));
      }
#endif

      // set contact wrenches
      geometry_msgs::Wrench wrench;
#if GAZEBO_MAJOR_VERSION >= 7
      wrench.force.x = force.X();
      wrench.force.y = force.Y();
      wrench.force.z = force.Z();
      wrench.torque.x = torque.X();
      wrench.torque.y = torque.Y();
      wrench.torque.z = torque.Z();
#else
      wrench.force.x = force.x;
      wrench.force.y = force.y;
      wrench.force.z = force.z;
      wrench.torque.x = torque.x;
      wrench.torque.y = torque.y;
      wrench.torque.z = torque.z;
#endif
      state.wrenches.push_back(wrench);

      // vector sum of forces and torques
      total_wrench.force.x += wrench.force.x;
      total_wrench.force.y += wrench.force.y;
      total_wrench.force.z += wrench.force.z;
      total_wrench.torque.x += wrench.torque.x;
      total_wrench.torque.y += wrench.torque.y;
      total_wrench.torque.z += wrench.torque.z;

      // rotate normal from world into user specified frame.
      // frame_rot is identity if world is used.
#if GAZEBO_MAJOR_VERSION >= 7
      ignition::math::Vector3d normal = frame_rot.RotateVectorReverse(
        ignition::math::Vector3d(contact.normal(j).x(), contact.normal(j).y(), contact.normal(j).z()));
#else
      math::Vector3 normal = frame_rot.RotateVectorReverse(
        math::Vector3(contact.normal(j).x(), contact.normal(j).y(), contact.normal(j).z()));
#endif

      // set contact normals pointing outwards of the collision object
      geometry_msgs::Vector3 contact_normal;
#if GAZEBO_MAJOR_VERSION >= 7
      contact_normal.x = (switch_body ? 1.0:-1.0) * normal.X();
      contact_normal.y = (switch_body ? 1.0:-1.0) * normal.Y();
      contact_normal.z = (switch_body ? 1.0:-1.0) * normal.Z();
      // vector sum of normals
      total_normal.x += normal.X();
      total_normal.y += normal.Y();
      total_normal.z += normal.Z();
#else
      contact_normal.x = (switch_body ? 1.0:-1.0) * normal.x;
      contact_normal.y = (switch_body ? 1.0:-1.0) * normal.y;
      contact_normal.z = (switch_body ? 1.0:-1.0) * normal.z;
      // vector sum of normals
      total_normal.x += normal.x;
      total_normal.y += normal.y;
      total_normal.z += normal.z;
#endif
      state.contact_normals.push_back(contact_normal);

      // force amplitude
#if GAZEBO_MAJOR_VERSION >= 7
      double force_length = force.Length();
#else
      double force_length = force.GetLength();
#endif

      // transform contact positions from world frame into user frame
      // set contact position
      geometry_msgs::Vector3 contact_position;
#if GAZEBO_MAJOR_VERSION >= 7
      ignition::math::Vector3d position = frame_rot.RotateVectorReverse(
        ignition::math::Vector3d(contact.position(j).x(), contact.position(j).y(), contact.position(j).z()) - frame_pos);
      contact_position.x = position.X();
      contact_position.y = position.Y();
      contact_position.z = position.Z();
      // average position weighted on force amplitude
      total_position.x += position.X() * force_length;
      total_position.y += position.Y() * force_length;
      total_position.z += position.Z() * force_length;
#else
      gazebo::math::Vector3 position = frame_rot.RotateVectorReverse(
        math::Vector3(contact.position(j).x(), contact.position(j).y(), contact.position(j).z()) - frame_pos);
      contact_position.x = position.x;
      contact_position.y = position.y;
      contact_position.z = position.z;
      // average position weighted on force amplitude
      total_position.x += position.x * force_length;
      total_position.y += position.y * force_length;
      total_position.z += position.z * force_length;
#endif
      state.contact_positions.push_back(contact_position);

      // sum of all the force amplitudes
      total_force_lengths += force_length;
#if GAZEBO_MAJOR_VERSION >= 7
      total_normal_lengths += normal.Length();
#else
      total_normal_lengths += normal.GetLength();
#endif

      // set contact depth, interpenetration
      state.depths.push_back(contact.depth(j));
    }
    // fill contact message
    state.total_wrench = total_wrench;
    this->contact_state_msg_.states.push_back(state);
  }

  // normalize the normal
  if (total_normal_lengths != 0)
  {
    total_normal.x = total_normal.x / total_normal_lengths;
    total_normal.y = total_normal.y / total_normal_lengths;
    total_normal.z = total_normal.z / total_normal_lengths;
  }

  // compute average
  if (total_force_lengths != 0)
  {
    average_position.x = total_position.x / total_force_lengths;
    average_position.y = total_position.y / total_force_lengths;
    average_position.z = total_position.z / total_force_lengths;
  }
  else
  {
    average_position.x = 0.0;
    average_position.y = 0.0;
    average_position.z = 0.0;
  }

  // fill tactile message
  tactile_contact_msg_.name = bumper_topic_name_;
  tactile_contact_msg_.position = average_position;
  tactile_contact_msg_.normal = total_normal;
  tactile_contact_msg_.wrench = total_wrench;

  // adjust time
  this->tactile_contact_msg_.header.stamp = contact_time;
  this->contact_state_msg_.header.stamp = contact_time;

  // publish both messages
  this->tactile_pub_.publish(this->tactile_contact_msg_);
  this->contact_pub_.publish(this->contact_state_msg_);
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosContact::ContactQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
