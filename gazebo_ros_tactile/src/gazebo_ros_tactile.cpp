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
 * Author: Dennis Stoppel and Guillaume Walck
 * Based on Bumper controller
 * Original Author: Nate Koenig
 * Date: 09 Sept. 2008
 */

/*
 * FRAME info
 * Gazebo provides positions and normals of contact in world frame
 * BUT Gazebo provides force and torques in link frame
 */

#include <map>
#include <string>
#include <vector>

#include <gazebo/common/Exception.hh>
#if GAZEBO_MAJOR_VERSION >= 7
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#endif
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <sdf/Param.hh>
#include <sdf/sdf.hh>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_ros_tactile/gazebo_ros_tactile.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <tf/tf.h>

#include <urdf_tactile/parser.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosTactile)

// //////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTactile::GazeboRosTactile()
  : SensorPlugin()
  , is_initialized_(false)
  , use_gaussianDistribution_(true)
  , update_rate_(TACT_PLUGIN_DEFAULT_UPDATE_RATE)
  , minForce_(TACT_PLUGIN_DEFAULT_MINFORCE)
{
}

// //////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTactile::~GazeboRosTactile()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

// //////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosTactile::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
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
#ifdef PUB_DEBUG_CONTACT_STATE
  this->bumper_topic_name_ = "bumper_states";
  if (_sdf->HasElement("bumperTopicName"))
    this->bumper_topic_name_ = _sdf->GetElement("bumperTopicName")->Get<std::string>();
#endif
  this->tactile_topic_name_ = "tactile_states";
  if (_sdf->HasElement("tactileTopicName"))
    this->tactile_topic_name_ = _sdf->GetElement("tactileTopicName")->Get<std::string>();
  if (_sdf->HasElement("constantDistribution"))
  {
    this->use_gaussianDistribution_ = false;
    sdf::ElementPtr constantElement = _sdf->GetElement("constantDistribution");
    if (constantElement->HasElement("distance"))
    {
      this->maxDistance_ = constantElement->GetElement("distance")->Get<double>();
    }
    else
    {
      this->maxDistance_ = TACT_PLUGIN_DEFAULT_MAXDISTANCE;
      ROS_INFO_STREAM("no distance parameter found, using default value: " << TACT_PLUGIN_DEFAULT_MAXDISTANCE);
    }
    if (constantElement->HasElement("angle"))
    {
      double maxAngle = constantElement->GetElement("angle")->Get<double>();
      this->minAngleProjection_ = std::cos(maxAngle / 180.0 * M_PI);
    }
    else
    {
      this->minAngleProjection_ = std::cos(TACT_PLUGIN_DEFAULT_ANGLE_THRESHOLD);
      ROS_INFO_STREAM("no angle parameter found, using default value: " << TACT_PLUGIN_DEFAULT_ANGLE_THRESHOLD);
    }
  }
  else
  {
    double stdDev;
    this->gaussianDistanceCoefficient_ = 2 * TACT_PLUGIN_DEFAULT_MAXDISTANCE * TACT_PLUGIN_DEFAULT_MAXDISTANCE;
    this->gaussianAngleCoefficient_ = 2 * TACT_PLUGIN_DEFAULT_ANGLE_THRESHOLD * TACT_PLUGIN_DEFAULT_ANGLE_THRESHOLD;
    if (_sdf->HasElement("gaussianDistribution"))
    {
      sdf::ElementPtr gaussianElement = _sdf->GetElement("gaussianDistribution");
      if (gaussianElement->HasElement("distance"))
      {
        stdDev = gaussianElement->GetElement("distance")->Get<double>();
        this->gaussianDistanceCoefficient_ = 2 * stdDev * stdDev;
      }
      else
        ROS_INFO_STREAM("no distance parameter found, using default value: " << TACT_PLUGIN_DEFAULT_MAXDISTANCE);

      if (gaussianElement->HasElement("angle"))
      {
        stdDev = gaussianElement->GetElement("angle")->Get<double>();
        this->gaussianAngleCoefficient_ = 2 * stdDev * stdDev;
      }
      else
        ROS_INFO_STREAM("no angle parameter found, using default value: " << TACT_PLUGIN_DEFAULT_ANGLE_THRESHOLD);
    }
    else
      ROS_WARN_STREAM("No distribution selected for tactile plugin, using default gaussian distribution");
  }

  if (_sdf->HasElement("minForce"))
    this->minForce_ = _sdf->GetElement("minForce")->Get<double>();

  if (_sdf->HasElement("updateRate"))
  {
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    if (this->update_rate_ < 0.0)
      this->update_rate_ = 0;
  }
  if (this->update_rate_ == 0)
    this->update_period_ = 0.0;
  else
    this->update_period_ = common::Time(0, common::Time::SecToNano(1.0 / this->update_rate_));
  this->last_update_time_ = common::Time(0);
  // get the body (link) name to which the sensor is attached
#if GAZEBO_MAJOR_VERSION >= 7
  std::string parentName = _parent->ParentName();
#else
  std::string parentName = _parent->GetParentName();
#endif
  local_name_ = parentName.substr(parentName.find_last_of(':') + 1);
  ROS_INFO_STREAM("contact plugin belongs to link named: " << local_name_);

  // by default use collision_name as parentName:local_name_ _collision
  collision_name_ = parentName + "::" + local_name_ + "_collision";
  ROS_INFO_STREAM("default collision name: " << collision_name_);

  // try access the real/given collision name used for the contact sensor (one level up from the plugin sdf)
  if (_sdf->GetParent()->HasElement("contact"))
  {
    if (_sdf->GetParent()->GetElement("contact")->HasElement("collision"))
    {
      collision_name_ = _sdf->GetParent()->GetElement("contact")->GetElement("collision")->Get<std::string>();
      collision_name_ = parentName + "::" + collision_name_;
      ROS_INFO_STREAM("real/given collision name from sdf: " << collision_name_);
    }
  }

  // "transform contact/collisions pose, forces to this body (link) name: "
  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("tactile plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Do NOT initialize transform frames here, because the model is not finished loading, so does not exist in the world
  // TransformFrameInit()

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix, for publishing a correct frame_id in the messages.
  // DO NOT use it for internal computations as there is no such tf_prefix in gazebo.
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_id_name_ = tf::resolve(prefix, this->frame_name_);

  // Begin parsing
  try
  {
    this->sensors = urdf::tactile::parseSensorsFromParam(robot_namespace_ + "/robot_description");
  }
  catch (const std::runtime_error& e)
  {
    std::cout << " Tactile plugin loading failed with message '" << e.what() << "'\n";
    return;
  }

  this->taxelPositions.clear();
  this->taxelNormals.clear();
  this->numOfSensors = 0;

#if GAZEBO_MAJOR_VERSION >= 7
  std::string gzSensorName = _parent->Name();
#else
  std::string gzSensorName = _parent->GetName();
#endif
  for (auto it = sensors.begin(), end = sensors.end(); it != end; ++it)
  {
    ROS_DEBUG_STREAM("sensorName tactile " << it->second->name_);
    ROS_DEBUG_STREAM("sensorName gz " << gzSensorName);
    const urdf::tactile::TactileSensorConstSharedPtr &sensor = it->second;
    if (!sensor)
      continue;  // some other sensor than tactile

    if (it->second->name_ != gzSensorName)
      continue;

    ROS_DEBUG_STREAM("Matching: " << gzSensorName);

    if (sensor->taxels_.size() == 0)
    {
      // if the taxel has no size, sensor is assumed to be an array
      // if(sensor -> array_ -> rows != 0 && sensor -> array_ -> cols != 0) //test rows and cols

      // Get parameter form tactile array (urdf)
      urdf::tactile::Vector2<double> spacing = sensor->array_->spacing;
      int rows = sensor->array_->rows;
      int cols = sensor->array_->cols;
      urdf::tactile::Vector2<double> taxelSize = sensor->array_->size;
      urdf::tactile::Vector2<double> offset = sensor->array_->offset;

      // Intiaization: push back and resize
      this->numOfTaxels.push_back((rows) * (cols));
#if GAZEBO_MAJOR_VERSION >= 7
      this->taxelNormals.push_back(
          std::vector<ignition::math::Vector3d>(numOfTaxels[this->numOfSensors], ignition::math::Vector3d(0, 0, 0)));
      this->taxelPositions.push_back(
          std::vector<ignition::math::Vector3d>(numOfTaxels[this->numOfSensors], ignition::math::Vector3d(0, 0, 0)));
#else
      this->taxelNormals.push_back(
          std::vector<gazebo::math::Vector3>(numOfTaxels[this->numOfSensors], gazebo::math::Vector3(0, 0, 0)));
      this->taxelPositions.push_back(
          std::vector<gazebo::math::Vector3>(numOfTaxels[this->numOfSensors], gazebo::math::Vector3(0, 0, 0)));
#endif
      sensor_msgs::ChannelFloat32 channel;
      channel.values.resize(numOfTaxels[this->numOfSensors]);
      this->tactile_state_msg_.sensors.push_back(channel);

      // Fill an array with the positions and normals of the grid cells of the sensor
      // if the order is col-major
      if (sensor->array_->order == 1)
      {
        for (int row_idx = 0; row_idx < rows; row_idx++)
        {
          for (int col_idx = 0; col_idx < cols; col_idx++)
          {
#if GAZEBO_MAJOR_VERSION >= 7
            this->taxelPositions[this->numOfSensors][cols * row_idx + col_idx] =
                ignition::math::Vector3d((-offset.x + col_idx * (spacing.x)), (-offset.y + row_idx * (spacing.y)), 0);

            this->taxelNormals[this->numOfSensors][cols * row_idx + col_idx] = ignition::math::Vector3d(0, 0, 1.0);
#else
            this->taxelPositions[this->numOfSensors][cols * row_idx + col_idx] =
                gazebo::math::Vector3((-offset.x + col_idx * (spacing.x)), (-offset.y + row_idx * (spacing.y)), 0);

            this->taxelNormals[this->numOfSensors][cols * row_idx + col_idx] = gazebo::math::Vector3(0, 0, 1.0);
#endif
          }
        }
      }
      // if the order is row-major
      else if (sensor->array_->order == 0)
      {
        for (int row_idx = 0; row_idx < rows; row_idx++)
        {
          for (int col_idx = 0; col_idx < cols; col_idx++)
          {
#if GAZEBO_MAJOR_VERSION >= 7
            this->taxelPositions[this->numOfSensors][rows * col_idx + row_idx] =
                ignition::math::Vector3d((-offset.x + col_idx * (spacing.x)), (-offset.y + row_idx * (spacing.y)), 0);

            this->taxelNormals[this->numOfSensors][cols * row_idx + col_idx] = ignition::math::Vector3d(0, 0, 1.0);
#else
            this->taxelPositions[this->numOfSensors][rows * col_idx + row_idx] =
                gazebo::math::Vector3((-offset.x + col_idx * (spacing.x)), (-offset.y + row_idx * (spacing.y)), 0);

            this->taxelNormals[this->numOfSensors][cols * row_idx + col_idx] = gazebo::math::Vector3(0, 0, 1.0);
#endif
          }
        }
      }
      // Incorrect order
      else
      {
        ROS_WARN_STREAM("Undefined order (neither row-major nor col-major): \t" << sensor->array_->order);
      }
    }
    else
    {
      // Intiaization: push back and resize
      this->numOfTaxels.push_back(sensor->taxels_.size());
#if GAZEBO_MAJOR_VERSION >= 7
      this->taxelNormals.push_back(
          std::vector<ignition::math::Vector3d>(numOfTaxels[this->numOfSensors], ignition::math::Vector3d(0, 0, 0)));
      this->taxelPositions.push_back(
          std::vector<ignition::math::Vector3d>(numOfTaxels[this->numOfSensors], ignition::math::Vector3d(0, 0, 0)));
#else
      this->taxelNormals.push_back(
          std::vector<gazebo::math::Vector3>(numOfTaxels[this->numOfSensors], gazebo::math::Vector3(0, 0, 0)));
      this->taxelPositions.push_back(
          std::vector<gazebo::math::Vector3>(numOfTaxels[this->numOfSensors], gazebo::math::Vector3(0, 0, 0)));
#endif
      sensor_msgs::ChannelFloat32 channel;
      channel.values.resize(numOfTaxels[this->numOfSensors]);
      this->tactile_state_msg_.sensors.push_back(channel);

      // Fill an array with the positions and normals of the taxels of the sensor
      for (unsigned int j = 0; j < numOfTaxels[this->numOfSensors]; j++)
      {
#if GAZEBO_MAJOR_VERSION >= 7
        this->taxelPositions[this->numOfSensors][j] =
            ignition::math::Vector3d(sensor->taxels_[j]->origin.position.x, sensor->taxels_[j]->origin.position.y,
                                     sensor->taxels_[j]->origin.position.z);
#else
        this->taxelPositions[this->numOfSensors][j] =
            gazebo::math::Vector3(sensor->taxels_[j]->origin.position.x, sensor->taxels_[j]->origin.position.y,
                                  sensor->taxels_[j]->origin.position.z);
#endif
        // normal=rotation times zAxis
        urdf::Vector3 zAxis(0, 0, 1);
        urdf::Vector3 urdfTaxelNormal = (sensor->taxels_[j]->origin.rotation) * zAxis;
#if GAZEBO_MAJOR_VERSION >= 7
        this->taxelNormals[this->numOfSensors][j] =
            ignition::math::Vector3d(urdfTaxelNormal.x, urdfTaxelNormal.y, urdfTaxelNormal.z);
#else
        this->taxelNormals[this->numOfSensors][j] =
            gazebo::math::Vector3(urdfTaxelNormal.x, urdfTaxelNormal.y, urdfTaxelNormal.z);
#endif
      }
    }
    this->tactile_state_msg_.sensors[numOfSensors].name = sensor->channel_;

    this->numOfSensors++;
  }

  if (numOfSensors == 0)
  {
    ROS_FATAL_STREAM("No matching for " << gzSensorName);
    return;
  }
  else if (numOfSensors > 1)
  {
    ROS_WARN_STREAM("Multiple matching for " << gzSensorName << " count: " << numOfSensors << "\n all matchings will "
                                                                                              "be published");
  }
#ifdef PUB_DEBUG_CONTACT_STATE
  this->contact_pub_ = this->rosnode_->advertise<gazebo_msgs::ContactsState>(std::string(this->bumper_topic_name_), 1);
#endif
  this->tactile_pub_ = this->rosnode_->advertise<tactile_msgs::TactileState>(std::string(this->tactile_topic_name_), 1);

  // Initialize
  // start custom queue for contact bumper
  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosTactile::ContactQueueThread, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = this->parentSensor->ConnectUpdated(boost::bind(&GazeboRosTactile::OnContact, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

// //////////////////////////////////////////////////////////////////////////////
// Initialize the transforms frames
void GazeboRosTactile::TransformFrameInit()
{
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
  if (this->my_link_ == NULL && this->frame_name_ != "world" && this->frame_name_ != "/map" && this->frame_name_ != "map")
  {
    // look through all models in the world, search for body
    // name that matches frameName
    for (physics::Model_V::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
    {
      if (*iter)
      {
        this->my_link_ = boost::dynamic_pointer_cast<physics::Link>((*iter)->GetLink(this->frame_name_));
      }
      if (this->my_link_)
        break;
    }

    // not found
    if (!this->my_link_)
    {
      ROS_INFO("gazebo_ros_bumper plugin: frameName: %s does not exist using world", this->frame_name_.c_str());
      ROS_DEBUG_STREAM("Model currently available:");
      for (physics::Model_V::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
      {
        if (*iter)
        {
          ROS_DEBUG_STREAM(" modelname: " << (*iter)->GetName());
          physics::Link_V all_links = (*iter)->GetLinks();
          ROS_DEBUG_STREAM("  available links :");
          for (physics::Link_V::iterator liter = all_links.begin(); liter != all_links.end(); ++liter)
          {
            ROS_DEBUG_STREAM("   scope name: " << (*liter)->GetScopedName() << " | name: " << (*liter)->GetName());
          }
        }
      }
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

  is_initialized_ = true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTactile::OnContact()
{
#ifdef PUB_DEBUG_CONTACT_STATE
  if (this->tactile_pub_.getNumSubscribers() <= 0 && contact_pub_.getNumSubscribers() <= 0)
    return;
#else
  if (this->tactile_pub_.getNumSubscribers() <= 0)
    return;
#endif

  // initialize transform frames once
  if (!is_initialized_)
  {
    TransformFrameInit();
  }

  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  std::vector<std::vector<float> > sensorForces;  // temporary vector of distributed force over all taxels of all
                                                  // sensors
  ROS_DEBUG_STREAM_NAMED("oncontact_start", "number of contacts: " << contacts.contact_size());

  /// \TODO: need a time for each Contact in i-loop, they may differ
  this->tactile_state_msg_.header.frame_id = this->frame_id_name_;
#if GAZEBO_MAJOR_VERSION >= 7
  common::Time meastime = this->parentSensor->LastMeasurementTime();
#else
  common::Time meastime = this->parentSensor->GetLastMeasurementTime();
#endif
  // handle updateRate
  if (meastime - this->last_update_time_ < this->update_period_)
    return;  // currently we discard data, maybe accumulating on top of the accumulation that
             // gazebo internally does between sensor calls might be useful
  this->last_update_time_ = meastime;

  this->tactile_state_msg_.header.stamp = ros::Time(meastime.sec, meastime.nsec);
#ifdef PUB_DEBUG_CONTACT_STATE
  this->contact_state_msg_.header.frame_id = this->frame_id_name_;
  this->contact_state_msg_.header.stamp = ros::Time(meastime.sec, meastime.nsec);
#endif

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
    frame_pose = my_link_->GetWorldPose().Ign();  // -this->myBody->GetCoMPose();->GetDirtyPose();
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
    frame_rot = math::Quaternion(1, 0, 0, 0);
    frame_pose = math::Pose(frame_pos, frame_rot);
#endif
  }

#ifdef PUB_DEBUG_CONTACT_STATE
  // set contact states size
  this->contact_state_msg_.states.clear();
#endif

  unsigned int contactsPacketSize = contacts.contact_size();
  unsigned int contactGroupSize;

  double normalForceScalar;
  double distance;

  double p_sum = 0.0;
  double forceDirection = 0.0;

  // for every timestep init the sensor to zero
  for (unsigned int m = 0; m < this->numOfSensors; m++)
  {                                                          // Loop over Sensors
    for (unsigned int k = 0; k < this->numOfTaxels[m]; k++)  // Loop over taxels
    {
      this->tactile_state_msg_.sensors[m].values[k] = 0.0f;
    }
  }

  for (unsigned int i = 0; i < contactsPacketSize; ++i)
  {
    // Create a ContactState
    gazebo_msgs::ContactState state;

    gazebo::msgs::Contact contact = contacts.contact(i);

    state.collision1_name = contact.collision1();
    state.collision2_name = contact.collision2();
    std::ostringstream stream;
    stream << "Debug:  i:(" << i + 1 << "/" << contactsPacketSize << ")     my geom:" << state.collision1_name
           << "   other geom:" << state.collision2_name
           << "         time:" << ros::Time(contact.time().sec(), contact.time().nsec()) << std::endl;
    state.info = stream.str();

    // is body 1 the collision body attached to the sensor plugin ?
    bool switch_body = false;
    if (state.collision2_name == collision_name_)
      switch_body = true;
#ifdef PUB_DEBUG_CONTACT_STATE
    state.wrenches.clear();
    state.contact_positions.clear();
    state.contact_normals.clear();
    state.depths.clear();
#endif
    // sum up all wrenches for each DOF
    geometry_msgs::Wrench total_wrench;
    total_wrench.force.x = 0;
    total_wrench.force.y = 0;
    total_wrench.force.z = 0;
    total_wrench.torque.x = 0;
    total_wrench.torque.y = 0;
    total_wrench.torque.z = 0;

    contactGroupSize = contact.position_size();
    for (unsigned int j = 0; j < contactGroupSize; ++j)
    {
      // Get force, torque. They are in local frame already.
      // forward transform them to world and then
      // and rotate into user specified frame.
      // frame_rot is identity if world is used (default for now)

      ROS_DEBUG_STREAM_NAMED("oncontact",
                             "collision body1 name: " << contact.wrench(j).body_1_name()
                                                      << ", body2 name: " << contact.wrench(j).body_2_name());
      // select the correct body
      gazebo::msgs::Wrench source_wrench;
      if (switch_body)
      {
        ROS_DEBUG_NAMED("oncontact", "using body2");
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
        force =
            ignition::math::Vector3d(source_wrench.force().x(), source_wrench.force().y(), source_wrench.force().z());
        torque = ignition::math::Vector3d(source_wrench.torque().x(), source_wrench.torque().y(),
                                          source_wrench.torque().z());
      }
      else
      {
        force = frame_rot.RotateVectorReverse(local_rot.RotateVector(
            ignition::math::Vector3d(source_wrench.force().x(), source_wrench.force().y(), source_wrench.force().z())));
        torque = frame_rot.RotateVectorReverse(local_rot.RotateVector(ignition::math::Vector3d(
            source_wrench.torque().x(), source_wrench.torque().y(), source_wrench.torque().z())));
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

      // set wrenches
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
#ifdef PUB_DEBUG_CONTACT_STATE
      state.wrenches.push_back(wrench);
#endif
      total_wrench.force.x += wrench.force.x;
      total_wrench.force.y += wrench.force.y;
      total_wrench.force.z += wrench.force.z;
      total_wrench.torque.x += wrench.torque.x;
      total_wrench.torque.y += wrench.torque.y;
      total_wrench.torque.z += wrench.torque.z;

      // ///////////////////////BEGIN OF FORCE TRANSFORMATION
      // transform contact positions into relative frame
      // set contact positions
      geometry_msgs::Vector3 contact_position;
#if GAZEBO_MAJOR_VERSION >= 7
      ignition::math::Vector3d position = frame_rot.RotateVectorReverse(
          ignition::math::Vector3d(contact.position(j).x(), contact.position(j).y(), contact.position(j).z()) -
          frame_pos);
      contact_position.x = position.X();
      contact_position.y = position.Y();
      contact_position.z = position.Z();
#else
      gazebo::math::Vector3 position = frame_rot.RotateVectorReverse(
          math::Vector3(contact.position(j).x(), contact.position(j).y(), contact.position(j).z()) - frame_pos);
      contact_position.x = position.x;
      contact_position.y = position.y;
      contact_position.z = position.z;
#endif
#ifdef PUB_DEBUG_CONTACT_STATE
      state.contact_positions.push_back(contact_position);
#endif

      // rotate normal from world into user specified frame.
      // frame_rot is identity if world is used.
#if GAZEBO_MAJOR_VERSION >= 7
      ignition::math::Vector3d normal = frame_rot.RotateVectorReverse(
          ignition::math::Vector3d(contact.normal(j).x(), contact.normal(j).y(), contact.normal(j).z()));
#else
      math::Vector3 normal = frame_rot.RotateVectorReverse(
          math::Vector3(contact.normal(j).x(), contact.normal(j).y(), contact.normal(j).z()));
#endif
      // set contact normals
#if GAZEBO_MAJOR_VERSION >= 7
      normal = (switch_body ? 1.0 : -1.0) * normal;
#else
      normal.x = (switch_body ? 1.0 : -1.0) * normal.x;
      normal.y = (switch_body ? 1.0 : -1.0) * normal.y;
      normal.z = (switch_body ? 1.0 : -1.0) * normal.z;
#endif
#ifdef PUB_DEBUG_CONTACT_STATE
      geometry_msgs::Vector3 contact_normal;
#if GAZEBO_MAJOR_VERSION >= 7
      contact_normal.x = normal.X();
      contact_normal.y = normal.Y();
      contact_normal.z = normal.Z();
#else
      contact_normal.x = normal.x;
      contact_normal.y = normal.y;
      contact_normal.z = normal.z;
#endif
      state.contact_normals.push_back(contact_normal);

      // set contact depth, interpenetration
      state.depths.push_back(contact.depth(j));
#endif
      // //////////////////////////////////END OF FORCE TRANSFORMATION

      // compute force amplitude along the contact normal (discard shear force)
#if GAZEBO_MAJOR_VERSION >= 7
      normalForceScalar = normal.Dot(force) * (-1.0);
#else
      normalForceScalar =
          (contact_normal.x * force.x + contact_normal.y * force.y + contact_normal.z * force.z) * (-1.0);
#endif
      ROS_DEBUG_STREAM_NAMED("oncontact", " normalForceScalar  " << normalForceScalar);

      // reset the sum of the distribution for this new contact
      p_sum = 0.0;
      sensorForces.clear();
      for (unsigned int m = 0; m < this->numOfSensors; m++)
      {  // Loop over Sensors
        std::vector<float> taxelForces(this->numOfTaxels[m],
                                       0.0);  // temporary vector of distributed force over all taxels of this sensor
        ROS_DEBUG_STREAM_NAMED("oncontact", " processing sensor " << m);
        for (unsigned int k = 0; k < this->numOfTaxels[m]; k++)  // Loop over taxels
        {
#if GAZEBO_MAJOR_VERSION >= 7
          // distance between force application point and taxelcenter
          distance = position.Distance(taxelPositions[m][k]);
          // compute angle between normal at application point and taxelnormal, to handle curvature
          forceDirection = normal.Dot(this->taxelNormals[m][k]);
#else
          // distance between force application point and taxelcenter
          distance =
              sqrt(pow((position.x - taxelPositions[m][k].x), 2) + pow((position.y - taxelPositions[m][k].y), 2) +
                   pow((position.z - taxelPositions[m][k].z), 2));
          // compute angle between normal at application point and taxelnormal, to handle curvature
          forceDirection = (normal.x * this->taxelNormals[m][k].x + normal.y * this->taxelNormals[m][k].y +
                            normal.z * this->taxelNormals[m][k].z);
#endif
          // considering pushing only, sensors detecting pulling forces are not considered,
          // because gazebo does not simulate stickyness
          if (normalForceScalar > 0)
          {
            if (use_gaussianDistribution_)
            {
              // Normal distribution relative to distance
              double p = exp(-(distance * distance / gaussianDistanceCoefficient_));  // normalization to one is done
                                                                                      // discretly later   / sqrt(2 * pi
                                                                                      // * stdDev * stdDev);
              // Normal distribution relative to angle
              ROS_DEBUG_STREAM_NAMED("oncontact", "distance:" << distance << " gaussianDist  " << p);
              double p_ang = exp(-((1.0 - forceDirection) * (1.0 - forceDirection) / gaussianAngleCoefficient_));
              ROS_DEBUG_STREAM_NAMED("oncontact", "angle:" << (1.0 - forceDirection) << " gaussianAngle  " << p_ang);
              p *= p_ang;
              // distribute the force to the taxel
              taxelForces[k] = p * normalForceScalar;
              // sum the discrete contributions for future normalization (depends on density of the taxels under that
              // contact and maxDistance_)
              p_sum += p;
              ROS_DEBUG_STREAM_NAMED("oncontact", " cell: " << k << " p: " << p << " p sum: " << p_sum);
            }
            else
            {  // binary decisions on consideration of this force for the current taxel
              // considering only contacts closer than maxDistance
              // binary direction check, to consider curvature
              if (distance < maxDistance_ && forceDirection > minAngleProjection_)
              {
                taxelForces[k] = normalForceScalar;
                p_sum += 1;
              }
              else
              {
                if (distance >= maxDistance_)
                  ROS_DEBUG_STREAM_NAMED("oncontact", " not contributing to cell " << k << " which is too far away ("
                                         << distance << ">" << maxDistance_ << ")");
                if (forceDirection <= std::cos(TACT_PLUGIN_DEFAULT_ANGLE_THRESHOLD))
                  ROS_DEBUG_STREAM_NAMED("oncontact", " not contributing to cell " << k
                                         << " forceDirection is not close enough to taxel normal");
              }
            }
          }
          else
          {
            ROS_DEBUG_STREAM_NAMED("oncontact", " not contributing to cell " << k << " normalForceScalar negative");
          }
        }  // END FOR Taxels
        sensorForces.push_back(taxelForces);
      }  // END FOR Sensors
      // normalize the pressure distribution to keep the sum of the distributed amplitudes over all taxels equal to the
      // force amplitude of that contact
      if (p_sum != 0)
      {
        ROS_DEBUG_STREAM_NAMED("oncontact_per_force", "contact:" << i << " contact group:" << j <<  " normalForceScalar  " << normalForceScalar << " p_sum "<< p_sum);
        for (unsigned int e = 0; e < this->numOfSensors; e++)
        {
          for (unsigned int f = 0; f < this->numOfTaxels[e]; f++)
          {
            this->tactile_state_msg_.sensors[e].values[f] += sensorForces[e][f] / p_sum;
          }
        }
      }
    }  // END FOR contactGroupSize
#ifdef PUB_DEBUG_CONTACT_STATE
    state.total_wrench = total_wrench;

    this->contact_state_msg_.states.push_back(state);
#endif
  }  // END FOR contactsPacketSize

  // Average over all contactPackets. Since gazebo accumulates contactPackets over each step between 2 calls of the
  // plugin, a different update_rate would artificially augment the distributed force if one does not average.
  // no averaging should be done on contactGroups because multiple contacts can occur during a single step and their
  // contribution should be added.
  if (contactsPacketSize > 0)
  {
    for (unsigned int e = 0; e < this->numOfSensors; e++)
    {
      for (unsigned int f = 0; f < this->numOfTaxels[e]; f++)
      {
        this->tactile_state_msg_.sensors[e].values[f] /= contactsPacketSize;
        // apply filtering
        if (this->tactile_state_msg_.sensors[e].values[f] < minForce_)
          this->tactile_state_msg_.sensors[e].values[f] = 0.0;
      }
    }
  }

#ifdef PUB_DEBUG_CONTACT_STATE
  this->contact_pub_.publish(this->contact_state_msg_);
#endif
  this->tactile_pub_.publish(this->tactile_state_msg_);
}

////////////////////////////////////////////////////////////////////////////////
// Put data to the interface
void GazeboRosTactile::ContactQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}  // namespace gazebo
