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
 * Desc: Bumper controller
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 */

#include <map>
#include <string>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>

#include <tf/tf.h>

#include <gazebo_ros_tactile/gazebo_ros_tactile.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <urdf_tactile/tactile.h>
#include <urdf/sensor.h>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosTactile)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTactile::GazeboRosTactile() : SensorPlugin()
{
	//std Vector befüllen mit gz Vector3, der alleCenter und Normals enthält
	//sinnlos? Positionen können sich ändern
	//parsen im Constructor? Sprich Vektor mit Pointern? Sensoren oder Texels?
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTactile::~GazeboRosTactile()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
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

# if GAZEBO_MAJOR_VERSION >= 7
  std::string worldName = _parent->WorldName();
# else
  std::string worldName = _parent->GetWorldName();
# endif
  this->world_ = physics::get_world(worldName);

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  // "publishing contact/collisions to this topic name: "
  //   << this->bumper_topic_name_ << std::endl;
  this->bumper_topic_name_ = "bumper_states";
  if (_sdf->HasElement("bumperTopicName"))
    this->bumper_topic_name_ =
      _sdf->GetElement("bumperTopicName")->Get<std::string>();

  // "transform contact/collisions pose, forces to this body (link) name: "
  //   << this->frame_name_ << std::endl;
  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("bumper plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  // if frameName specified is "world", "/map" or "map" report back
  // inertial values in the gazebo world
  if (this->local_link_ == NULL && this->frame_name_ != "world" &&
    this->frame_name_ != "/map" && this->frame_name_ != "map")
  {
    // lock in case a model is being spawned
    //boost::recursive_mutex::scoped_lock lock(*gazebo::Simulator::Instance()->GetMRMutex());
    // look through all models in the world, search for body
    // name that matches frameName
    physics::Model_V all_models = world_->GetModels();
    for (physics::Model_V::iterator iter = all_models.begin();
      iter != all_models.end(); iter++)
    {
      if (*iter) this->local_link_ =
        boost::dynamic_pointer_cast<physics::Link>((*iter)->GetLink(this->frame_name_));
      if (this->local_link_) break;
    }

      // not found
    if (!this->local_link_)
    {
      ROS_INFO("gazebo_ros_bumper plugin: frameName: %s does not exist"
                " using world",this->frame_name_.c_str());
    }
  }


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
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);
  
  //TODO Parse here
  // class TactileSensor : urdf::SensorBase
  
  this-> sensors = urdf::parseSensorsFromParam("robot_description", urdf::getSensorParser("tactile"));
  this-> numOfSensors=sensors.size();
  this->tactile_state_msg_.sensors.resize(numOfSensors); //or place it at ##? //implement it in the initiliazation, less memory allocating operations
  
  //this-> numOfTaxels=12;//taxels_.size();
  this->taxelPositions.resize(numOfSensors);//, std::vector<gazebo::math::Vector3>(numOfTaxels, gazebo::math::Vector3(0, 0, 0)));
  this->taxelNormals.resize(numOfSensors);//, std::vector<gazebo::math::Vector3>(numOfTaxels, gazebo::math::Vector3(0, 0, 0)));
  //unsigned int w=12;
  this->numOfTaxels.resize(numOfSensors,12);
  
  for(unsigned int i=0; i < numOfSensors; i++){
	  this->numOfTaxels[i]=12;
	  this->tactile_state_msg_.sensors[i].values.resize(numOfTaxels[i]);
	  this->taxelPositions[i].resize(numOfTaxels[i]);
	  this->taxelNormals[i].resize(numOfTaxels[i]);
	  // slso resize taxelpos and taxel normal
	  for(unsigned int j=0; j < numOfTaxels[i]; j++){
		  this->taxelNormals[i][j]=gazebo::math::Vector3(i, j, 0);
		  this->taxelPositions[i][j]=gazebo::math::Vector3(0, i, j);
		  
	  }
	  
  }

/*
  this->contact_pub_ = this->rosnode_->advertise<gazebo_msgs::ContactsState>(
    std::string(this->bumper_topic_name_), 1);*/
    
  this->tactile_pub_ = this->rosnode_->advertise<tactile_msgs::TactileState>(
    std::string(this->bumper_topic_name_), 1);

  // Initialize
  // start custom queue for contact bumper
  this->callback_queue_thread_ = boost::thread(
      boost::bind(&GazeboRosTactile::ContactQueueThread, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = this->parentSensor->ConnectUpdated(
     boost::bind(&GazeboRosTactile::OnContact, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTactile::OnContact()
{
  if (this->tactile_pub_.getNumSubscribers() <=0) //contact_pub_.getNumSubscribers() <= 0)
    return;

  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  /// \TODO: need a time for each Contact in i-loop, they may differ
  this->tactile_state_msg_.header.frame_id = this->frame_name_;
  common::Time meastime = this->parentSensor->GetLastMeasurementTime();
  this->tactile_state_msg_.header.stamp = ros::Time(meastime.sec, meastime.nsec);
  //this->contact_state_msg_.header.stamp = ros::Time(contacts.time().sec(),
  //                             contacts.time().nsec());
  //TODO this->tactile_state_msg_.header.seq=?

  // get reference frame (body(link)) pose and subtract from it to get
  // relative force, torque, position and normal vectors
  math::Pose pose, frame_pose;
  math::Quaternion rot, frame_rot;
  math::Vector3 pos, frame_pos;

  // Get frame orientation if frame_id is given */
  if (local_link_)
  {
    frame_pose = local_link_->GetWorldPose();  //-this->myBody->GetCoMPose();->GetDirtyPose();
    frame_pos = frame_pose.pos;
    frame_rot = frame_pose.rot;
  }
  else
  {
    // no specific frames specified, use identity pose, keeping
    // relative frame at inertial origin
    frame_pos = math::Vector3(0, 0, 0);
    frame_rot = math::Quaternion(1, 0, 0, 0);  // gazebo u,x,y,z == identity
    frame_pose = math::Pose(frame_pos, frame_rot);
  }


  ROS_INFO_STREAM_THROTTLE(1.0, "frame_pos" << frame_pos.x << "frame_rot" << frame_rot.x);

  // set contact states size
  this->contact_state_msg_.states.clear();
  
  //this->tactile_state_msg_.sensors.clear(); //is this the right way? //std::vector assign or close, to put zeros for values, now: implicitly done

  
  //tactile::urdf::SensorMap sensors;
for (unsigned int m=0; m < this->numOfSensors; m++){	//Loop over Sensors
  for (unsigned int k =0; k < this->numOfTaxels[m]; k++) //Loop over taxels //taxelpos.size();k++)
  {
  
    sensor_msgs::ChannelFloat32 &tSensor = this->tactile_state_msg_.sensors[m];
	 
	 
	 
	 double finalProjectedForce=0.0; //same
	  
	  
	  
	  // Loop over Collisions
	  // GetContacts returns all contacts on the collision body
	  unsigned int contactsPacketSize = contacts.contact_size();
	  for (unsigned int i = 0; i < contactsPacketSize; ++i)
	  {
		  // Create a ContactState
		  //gazebo_msgs::ContactState state;
		  gazebo::msgs::Contact contact = contacts.contact(i);
		  /*
		  state.collision1_name = contact.collision1();
		  state.collision2_name = contact.collision2();
		  std::ostringstream stream;
		  stream << "Debug:  i:(" << i << "/" << contactsPacketSize
		  << ")     my geom:" << state.collision1_name
		  << "   other geom:" << state.collision2_name
		  << "         time:" << ros::Time(contact.time().sec(), contact.time().nsec())
		  << std::endl;
		  state.info = stream.str();
		  
		  state.wrenches.clear();
		  state.contact_positions.clear();
		  state.contact_normals.clear();
		  state.depths.clear();
		  
		  //create a tactile state
		  tactile_msgs::TactileState tState;
		  
		  //sensor_msgs::ChannelFloat32 tSensors;##

		  
		  // sum up all wrenches for each DOF
		  geometry_msgs::Wrench total_wrench;
		  total_wrench.force.x = 0;
		  total_wrench.force.y = 0;
		  total_wrench.force.z = 0;
		  total_wrench.torque.x = 0;
		  total_wrench.torque.y = 0;
		  total_wrench.torque.z = 0;
		  */
		  
		  //Deklarationen
		  unsigned int contactGroupSize = contact.position_size();
		  double normalForceScalar;
		  double stdDev=2.0;
		  double distance;
		  double critDist=1.0;
		  //double finalProjectedForce=0.0;
		  double p=1.0;	//Multiplicator
		  const double pi= 3.14159265359;
		  
		  
		  for (unsigned int j = 0; j < contactGroupSize; ++j)
		  {
		  
		  
		  //math::Pose frame_pose;
		  //frame_pos = math::Vector3(0, 0, 0);
		  //
		  ROS_INFO_STREAM_THROTTLE(1.0,"state.contact_positions.x:" << frame_pos.x);
		  //double distance= sqrt((frame_pos.x-state.contact_positions.x)*(frame_pos.x-state.contact_positions.x));//sqrt((frame_pos.x-state.contact_positions.x)*(frame_pos.x-state.contact_positions.x)+(frame_pos.y-state.contact_positions.y)*(frame_pos.y-state.contact_positions.y)+(frame_pos.z-state.contact_positions.z)*(frame_pos.z-state.contact_positions.z)); 
		  
		  // Get force, torque and rotate into user specified frame.
		  // frame_rot is identity if world is used (default for now)
		  math::Vector3 force = frame_rot.RotateVectorReverse(math::Vector3(
		  contact.wrench(j).body_1_wrench().force().x(),
		  contact.wrench(j).body_1_wrench().force().y(),
		  contact.wrench(j).body_1_wrench().force().z()));
		  math::Vector3 torque = frame_rot.RotateVectorReverse(math::Vector3(
		  contact.wrench(j).body_1_wrench().torque().x(),
		  contact.wrench(j).body_1_wrench().torque().y(),
		  contact.wrench(j).body_1_wrench().torque().z()));
		  
		  
		  
		  //////////////////////////BEGIN OF FORCE TRANSFORMTION
		  // transform contact positions into relative frame
		  // set contact positions
		  gazebo::math::Vector3 position = frame_rot.RotateVectorReverse(
		  math::Vector3(contact.position(j).x(),
		  contact.position(j).y(),
		  contact.position(j).z()) - frame_pos);
		  geometry_msgs::Vector3 contact_position;
		  contact_position.x = position.x;
		  contact_position.y = position.y;
		  contact_position.z = position.z;
		  //state.contact_positions.push_back(contact_position);
		  
		  // rotate normal into user specified frame.
		  // frame_rot is identity if world is used.
		  math::Vector3 normal = frame_rot.RotateVectorReverse(
		  math::Vector3(contact.normal(j).x(),
		  contact.normal(j).y(),
		  contact.normal(j).z()));
		  // set contact normals
		  geometry_msgs::Vector3 contact_normal;
		  contact_normal.x = normal.x;
		  contact_normal.y = normal.y;
		  contact_normal.z = normal.z;
		  //state.contact_normals.push_back(contact_normal);
		  
		  ////////////////////////////////////END OF FORCE TRANSFORMATION
		  
		  
		  
		  
		  //TEST Dotproduct
		  //distance=math::Vector3::Dot(transformedForceAP, transformedForceAP)const;
		  
		  
		  //Distanzbestimmen zwischen Kraft und Taxelzentrum
		  distance=sqrt(pow((position.x-taxelPositions[m][k].x),2)+pow((position.y-taxelPositions[m][k].y),2)+pow((position.z-taxelPositions[m][k].z),2));

		  if(distance>critDist){
			finalProjectedForce+=0.0; // bzw continue;
			//nächster Durchlauf
		  }
		  else{
			//project Force on normal
			normalForceScalar=(this->taxelNormals[m][k].x*force.x+this->taxelNormals[m][k].y*force.y+this->taxelNormals[m][k].z*force.z)/sqrt(pow(this->taxelNormals[m][k].x,2)+pow(this->taxelNormals[m][k].y,2)+pow(this->taxelNormals[m][k].z,2)); //Normalize the taxelNormals, lookup if it norm.
		  
			if(normalForceScalar > 0){
				//Normalverteilung erzeugen
				p=exp(-(distance*distance/(2*stdDev*stdDev)))/sqrt(2*pi*stdDev*stdDev);
				finalProjectedForce+=p*normalForceScalar;
			}
		  
		  }
		  
		  
		  
		  
		  
		  // set contact depth, interpenetration
		  //state.depths.push_back(contact.depth(j));
		  }//END FOR contactGroupSize
		  
		  
		  //state.total_wrench = total_wrench;
		  //this->contact_state_msg_.states.push_back(state);
		  
	  }// END FOR contactsPacketSize (What does this mean?)
	  
	  //is the force to small first contact thershold
	  this->tactile_state_msg_.sensors[m].values[k]=finalProjectedForce;//tSensor.values.push_back(finalProjectedForce); //if changed above, access, not pubsh_back
  }//END FOR Taxels
  this->tactile_pub_.publish(this->tactile_state_msg_);
}//END FOR Sensors
  
  }
  
  
  ////////////////////////////////////////////////////////////////////////////////
  // Put laser data to the interface
  void GazeboRosTactile::ContactQueueThread()
  {
  static const double timeout = 0.01;
  
  while (this->rosnode_->ok())
  {
  this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
  }
  }
