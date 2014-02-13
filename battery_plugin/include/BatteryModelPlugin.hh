/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _GAZEBO_BATTERY_MODEL_PLUGIN_HH_
#define _GAZEBO_BATTERY_MODEL_PLUGIN_HH_


#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/gazebo.hh>

<<<<<<< local
#include <geometry_msgs/Pose.h>

=======
>>>>>>> other
#include <gazebo_plugins/PubQueue.h>
#include "battery_plugin/Battery.h"

<<<<<<< local
#include <geometry_msgs/Twist.h>

=======
>>>>>>> other
namespace gazebo
{
  class BatteryModelPlugin : public ModelPlugin
  {
    public: BatteryModelPlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();
    public: virtual  ~BatteryModelPlugin();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    private: void OnUpdate();

<<<<<<< local
    private: void OnVelMsg(ConstPosePtr &_msg);
=======
    private: void OnMoveCmdMsg(ConstPosePtr &_msg);
>>>>>>> other

    private: void WriteBatteryState(const common::Time &_simTime,
                                    const common::Time &_wallTime,
                                    bool _force);

<<<<<<< local
    private: void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

=======
>>>>>>> other
    private: double lastUpdate;

    private: transport::NodePtr node;
<<<<<<< local
    private: transport::SubscriberPtr velSub;
=======
    private: transport::SubscriberPtr moveSub;
>>>>>>> other
    private: physics::JointPtr leftWheel, rightWheel;
    private: double wheelSpeed[2];
    private: double torque;
    private: double wheelSeparation;
    private: double wheelRadius;

    private: double initCharge;         // 0-100%
    private: double battCharge;         // 0-100%
    private: double ratedCapacity;      // Ah  
    private: double battCapacity;      // Ah  
    private: double nominalVoltage;     // V
    private: double nominalDischargeCurrent; // A

<<<<<<< local
    
    private: ros::Subscriber cmd_vel_subscriber_;

    private: ros::Publisher cmd_vel_throttle_publisher_;

=======
>>>>>>> other
    /// \brief The absolute wall time when the run started
    private: common::Time runStartTimeWall;

    /// \brief Wall time at which TurtleBot achieved the last checkpoint.
    private: gazebo::common::Time stopTimeWall;

    /// \brief Current sim time used to track whether or not there was a reset.
    private: gazebo::common::Time currentSimTime;

    /// \brief Sim time elapsed since crossing the first gate
    private: common::Time elapsedTimeSim;

    /// \brief Name of the file that we're writing battery data to
    private: boost::filesystem::path batteryFilePath;

    /// \brief The stream associated with batteryFilePath
    private: std::ofstream batteryFileStream;

    /// \brief When we last wrote battery data to disk
    private: common::Time prevBatteryTime;
    private: common::Time prevUpdateTime;

    /// \brief publisher of battery status
    private: ros::Publisher pubBatteryMsg;
    private: PubQueue<battery_plugin::Battery>::Ptr pubBatteryMsgQueue;

    /// \brief ros node handle
    private: ros::NodeHandle *rosNode;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue* pmq;
    private: boost::thread deferredLoadThread;

    private: event::ConnectionPtr updateConnection;

    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

  };
}
#endif
