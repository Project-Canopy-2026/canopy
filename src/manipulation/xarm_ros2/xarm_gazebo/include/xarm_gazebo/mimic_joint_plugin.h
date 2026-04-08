/* 

Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Refer to: https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins

Modified for ROS2 compatibility and UFACTORY xArm gripper simulation

Modified by: Vinman <vinman.cub@gmail.com>
============================================================================*/

#ifndef XARM_GAZEBO_MIMIC_JOINT_PLUGIN_H
#define XARM_GAZEBO_MIMIC_JOINT_PLUGIN_H

// #include <rclcpp/rclcpp.hpp>
// Gazebo includes
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointForceCmd.hh>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gazebo {
  class GazeboMimicJointPluginPrivate;

  class GazeboMimicJointPlugin
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate {
  public:
    GazeboMimicJointPlugin();
    virtual ~GazeboMimicJointPlugin() override;

    virtual void Configure(const Entity &_entity, 
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &/*_eventMgr*/) override;
  
    virtual void PostUpdate(const UpdateInfo &/*_info*/, 
                            const EntityComponentManager &_ecm) override;

  private:
    std::unique_ptr<GazeboMimicJointPluginPrivate> dataPtr;
  };
}


#endif