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

#include "xarm_gazebo/mimic_joint_plugin.h"


namespace gazebo {
  struct GazeboMimicJointPluginPrivate
  {
    bool hasPID;
    double offset;
    double multiplier;
    double maxEffort;
    double sensitiveness;

    Entity jointEntity{kNullEntity};
    Entity mimicJointEntity{kNullEntity};    
  };

  GazeboMimicJointPlugin::GazeboMimicJointPlugin()
    : dataPtr(std::make_unique<GazeboMimicJointPluginPrivate>())
  {
  }

  GazeboMimicJointPlugin::~GazeboMimicJointPlugin()
  {
  }

  void GazeboMimicJointPlugin::Configure(const Entity &_entity,
                                         const std::shared_ptr<const sdf::Element> &_sdf,
                                         EntityComponentManager &_ecm,
                                         EventManager &/*_eventMgr*/)
  {
    std::string jointName, mimicJointName;
    if (_sdf->HasElement("joint"))
      jointName = _sdf->Get<std::string>("joint");
    else
    {
      // gzerr << "MimicJointSystem: Missing required <joint> parameter\n";
      return;
    }

    if (_sdf->HasElement("mimicJoint"))
      mimicJointName = _sdf->Get<std::string>("mimicJoint");
    else
    {
      // gzerr << "MimicJointSystem: Missing required <mimicJoint> parameter\n";
      return;
    }

    if (_sdf->HasElement("multiplier"))
      this->dataPtr->multiplier = _sdf->Get<double>("multiplier");

    if (_sdf->HasElement("offset"))
      this->dataPtr->offset = _sdf->Get<double>("offset");

    if (_sdf->HasElement("sensitiveness"))
      this->dataPtr->sensitiveness = _sdf->Get<double>("sensitiveness");

    if (_sdf->HasElement("maxEffort"))
      this->dataPtr->maxEffort = _sdf->Get<double>("maxEffort");

    if (_sdf->HasElement("hasPID"))
    {
      this->dataPtr->hasPID = true;
      // // If <hasPID/> exists with no value, treat as true
      // if (_sdf->GetElement("hasPID")->GetValue())
      //   this->dataPtr->hasPID = _sdf->Get<bool>("hasPID");
      // else
      //   this->dataPtr->hasPID = true;
    }

    gz::sim::Model model(_entity);
    this->dataPtr->jointEntity = model.JointByName(_ecm, jointName);
    this->dataPtr->mimicJointEntity = model.JointByName(_ecm, mimicJointName);

    auto joint = Joint(this->dataPtr->jointEntity);
    joint.EnablePositionCheck(_ecm, true);
    auto mimic_joint = Joint(this->dataPtr->mimicJointEntity);
    mimic_joint.EnablePositionCheck(_ecm, true);
  }

  // Implement PreUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  void GazeboMimicJointPlugin::PostUpdate(const UpdateInfo &/*_info*/, 
                                          const EntityComponentManager &_ecm)
  {
    if (this->dataPtr->jointEntity == gz::sim::kNullEntity || this->dataPtr->mimicJointEntity == gz::sim::kNullEntity)
      return;

    auto jointPosComp = _ecm.Component<components::JointPosition>(this->dataPtr->jointEntity);
    auto mimicPosComp = _ecm.Component<components::JointPosition>(this->dataPtr->mimicJointEntity);

    if (!jointPosComp || jointPosComp->Data().empty() ||
        !mimicPosComp || mimicPosComp->Data().empty())
      return;

    double jointPos = jointPosComp->Data()[0];
    double mimicPos = mimicPosComp->Data()[0];

    double targetPos = this->dataPtr->multiplier * jointPos + this->dataPtr->offset;

    // Sensitiveness threshold
    if (std::abs(targetPos - mimicPos) <= this->dataPtr->sensitiveness)
      return;

    // ⚠️ 关键修复：const_cast 允许写入
    auto &ecm = const_cast<EntityComponentManager &>(_ecm);

    if (this->dataPtr->hasPID)
    {
      // In Ignition/Gz Sim, there is no built-in PID in the system.
      // You would need to implement one or use JointForceCmd with P control.
      // For now, we use simple P control as fallback.
      double error = targetPos - mimicPos;
      double effort = 80.0 * error;  // Tune gain as needed
      effort = std::clamp(effort, -this->dataPtr->maxEffort, this->dataPtr->maxEffort);
      ecm.SetComponentData<components::JointForceCmd>(this->dataPtr->mimicJointEntity, {effort});
    }
    else
    {
      // Gz Sim does NOT support direct position setting in physics simulation.
      // So even without PID, we must use force/torque.
      // Therefore, we treat both cases similarly.
      double error = targetPos - mimicPos;
      double effort = 50.0 * error;  // Lower gain if no PID
      effort = std::clamp(effort, -this->dataPtr->maxEffort, this->dataPtr->maxEffort);
      ecm.SetComponentData<components::JointForceCmd>(this->dataPtr->mimicJointEntity, {effort});
    }
  }

  // Register plugin
  GZ_ADD_PLUGIN(GazeboMimicJointPlugin,
                GazeboMimicJointPlugin::System,
                // gz::sim::System,
                GazeboMimicJointPlugin::ISystemConfigure,
                GazeboMimicJointPlugin::ISystemPostUpdate)
  // Add plugin alias so that we can refer to the plugin without the version
  // namespace
  GZ_ADD_PLUGIN_ALIAS(GazeboMimicJointPlugin, "gz::sim::systems::GazeboMimicJointPlugin")
}  // namespace gazebo
