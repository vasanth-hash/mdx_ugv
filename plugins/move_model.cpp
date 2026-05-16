#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/sdf.hh>
#include <vector>

namespace mdx_ugv
{
  class MoveModel : public ignition::gazebo::v6::System,
                    public ignition::gazebo::v6::ISystemConfigure,
                    public ignition::gazebo::v6::ISystemUpdate
  {
  public:
    struct Waypoint {
      ignition::math::v6::Pose3d pose;
      double time;
    };

    void Configure(const ignition::gazebo::v6::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::v6::EntityComponentManager &/*_ecm*/,
                   ignition::gazebo::v6::EventManager &) override
    {
      this->modelEntity = _entity;

      this->loop = _sdf->Get<bool>("loop", true).first;

      if (_sdf->HasElement("waypoint")) {
        auto elem = _sdf->FindElement("waypoint");
        while (elem) {
          Waypoint wp;
          if (elem->HasElement("pose")) {
            wp.pose = elem->Get<ignition::math::v6::Pose3d>("pose");
          } else {
            ignerr << "Waypoint missing <pose> element. Skipping.\n";
            elem = elem->GetNextElement("waypoint");
            continue;
          }
          if (elem->HasElement("time")) {
            wp.time = elem->Get<double>("time");
          } else {
            ignerr << "Waypoint missing <time> element. Skipping.\n";
            elem = elem->GetNextElement("waypoint");
            continue;
          }
          waypoints.push_back(wp);
          elem = elem->GetNextElement("waypoint");
        }
      }

      if (!waypoints.empty()) {
        loopDuration = waypoints.back().time;
      }

      // ignmsg << "MoveModel configured for entity [" << modelEntity << "] with " << waypoints.size() << " waypoints. Loop: " << (loop ? "true" : "false") << "\n";
    }

    void Update(const ignition::gazebo::v6::UpdateInfo &_info,
                ignition::gazebo::v6::EntityComponentManager &_ecm) override
    {
      if (this->modelEntity == ignition::gazebo::v6::kNullEntity || waypoints.empty())
        return;

      double simTime = std::chrono::duration<double>(_info.simTime).count();
      double adjustedTime = simTime;
      if (loop && loopDuration > 0) {
        adjustedTime = fmod(simTime, loopDuration);
      }

      Waypoint currentWp = waypoints[0];
      Waypoint nextWp = waypoints[0];
      size_t currentIdx = 0;
      size_t nextIdx = 0;

      if (adjustedTime >= loopDuration && loop) {
        adjustedTime = loopDuration; // Clamp to last waypoint for smooth transition
      }

      for (size_t i = 0; i < waypoints.size(); ++i) {
        if (adjustedTime >= waypoints[i].time) {
          currentWp = waypoints[i];
          currentIdx = i;
          if (i + 1 < waypoints.size()) {
            nextWp = waypoints[i + 1];
            nextIdx = i + 1;
          } else if (loop) {
            nextWp = waypoints[0];
            nextIdx = 0;
          } else {
            nextWp = waypoints[i];
            nextIdx = i;
          }
        } else {
          break;
        }
      }

      ignition::math::v6::Pose3d targetPose = currentWp.pose;
      if (nextIdx != currentIdx && adjustedTime < nextWp.time) {
        double t = (adjustedTime - currentWp.time) / (nextWp.time - currentWp.time);
        targetPose.Pos() = currentWp.pose.Pos() + t * (nextWp.pose.Pos() - currentWp.pose.Pos());
        targetPose.Rot() = ignition::math::v6::Quaterniond::Slerp(t, currentWp.pose.Rot(), nextWp.pose.Rot());
      } else if (nextIdx == 0 && loop && adjustedTime >= loopDuration) {
        targetPose = currentWp.pose;
        if (!loggedLoop) {
          ignmsg << "MoveModel: Looping back to first waypoint at t=" << simTime << "s\n";
          loggedLoop = true;
        }
      } else {
        loggedLoop = false; 
      }

      auto poseComp = _ecm.Component<ignition::gazebo::v6::components::Pose>(this->modelEntity);
      if (!poseComp) {
        _ecm.CreateComponent(this->modelEntity, ignition::gazebo::v6::components::Pose(targetPose));
      } else {
        poseComp->Data() = targetPose;
      }

      if (static_cast<int>(simTime) % 5 == 0 && lastLogTime != static_cast<int>(simTime)) {
        // ignmsg << "MoveModel: Setting pose to [" << targetPose << "] at t=" << simTime << "s (adjusted: " << adjustedTime << "s)\n";
        lastLogTime = static_cast<int>(simTime);
      }
    }

  private:
    ignition::gazebo::v6::Entity modelEntity{ignition::gazebo::v6::kNullEntity};
    std::vector<Waypoint> waypoints;
    double loopDuration{0.0};
    bool loop{true};
    bool loggedLoop{false}; // Flag to prevent repeated loop logging
    int lastLogTime{-1}; // Track last logged time for pose updates
  };
}

IGNITION_ADD_PLUGIN(
  mdx_ugv::MoveModel,
  ignition::gazebo::v6::System,
  ignition::gazebo::v6::ISystemConfigure,
  ignition::gazebo::v6::ISystemUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(mdx_ugv::MoveModel, "mdx_ugv::MoveModel")