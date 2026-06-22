#include "art_planner/validity_checker/validity_checker.h"

#include <atomic>
#include <iostream>



using namespace art_planner;

namespace {
constexpr int kMaxInvalidStateLogs = 20;
std::atomic<int> g_invalid_state_logs{0};

bool shouldPrintInvalidStateLog() {
  return g_invalid_state_logs.fetch_add(1, std::memory_order_relaxed) < kMaxInvalidStateLogs;
}
}  // namespace



StateValidityChecker::StateValidityChecker(const ob::SpaceInformationPtr &si,
                                           const ParamsConstPtr& params) :
     ob::StateValidityChecker(si),
     checker_feet_(params),
     checker_body_(params),
     params_(params) {
}



void StateValidityChecker::setMap(const std::shared_ptr<Map> &map) {
  checker_feet_.setMap(map);
  checker_body_.setMap(map);
}



void StateValidityChecker::updateHeightField() {
  checker_feet_.updateHeightField();
  checker_body_.updateHeightField();
}



bool StateValidityChecker::hasMap() const {
  return checker_feet_.hasMap() && checker_body_.hasMap();
}



bool StateValidityChecker::isValid(const ob::State *state) const {
  const auto state_pose = Pose3FromSE3(state);
  const auto pose_base = state_pose*Pose3FromXYZ(params_->robot.torso.offset.x,
                                                 params_->robot.torso.offset.y,
                                                 params_->robot.torso.offset.z - params_->robot.feet.offset.z);
  const bool body_valid = checker_body_.isValid(pose_base);
  if (!body_valid) {
    if (shouldPrintInvalidStateLog()) {
      std::cerr << "[art_planner诊断] 状态无效：机身碰撞检查失败。"
                << " feet中心=(" << state_pose.translation().x() << ", "
                << state_pose.translation().y() << ", "
                << state_pose.translation().z() << "), 机身中心=("
                << pose_base.translation().x() << ", "
                << pose_base.translation().y() << ", "
                << pose_base.translation().z() << "), torso.offset.z="
                << params_->robot.torso.offset.z << ", feet.offset.z="
                << params_->robot.feet.offset.z << std::endl;
    }
    return false;
  }

  const bool feet_valid = checker_feet_.isValid(state_pose);
  if (!feet_valid && shouldPrintInvalidStateLog()) {
    std::cerr << "[art_planner诊断] 状态无效：四个脚/支撑框检查失败。"
              << " feet中心=(" << state_pose.translation().x() << ", "
              << state_pose.translation().y() << ", "
              << state_pose.translation().z() << "), feet.offset=("
              << params_->robot.feet.offset.x << ", "
              << params_->robot.feet.offset.y << ", "
              << params_->robot.feet.offset.z << "), feet.reach=("
              << params_->robot.feet.reach.x << ", "
              << params_->robot.feet.reach.y << ", "
              << params_->robot.feet.reach.z << ")" << std::endl;
  }
  return feet_valid;
}
