#include "art_planner/validity_checker/validity_checker_feet.h"

#include <atomic>
#include <iostream>
#include <limits>

#include <grid_map_core/iterators/PolygonIterator.hpp>

#include "art_planner/utils.h"



using namespace art_planner;

namespace {
constexpr int kMaxFeetInvalidLogs = 30;
std::atomic<int> g_feet_invalid_logs{0};

bool shouldPrintFeetInvalidLog() {
  return g_feet_invalid_logs.fetch_add(1, std::memory_order_relaxed) < kMaxFeetInvalidLogs;
}

void printFootPose(const char* reason,
                   const Pose3& pose,
                   const ParamsConstPtr& params,
                   const std::shared_ptr<Map>& map) {
  const grid_map::Position pos(pose.translation().x(), pose.translation().y());
  double elevation = std::numeric_limits<double>::quiet_NaN();
  double elevation_masked = std::numeric_limits<double>::quiet_NaN();
  try {
    elevation = map->getHeightAtPosition(pos);
    const auto& grid_map = map->getMap();
    if (grid_map.exists("elevation_masked")) {
      elevation_masked = grid_map.atPosition("elevation_masked", pos);
    }
  } catch (const std::exception&) {
  }

  std::cerr << "[art_planner诊断] 支撑框无效：" << reason
            << " 支撑框中心=(" << pose.translation().x() << ", "
            << pose.translation().y() << ", " << pose.translation().z()
            << "), 框底部约=" << pose.translation().z() - params->robot.feet.reach.z * 0.5
            << ", 框顶部约=" << pose.translation().z() + params->robot.feet.reach.z * 0.5
            << ", elevation约=" << elevation
            << ", elevation_masked约=" << elevation_masked
            << ", feet.reach.z=" << params->robot.feet.reach.z
            << "。如果 elevation_masked 是 -inf 或非常低，说明该处被认为不可通行/未知；"
            << "如果地形高程不在框底部和顶部之间，需要调整 robot.feet.offset.z 或 TF 高度。"
            << std::endl;
}
}  // namespace



ValidityCheckerFeet::ValidityCheckerFeet(const ParamsConstPtr& params) : params_(params) {
  box_length_ = params_->robot.feet.reach.x;
  box_width_ = params_->robot.feet.reach.y;

  checker_.reset(new HeightMapBoxChecker(box_length_, box_width_, params_->robot.feet.reach.z));
}



void ValidityCheckerFeet::setMap(const MapPtr &map) {
  traversability_map_ = map;
}



static HeightMapBoxChecker::dPose d_pose;



bool ValidityCheckerFeet::boxIsValidAtPose(const Pose3 &pose) const {
  // Return if outside of map bounds.
  if (!traversability_map_->isInside(grid_map::Position(pose.translation().x(),
                                                        pose.translation().y()))) {
    if (params_->planner.unknown_space_untraversable && shouldPrintFeetInvalidLog()) {
      printFootPose("位置超出地图范围，且 unknown_space_untraversable=true。",
                    pose,
                    params_,
                    traversability_map_);
    }
    return !params_->planner.unknown_space_untraversable;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  d_pose.origin[0] = pose.translation().x();
  d_pose.origin[1] = pose.translation().y();
  d_pose.origin[2] = pose.translation().z();
  Eigen::Map<Eigen::Matrix<dReal, 3, 4,Eigen::RowMajor> > rot(d_pose.rotation.data());
  rot.topLeftCorner(3,3) = pose.matrix().topLeftCorner(3,3);
  const bool has_contact = static_cast<bool>(checker_->checkCollision({d_pose}));
  if (!has_contact && shouldPrintFeetInvalidLog()) {
    printFootPose("支撑框没有和 elevation_masked 层接触。",
                  pose,
                  params_,
                  traversability_map_);
  }
  return has_contact;
}



bool ValidityCheckerFeet::boxesAreValidAtPoses(const std::vector<Pose3> &poses) const {
  bool valid = true;
  for (const auto& pose: poses) {
    valid &= boxIsValidAtPose(pose);
    // Need all four feet to be valid, so we can return, when one is not.
    if (!valid) break;
  }
  return valid;
}



bool ValidityCheckerFeet::isValid(const Pose3& pose) const {
  std::vector<Pose3> foot_poses({
                     pose*Pose3FromXYZ( params_->robot.feet.offset.x,  params_->robot.feet.offset.y, 0.0f),
                     pose*Pose3FromXYZ( params_->robot.feet.offset.x, -params_->robot.feet.offset.y, 0.0f),
                     pose*Pose3FromXYZ(-params_->robot.feet.offset.x,  params_->robot.feet.offset.y, 0.0f),
                     pose*Pose3FromXYZ(-params_->robot.feet.offset.x, -params_->robot.feet.offset.y, 0.0f)});
  return boxesAreValidAtPoses(foot_poses);
}



bool ValidityCheckerFeet::hasMap() const {
  return static_cast<bool>(traversability_map_);
}



void ValidityCheckerFeet::updateHeightField() {
  std::lock_guard<std::mutex> lock(mutex_);
  checker_->setHeightField(traversability_map_, "elevation_masked");
}
