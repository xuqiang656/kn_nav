#include "art_planner/validity_checker/validity_checker_body.h"

#include <atomic>
#include <iostream>



using namespace art_planner;

namespace {
constexpr int kMaxBodyInvalidLogs = 20;
std::atomic<int> g_body_invalid_logs{0};

bool shouldPrintBodyInvalidLog() {
  return g_body_invalid_logs.fetch_add(1, std::memory_order_relaxed) < kMaxBodyInvalidLogs;
}
}  // namespace



ValidityCheckerBody::ValidityCheckerBody(const ParamsConstPtr& params) : params_(params) {
  checker_.reset(new HeightMapBoxChecker(params_->robot.torso.length,
                                         params_->robot.torso.width,
                                         params_->robot.torso.height));
}



void ValidityCheckerBody::setMap(const MapPtr& map) {
  elevation_map_ = map;
}



static HeightMapBoxChecker::dPose d_pose;



bool ValidityCheckerBody::isValid(const Pose3& pose) const {
  // Return no collision if outside of map bounds.
  if (!elevation_map_->isInside(grid_map::Position(pose.translation().x(),
                                                   pose.translation().y()))) {
    return true;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  d_pose.origin[0] = pose.translation().x();
  d_pose.origin[1] = pose.translation().y();
  d_pose.origin[2] = pose.translation().z();
  Eigen::Map<Eigen::Matrix<Scalar, 3, 4,Eigen::RowMajor> > rot(d_pose.rotation.data());
  rot.topLeftCorner(3,3) = pose.matrix().topLeftCorner(3,3);
  const bool in_collision = static_cast<bool>(checker_->checkCollision({d_pose}));
  if (in_collision && shouldPrintBodyInvalidLog()) {
    const grid_map::Position pos(pose.translation().x(), pose.translation().y());
    double elevation = std::numeric_limits<double>::quiet_NaN();
    try {
      elevation = elevation_map_->getHeightAtPosition(pos);
    } catch (const std::exception&) {
    }

    std::cerr << "[art_planner诊断] 机身无效：机身包围盒与 elevation 层碰撞。"
              << " 机身中心=(" << pose.translation().x() << ", "
              << pose.translation().y() << ", " << pose.translation().z()
              << "), 机身底部约=" << pose.translation().z() - params_->robot.torso.height * 0.5
              << ", 地形高程约=" << elevation
              << ", torso.height=" << params_->robot.torso.height
              << "。若机身底部低于/接近地面，请增大 robot.torso.offset.z，"
              << "或重新校正 base_link.z 与 robot.feet.offset.z。" << std::endl;
  }
  return !in_collision;
}



bool ValidityCheckerBody::hasMap() const {
  return static_cast<bool>(elevation_map_);
}



void ValidityCheckerBody::updateHeightField() {
  std::lock_guard<std::mutex> lock(mutex_);
  checker_->setHeightField(elevation_map_, params_->planner.elevation_layer);
}
