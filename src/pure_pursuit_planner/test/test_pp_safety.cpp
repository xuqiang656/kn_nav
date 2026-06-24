#include <gtest/gtest.h>

#include <vector>

#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"

namespace pure_pursuit_planner
{
namespace
{

TEST(PurePursuitSafety, UsesConfiguredGoalThreshold)
{
  PurePursuitConfig config;
  config.goal_threshold = 0.4;
  PurePursuitComponent planner(config);
  planner.setPath({0.0, 1.0}, {0.0, 1.0}, {}, {});
  planner.setPose({0.75, 1.0, 0.0}, 0.0);

  const auto [velocity, yaw_rate] = planner.isGoalReached(0.2, 0.1);
  EXPECT_DOUBLE_EQ(velocity, 0.0);
  EXPECT_DOUBLE_EQ(yaw_rate, 0.0);
}

TEST(PurePursuitSafety, EmptyOrMismatchedPathReturnsZero)
{
  PurePursuitComponent planner(PurePursuitConfig{});
  const Pose2D pose{0.0, 0.0, 0.0};

  EXPECT_EQ(
    planner.computeVelocity({}, {}, {}, {}, pose, 0.0),
    (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(
    planner.computeVelocity({0.0}, {}, {0.0}, {0.0}, pose, 0.0),
    (std::vector<double>{0.0, 0.0}));
}

TEST(PurePursuitSafety, SearchOnEmptyPathReturnsInvalidIndexSafely)
{
  PurePursuitComponent planner(PurePursuitConfig{});
  planner.odom_sub_flag = true;
  const auto [index, lookahead] = planner.searchTargetIndex();

  EXPECT_EQ(index, -1);
  EXPECT_DOUBLE_EQ(lookahead, 0.0);
}

}  // namespace
}  // namespace pure_pursuit_planner
