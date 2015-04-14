#include "aav_control/cte_calculator.h"

#include <gtest/gtest.h>

void setupStraightForTenPath(aav_msgs::QuinticPath &path)
{
  aav_msgs::PlanningPoint pp;
  pp.acceleration = 1;
  aav_msgs::QuinticPathSegment segment;
  segment.points.push_back(pp);
  segment.x_segment.P0 = 0.0;
  segment.x_segment.P1 = 1.0;
  segment.x_segment.P2 = 2.0;
  segment.x_segment.P3 = 8.0;
  segment.x_segment.P4 = 9.0;
  segment.x_segment.P5 = 10.0;
  path.segments.push_back(segment);
}

TEST(TestSuite, calculate_zero_cte_when_on_route)
{
  aav_msgs::QuinticPath path;
  setupStraightForTenPath(path);
  aav_control::CteCalculator cte(path);
  tf2::Vector3 position(0, 0, 0);
  EXPECT_EQ(0, cte.calculate(position));
}

TEST(TestSuite, calculate_positive_cte_when_right_of_route)
{
  aav_msgs::QuinticPath path;
  setupStraightForTenPath(path);
  aav_control::CteCalculator cte(path);
  tf2::Vector3 position(0, -1, 0);
  EXPECT_EQ(1, cte.calculate(position));
}

TEST(TestSuite, calculate_negative_cte_when_left_of_route)
{
  aav_msgs::QuinticPath path;
  setupStraightForTenPath(path);
  aav_control::CteCalculator cte(path);
  tf2::Vector3 position(0, 1, 0);
  EXPECT_EQ(-1, cte.calculate(position));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

