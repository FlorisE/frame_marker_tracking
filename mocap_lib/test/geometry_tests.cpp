#include "../include/mocap_lib/marker_lib.h"
#include "../include/mocap_lib/geometry_lib.h"
#include <gtest/gtest.h>
#include <urdf/model.h>
#include <cmath>
#include <iostream>
#include <utility>

using namespace std;
using namespace geometry_lib;

const double ABS_ERROR = 0.001; // mm precision if using metric units

TEST(TestSuite, pointPlaneDistance1)
{
  Plane plane(1, 0, 0, 0);
  Point3D circle_center(2, 2, 1);
  double d = distance(circle_center, plane);
  ASSERT_EQ(d, 2);
}

TEST(TestSuite, pointPlaneDistance2)
{
  Plane plane(0, 1, 0, 0);
  Point3D circle_center(2, 2, 1);
  double d = distance(circle_center, plane);
  ASSERT_EQ(d, 2);
}

TEST(TestSuite, pointPlaneDistance3)
{
  Plane plane(0, 0, 1, 0);
  Point3D circle_center(2, 2, 1);
  double d = distance(circle_center, plane);
  ASSERT_EQ(d, 1);
}

TEST(TestSuite, pointPlaneDistance4)
{
  Plane plane(1/sqrt(2), 1/sqrt(2), 0, 0);
  Point3D circle_center(2, 2, 1);
  double d = distance(circle_center, plane);
  ASSERT_NEAR(d, sqrt(8), ABS_ERROR);
}

TEST(TestSuite, overlappingSphereWithPlane)
{
  Plane plane(0, 0, 1, 0);
  Point3D circle_center(2, 2, 1);
  Sphere sphere(circle_center, 2.0f);
  optional<Circle3D> circle = intersect(plane, sphere);
  ASSERT_TRUE(circle.has_value());
  ASSERT_NEAR(circle.value().radius, sqrt(3), ABS_ERROR);
  ASSERT_NEAR(circle.value().center.x, 2, ABS_ERROR);
  ASSERT_NEAR(circle.value().center.y, 2, ABS_ERROR);
  ASSERT_NEAR(circle.value().center.z, 0, ABS_ERROR);
}

TEST(TestSuite, overlappingCircles)
{
  Circle a(3, 3, 2);
  Circle b(5, 5, 2);
  any intersection = intersect(a, b);
  ASSERT_TRUE(intersection.has_value());
  ASSERT_TRUE(intersection.type() == typeid(pair<Point2D, Point2D>));
  auto& intersection_points = any_cast<pair<Point2D, Point2D>&>(intersection);
  ASSERT_NEAR(intersection_points.first.x, 5, ABS_ERROR);
  ASSERT_NEAR(intersection_points.first.y, 3, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.x, 3, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.y, 5, ABS_ERROR);
}

TEST(TestSuite, overlappingCirclesDifferentRadii)
{
  Circle a(3, 3, 2);
  Circle b(5, 5, 3);
  any intersection = intersect(a, b);
  ASSERT_TRUE(intersection.has_value());
  ASSERT_TRUE(intersection.type() == typeid(pair<Point2D, Point2D>));
  auto& intersection_points = any_cast<pair<Point2D, Point2D>&>(intersection);
  ASSERT_NEAR(intersection_points.first.x, 4.739, ABS_ERROR);
  ASSERT_NEAR(intersection_points.first.y, 2.011, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.x, 2.011, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.y, 4.739, ABS_ERROR);
}

TEST(TestSuite, overlappingCirclesReversed)
{
  Circle a(5, 5, 2);
  Circle b(3, 3, 2);
  any intersection = intersect(a, b);
  ASSERT_TRUE(intersection.has_value());
  ASSERT_TRUE(intersection.type() == typeid(pair<Point2D, Point2D>));
  auto& intersection_points = any_cast<pair<Point2D, Point2D>&>(intersection);
  ASSERT_NEAR(intersection_points.first.x, 3, ABS_ERROR);
  ASSERT_NEAR(intersection_points.first.y, 5, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.x, 5, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.y, 3, ABS_ERROR);
}

TEST(TestSuite, overlappingCirclesNegative)
{
  Circle a(-3, -3, 2);
  Circle b(-5, -5, 2);
  any intersection = intersect(a, b);
  ASSERT_TRUE(intersection.has_value());
  ASSERT_TRUE(intersection.type() == typeid(pair<Point2D, Point2D>));
  auto& intersection_points = any_cast<pair<Point2D, Point2D>&>(intersection);
  ASSERT_NEAR(intersection_points.first.x, -5, ABS_ERROR);
  ASSERT_NEAR(intersection_points.first.y, -3, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.x, -3, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.y, -5, ABS_ERROR);
}

TEST(TestSuite, overlappingCirclesReversedNegative)
{
  Circle a(-5, -5, 2);
  Circle b(-3, -3, 2);
  any intersection = intersect(a, b);
  ASSERT_TRUE(intersection.has_value());
  ASSERT_TRUE(intersection.type() == typeid(pair<Point2D, Point2D>));
  auto& intersection_points = any_cast<pair<Point2D, Point2D>&>(intersection);
  ASSERT_NEAR(intersection_points.first.x, -3, ABS_ERROR);
  ASSERT_NEAR(intersection_points.first.y, -5, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.x, -5, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.y, -3, ABS_ERROR);
}

TEST(TestSuite, overlappingCirclesNegativePositive)
{
  Circle a(1, 1, 2);
  Circle b(-1, -1, 2);
  any intersection = intersect(a, b);
  ASSERT_TRUE(intersection.has_value());
  ASSERT_TRUE(intersection.type() == typeid(pair<Point2D, Point2D>));
  auto& intersection_points = any_cast<pair<Point2D, Point2D>&>(intersection);
  ASSERT_NEAR(intersection_points.first.x, -1, ABS_ERROR);
  ASSERT_NEAR(intersection_points.first.y, 1, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.x, 1, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.y, -1, ABS_ERROR);
}

TEST(TestSuite, overlappingCirclesReversedNegativePositive)
{
  Circle a(-1, -1, 2);
  Circle b(1, 1, 2);
  any intersection = intersect(a, b);
  ASSERT_TRUE(intersection.has_value());
  ASSERT_TRUE(intersection.type() == typeid(pair<Point2D, Point2D>));
  auto& intersection_points = any_cast<pair<Point2D, Point2D>&>(intersection);
  ASSERT_NEAR(intersection_points.first.x, 1, ABS_ERROR);
  ASSERT_NEAR(intersection_points.first.y, -1, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.x, -1, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.y, 1, ABS_ERROR);
}

TEST(TestSuite, touchingCircles)
{
  Circle a(2, 2, 2);
  Circle b(2, 6, 2);
  any intersection = intersect(a, b);
  ASSERT_TRUE(intersection.has_value());
  ASSERT_TRUE(intersection.type() == typeid(pair<Point2D, Point2D>));
  auto& intersection_points = any_cast<pair<Point2D, Point2D>&>(intersection);
  ASSERT_NEAR(intersection_points.first.x, 2, ABS_ERROR);
  ASSERT_NEAR(intersection_points.first.y, 4, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.x, 2, ABS_ERROR);
  ASSERT_NEAR(intersection_points.second.y, 4, ABS_ERROR);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}
