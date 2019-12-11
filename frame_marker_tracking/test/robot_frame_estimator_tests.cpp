#include "../src/robot_frame_estimator.cpp"
//#include "marker_lib.h"
#include <gtest/gtest.h>
#include <urdf/model.h>
#include <cmath>
#include <iostream>

const double ABS_ERROR = 0.001; // mm precision if using metric units

TEST(TestSuite, testCase1)
{
  ros::NodeHandle nh;
  urdf::Model robot_model;
  multimap<string, shared_ptr<marker_lib::Marker>> frames_and_markers;
  RobotFrameEstimator estimator(nh, robot_model, frames_and_markers);
}

TEST(TestSuite, crossProduct)
{
  urdf::Vector3 left(0, 1, 0);
  urdf::Vector3 right(0, 0, 1);
  urdf::Vector3 result = cross(left, right);
  ASSERT_EQ(result.x, 1);
  ASSERT_EQ(result.y, 0);
  ASSERT_EQ(result.z, 0);
}

TEST(TestSuite, vectorLength)
{
  urdf::Vector3 v(1, 1, 1);
  ASSERT_NEAR(norm(v), sqrt(3), ABS_ERROR);
}

TEST(TestSuite, vectorDot)
{
  urdf::Vector3 a(1, 2, 3);
  urdf::Vector3 b(3, 2, 1);
  ASSERT_EQ(dot(a, b), 10);
}

TEST(TestSuite, muliplyRotationByScalarAssign)
{
  urdf::Rotation rot(0, 0, 0, 1);
  rot *= 2;
  ASSERT_EQ(rot.w, 2);
}

TEST(TestSuite, muliplyScalarByRotationAssign)
{
  double d = 2;
  urdf::Rotation rot(0, 0, 0, 1);

  d *= rot;

  ASSERT_EQ(rot.w, 2);
}

double rad(double degree)
{
  return degree * 3.1415926 / 180;
}

const double RAD_ERROR = 0.1;

TEST(TestSuite, slerp)
{
  urdf::Rotation rot1;
  urdf::Rotation rot2;
  rot1.setFromRPY(rad(30), rad(30), rad(30));
  rot2.setFromRPY(rad(32), rad(32), rad(32));
  urdf::Rotation interpolated = slerp(rot1, rot2, 0.5);
  double r, p, y;
  interpolated.getRPY(r, p, y);
  ASSERT_NEAR(r, rad(31), RAD_ERROR);
  ASSERT_NEAR(p, rad(31), RAD_ERROR);
  ASSERT_NEAR(y, rad(31), RAD_ERROR);
}

TEST(TestSuite, blendRotation)
{
  urdf::Rotation rot1;
  urdf::Rotation rot2;
  urdf::Rotation rot3;
  rot1.setFromRPY(rad(30), rad(30), rad(30));
  rot2.setFromRPY(rad(31), rad(31), rad(31));
  rot3.setFromRPY(rad(32), rad(32), rad(32));
  vector<shared_ptr<urdf::Rotation>> rotations;
  rotations.push_back(make_shared<urdf::Rotation>(rot1));
  rotations.push_back(make_shared<urdf::Rotation>(rot2));
  rotations.push_back(make_shared<urdf::Rotation>(rot3));
  urdf::Rotation blended = blend(rotations);
  double r, p, y;
  blended.getRPY(r, p, y);
  ASSERT_NEAR(r, rad(31), RAD_ERROR);
  ASSERT_NEAR(p, rad(31), RAD_ERROR);
  ASSERT_NEAR(y, rad(31), RAD_ERROR);
}

TEST(TestSuite, getJointPosition)
{
  urdf::Vector3 parent_joint_position(0, 0, 0);
  std::string parent_joint_name = "parent_joint";

  urdf::Joint joint;
  urdf::Vector3 joint_link(2, 0, 0);
  urdf::Rotation joint_rotation;
  joint_rotation.setFromRPY(0, -rad(90), 0);
  std::string joint_name = "joint";
  std::string joint_link_name = "link";
  joint.name = joint_name;
  joint.type = urdf::Joint::REVOLUTE;
  joint.child_link_name = joint_link_name;
  joint.parent_to_joint_origin_transform.position = joint_link;
  joint.parent_to_joint_origin_transform.rotation = joint_rotation;
  urdf::JointSharedPtr joint_ptr = make_shared<urdf::Joint>(joint);

  map<string, urdf::Vector3> known_joint_positions;
  known_joint_positions.insert(make_pair(parent_joint_name, parent_joint_position));
  auto position = known_joint_positions.find(joint_name);

  get_joint_position(
    parent_joint_position, joint_rotation, joint_ptr, known_joint_positions, position
  );

  ASSERT_NEAR(position->second.x, 0, ABS_ERROR);
  ASSERT_NEAR(position->second.y, 0, ABS_ERROR);
  ASSERT_NEAR(position->second.z, 2, ABS_ERROR);
}

// compare __ with _|
TEST(TestSuite, getJointRotation)
{
  multimap<string, shared_ptr<marker_lib::Marker>> mocap_markers;
  marker_lib::Marker mocap1(1, 2, 0, 1);
  marker_lib::Marker mocap2(2, 2, 0, 2);
  marker_lib::Marker mocap3(3, 2, 0, 3);
  mocap_markers.insert(make_pair("test", make_shared<marker_lib::Marker>(mocap1)));
  mocap_markers.insert(make_pair("test", make_shared<marker_lib::Marker>(mocap2)));
  mocap_markers.insert(make_pair("test", make_shared<marker_lib::Marker>(mocap3)));

  multimap<string, shared_ptr<marker_lib::Marker>> stored_markers;
  marker_lib::Marker stored1(1, 1, 0, 0);
  marker_lib::Marker stored2(2, 2, 0, 0);
  marker_lib::Marker stored3(3, 3, 0, 0);
  stored_markers.insert(make_pair("test", make_shared<marker_lib::Marker>(stored1)));
  stored_markers.insert(make_pair("test", make_shared<marker_lib::Marker>(stored2)));
  stored_markers.insert(make_pair("test", make_shared<marker_lib::Marker>(stored3)));

  urdf::Rotation parent_joint_rotation;
  parent_joint_rotation.setFromRPY(0, 0, 0);
  urdf::Vector3 position(2, 0, 0);
  urdf::Joint joint;
  joint.child_link_name = "test";
  joint.type = urdf::Joint::REVOLUTE;
  urdf::JointSharedPtr joint_ptr = make_shared<urdf::Joint>(joint);
  map<string, urdf::Rotation> res;

  bool status = get_joint_rotation(mocap_markers, stored_markers, parent_joint_rotation, position, joint_ptr, res);
  ASSERT_TRUE(status);
  ASSERT_EQ(res.size(), 1);
  auto pair = res.find("test");
  urdf::Rotation res_rot = pair->second;
  double r, p, y;
  res_rot.getRPY(r, p, y);
  ASSERT_NEAR(r, 0, RAD_ERROR);
  ASSERT_NEAR(p, rad(90), RAD_ERROR);
  ASSERT_NEAR(y, 0, RAD_ERROR);
}

TEST(TestSuite, updateSkippedLink)
{
  // three link mechanism, with second and third link bend 45 degrees
  //const map<string, urdf::JointSharedPtr> joint_for_link;
  multimap<string, shared_ptr<marker_lib::Marker>> mocap_markers;
  multimap<string, shared_ptr<marker_lib::Marker>> stored_markers;

  // let each joint be 2 cm long
  urdf::Joint j0;
  j0.name = "joint0";
  j0.child_link_name = "joint0";
  urdf::Vector3 j0_pos(2, 0, 0);
  urdf::Vector3 j0_abs_pos(0, 0, 0);
  urdf::Rotation j0_rot;
  j0_rot.setFromRPY(0, 0, 0);
  j0.type = urdf::Joint::REVOLUTE;
  j0.axis.y = 1;
  j0.parent_to_joint_origin_transform.position = j0_pos;
  urdf::JointSharedPtr j0_ptr = make_shared<urdf::Joint>(j0);
  urdf::Joint j1;
  j1.name = "joint1";
  j1.child_link_name = "joint1";
  urdf::Vector3 j1_pos(2, 0, 0);
  urdf::Vector3 j1_abs_pos(2, 0, 0);
  j1.type = urdf::Joint::REVOLUTE;
  j1.axis.y = 1;
  j1.parent_to_joint_origin_transform.position = j1_pos;
  urdf::JointSharedPtr j1_ptr = make_shared<urdf::Joint>(j1);
  urdf::Joint j2;
  j2.name = "joint2";
  j2.child_link_name = "joint2";
  urdf::Vector3 j2_pos(2, 0, 0);
  urdf::Vector3 j2_abs_pos(2+sqrt(2), 0, sqrt(2));
  j2.type = urdf::Joint::REVOLUTE;
  j2.axis.y = 1;
  j2.parent_to_joint_origin_transform.position = j2_pos;
  urdf::JointSharedPtr j2_ptr = make_shared<urdf::Joint>(j2);

  marker_lib::Marker m1(1, 2+sqrt(2), 0, sqrt(2)+0.5);
  marker_lib::Marker m2(2, 2+sqrt(2), 0, sqrt(2)+1.5);
  mocap_markers.insert(make_pair(j2.name, make_shared<marker_lib::Marker>(m1)));
  mocap_markers.insert(make_pair(j2.name, make_shared<marker_lib::Marker>(m2)));

  marker_lib::Marker sm1(1, 0, 0, 0.5);
  marker_lib::Marker sm2(2, 0, 0, 1.5);
  stored_markers.insert(make_pair(j2.name, make_shared<marker_lib::Marker>(sm1)));
  stored_markers.insert(make_pair(j2.name, make_shared<marker_lib::Marker>(sm2)));

  map<string, urdf::Vector3> known_joint_positions;
  known_joint_positions.insert(make_pair(j0.name, j0_abs_pos));

  map<string, urdf::Rotation> known_joint_rotations;
  known_joint_rotations.insert(make_pair(j0.name, j0_rot));

  update_skipped_link(
    /*joint_for_link,*/ mocap_markers, stored_markers, j0_ptr, j1_ptr, j2_ptr,
    known_joint_positions, known_joint_rotations
  );

  auto j1_determined_pos_it = known_joint_positions.find(j1.child_link_name);
  ASSERT_TRUE(j1_determined_pos_it != known_joint_positions.end());

  auto j1_determined_pos = j1_determined_pos_it->second;
  ASSERT_NEAR(j1_determined_pos.y, 0, ABS_ERROR);
  ASSERT_NEAR(j1_determined_pos.z, 0, ABS_ERROR);
  ASSERT_NEAR(j1_determined_pos.x, 2, ABS_ERROR);

  auto j1_determined_rot_it = known_joint_rotations.find(j1.child_link_name);
  ASSERT_TRUE(j1_determined_rot_it != known_joint_rotations.end());

  auto j1_determined_rot = j1_determined_rot_it->second;
  double roll, pitch, yaw;
  j1_determined_rot.getRPY(roll, pitch, yaw);
  ASSERT_NEAR(roll, 0, RAD_ERROR);
  ASSERT_NEAR(pitch, -rad(45), RAD_ERROR);
  ASSERT_NEAR(yaw, 0, RAD_ERROR);

  auto j2_determined_pos_it = known_joint_positions.find(j2.child_link_name);
  ASSERT_TRUE(j2_determined_pos_it != known_joint_positions.end());

  auto j2_determined_pos = j2_determined_pos_it->second;
  ASSERT_NEAR(j2_determined_pos.x, 2+sqrt(2), ABS_ERROR);
  ASSERT_NEAR(j2_determined_pos.y, 0, ABS_ERROR);
  ASSERT_NEAR(j2_determined_pos.z, sqrt(2), ABS_ERROR);

  auto j2_determined_rot_it = known_joint_rotations.find(j2.child_link_name);
  ASSERT_TRUE(j2_determined_rot_it != known_joint_rotations.end());

  auto j2_determined_rot = j2_determined_rot_it->second;
  j2_determined_rot.getRPY(roll, pitch, yaw);
  ASSERT_NEAR(roll, 0, RAD_ERROR);
  ASSERT_NEAR(pitch, -rad(90), RAD_ERROR);
  ASSERT_NEAR(yaw, 0, RAD_ERROR);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}
