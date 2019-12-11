#ifndef ROBOT_FRAME_ESTIMATOR_NODE_H
#define ROBOT_FRAME_ESTIMATOR_NODE_H

#include <ros/ros.h>
#include <mocap_lib/marker_lib.h>
#include <qualisys_msgs/Markers.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>
#include <string>

class RobotFrameEstimator
{
public:
  RobotFrameEstimator(
    ros::NodeHandle& nh, const urdf::Model& robot_model, 
    const std::multimap<std::string, std::shared_ptr<marker_lib::Marker>>& markers_and_frames
  );
  void markers_callback(const qualisys_msgs::Markers::ConstPtr& markers);
private:
  ros::NodeHandle nh_;
  std::string marker_path_;
  std::string markers_topic_;
  std::string urdf_path_;
  std::string source_frame_;
  std::string global_frame_;
  urdf::Model robot_model_;
  ros::Subscriber markers_sub_;
  std::multimap<std::string, std::shared_ptr<marker_lib::Marker>> stored_markers_;
  std::vector<std::shared_ptr<marker_lib::Marker>> marker_list_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped global_frame_to_source_frame_;
};

#endif
