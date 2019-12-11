#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Transformer
{
public:
  Transformer(ros::NodeHandle nh) : nh_(nh), listener_(buffer_), rate_(1000) {}

  void spin()
  {
    while (nh_.ok())
    {
      world_to_camera_ = buffer_.lookupTransform("camera", "world", ros::Time(0), ros::Duration(1.0));
      geometry_msgs::TransformStamped transform;
      transform.header.frame_id = "depth_camera_levelled";
      transform.header.stamp = ros::Time::now();
      transform.child_frame_id = "world";
      transform.transform.translation.x = world_to_camera_.transform.translation.y;
      transform.transform.translation.y = -world_to_camera_.transform.translation.x;
      transform.transform.translation.z = -world_to_camera_.transform.translation.z;

      double roll, pitch, yaw;
      tf2::transformToKDL(world_to_camera_).M.GetRPY(roll, pitch, yaw);
      tf2::Quaternion quat;
      quat.setRPY(0, 0, 0);
      transform.transform.rotation = tf2::toMsg(quat);

      br_.sendTransform(transform);
      rate_.sleep();
    }
  }
private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  geometry_msgs::TransformStamped world_to_camera_;
  tf2_ros::TransformBroadcaster br_;
  ros::Rate rate_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "qualisys_to_depth_sensor_transformer");
  ros::NodeHandle nh("~");
  Transformer transformer(nh);
  transformer.spin();
}
