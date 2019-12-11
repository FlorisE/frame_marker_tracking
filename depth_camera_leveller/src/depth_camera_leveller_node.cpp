#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Leveller
{
public:
  Leveller(ros::NodeHandle nh) : nh_(nh), listener_(buffer_), rate_(1000) {}

  void spin()
  {
    while (nh_.ok())
    {
      mocap_to_camera_ = buffer_.lookupTransform("depth_camera", "mocap", ros::Time(0), ros::Duration(1.0));
      geometry_msgs::TransformStamped transform;
      transform.header.frame_id = "depth_camera_left";
      transform.header.stamp = ros::Time::now();
      transform.child_frame_id = "depth_camera_levelled";

      double roll, pitch, yaw;
      tf2::transformToKDL(mocap_to_camera_).M.GetRPY(roll, pitch, yaw);
      tf2::Quaternion q;
      q.setRPY(0, -pitch, 0);
      transform.transform.rotation = tf2::toMsg(q);

      br_.sendTransform(transform);
      rate_.sleep();
    }
  }
private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  geometry_msgs::TransformStamped mocap_to_camera_;
  tf2_ros::TransformBroadcaster br_;
  ros::Rate rate_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_camera_leveller");
  ros::NodeHandle nh("~");
  Leveller leveller(nh);
  leveller.spin();
}
