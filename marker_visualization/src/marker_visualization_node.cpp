#include <ros/ros.h>
#include <qualisys_msgs/Markers.h>
#include <qualisys_msgs/Marker.h>
#include <visualization_msgs/Marker.h>
#include <marker_visualization_srvs/SetMarkerColor.h>
#include <marker_visualization_srvs/ResetMarkerColor.h>
#include <marker_visualization_srvs/SetMarkerLabel.h>
#include <marker_visualization_srvs/ResetMarkerLabel.h>
#include <map>

typedef marker_visualization_srvs::SetMarkerColor::Request SetMarkerColorRequest;
typedef marker_visualization_srvs::SetMarkerColor::Response SetMarkerColorResponse;
typedef marker_visualization_srvs::ResetMarkerColor::Request ResetMarkerColorRequest;
typedef marker_visualization_srvs::ResetMarkerColor::Response ResetMarkerColorResponse;
typedef marker_visualization_srvs::SetMarkerLabel::Request SetMarkerLabelRequest;
typedef marker_visualization_srvs::SetMarkerLabel::Response SetMarkerLabelResponse;
typedef marker_visualization_srvs::ResetMarkerLabel::Request ResetMarkerLabelRequest;
typedef marker_visualization_srvs::ResetMarkerLabel::Response ResetMarkerLabelResponse;

class MarkerVisualizer {
public:
  MarkerVisualizer(const ros::NodeHandle nh) : nh_(nh) {
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    marker_label_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker_label", 10);
    sub_ = nh_.subscribe("markers", 1000, &MarkerVisualizer::visualize_markers, this);
    nh_.param("marker_default_r", default_marker_color_.r, 0.0f);
    nh_.param("marker_default_g", default_marker_color_.g, 1.0f);
    nh_.param("marker_default_b", default_marker_color_.b, 0.0f);
    nh_.param("marker_default_a", default_marker_color_.a, 1.0f);
    nh_.param("marker_frame", frame_, std::string("mocap"));
    nh_.param("namespace", ns_, std::string("mocap_markers"));
    set_marker_color_ = nh_.advertiseService("set_marker_color", &MarkerVisualizer::set_marker_color, this);
    reset_marker_color_ = nh_.advertiseService("reset_marker_color", &MarkerVisualizer::reset_marker_color, this);
    set_marker_label_ = nh_.advertiseService("set_marker_label", &MarkerVisualizer::set_marker_label, this);
    reset_marker_label_ = nh_.advertiseService("reset_marker_label", &MarkerVisualizer::reset_marker_label, this);
  }

  void visualize_markers(const qualisys_msgs::Markers& marker_list)
  {
    uint32_t shape = visualization_msgs::Marker::SPHERE;
  
    for (const qualisys_msgs::Marker& marker : marker_list.markers)
    {
      visualization_msgs::Marker viz_marker_sphere;
      viz_marker_sphere.header.frame_id = frame_;
      viz_marker_sphere.header.stamp = ros::Time::now();
      viz_marker_sphere.ns = ns_;
      viz_marker_sphere.id = marker.index.data;
      viz_marker_sphere.type = shape;
      viz_marker_sphere.action = visualization_msgs::Marker::ADD;
      viz_marker_sphere.pose.position.x = marker.point.x;
      viz_marker_sphere.pose.position.y = marker.point.y;
      viz_marker_sphere.pose.position.z = marker.point.z;
      viz_marker_sphere.pose.orientation.x = 0.0f;
      viz_marker_sphere.pose.orientation.y = 0.0f;
      viz_marker_sphere.pose.orientation.z = 0.0f;
      viz_marker_sphere.pose.orientation.w = 1.0f;
      viz_marker_sphere.scale.x = 0.014f;
      viz_marker_sphere.scale.y = 0.014f;
      viz_marker_sphere.scale.z = 0.014f;
      auto marker_color_it = marker_color_.find(marker.index.data);
      if (marker_color_it != marker_color_.end())
        viz_marker_sphere.color = marker_color_it->second;
      else
        viz_marker_sphere.color = default_marker_color_;
      viz_marker_sphere.lifetime = ros::Duration(0.1);
      marker_pub_.publish(viz_marker_sphere);

      uint32_t label_shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
      auto marker_label_it = marker_label_.find(marker.index.data);
      if (marker_label_it != marker_label_.end())
      {
        visualization_msgs::Marker viz_marker_label;
        viz_marker_label.header.frame_id = frame_;
        viz_marker_label.header.stamp = ros::Time::now();
        viz_marker_label.ns = ns_;
        viz_marker_label.id = marker.index.data;
        viz_marker_label.type = label_shape;
        viz_marker_label.action = visualization_msgs::Marker::ADD;
        viz_marker_label.pose.position.x = marker.point.x;
        viz_marker_label.pose.position.y = marker.point.y;
        viz_marker_label.pose.position.z = marker.point.z;
        viz_marker_label.pose.orientation.x = 0.0f;
        viz_marker_label.pose.orientation.y = 0.0f;
        viz_marker_label.pose.orientation.z = 0.0f;
        viz_marker_label.pose.orientation.w = 1.0f;
        viz_marker_label.scale.x = 0.01f;
        viz_marker_label.scale.y = 0.01f;
        viz_marker_label.scale.z = 0.01f;
        viz_marker_label.color.r = 1.0f;
        viz_marker_label.color.g = 1.0f;
        viz_marker_label.color.b = 1.0f;
        viz_marker_label.color.a = 1.0f;
        viz_marker_label.text = marker_label_it->second;
        viz_marker_label.lifetime = ros::Duration(0.1);
        marker_label_pub_.publish(viz_marker_label);
      }

      ROS_INFO("Published markers");
    }
  }

  bool set_marker_label(SetMarkerLabelRequest& request, SetMarkerLabelResponse& response)
  {
    marker_label_[request.id.data] = request.label.data;
    return true;
  }

  bool reset_marker_label(ResetMarkerLabelRequest& request, ResetMarkerLabelResponse& response)
  {
    marker_label_.erase(request.id.data);
    return true;
  }

  bool set_marker_color(SetMarkerColorRequest& request, SetMarkerColorResponse& response)
  {
    marker_color_[request.id.data] = request.color;
    return true;
  }

  bool reset_marker_color(ResetMarkerColorRequest& request, ResetMarkerColorResponse& response)
  {
    marker_color_.erase(request.id.data);
    return true;
  }
private:
  ros::Subscriber sub_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_label_pub_;
  ros::NodeHandle nh_;
  std::string frame_;
  std::string ns_;
  ros::ServiceServer set_marker_color_;
  ros::ServiceServer reset_marker_color_;
  ros::ServiceServer set_marker_label_;
  ros::ServiceServer reset_marker_label_;
  std_msgs::ColorRGBA default_marker_color_;
  std::map<int, std_msgs::ColorRGBA> marker_color_;
  std::map<int, std::string> marker_label_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_visualization");
  ros::NodeHandle nh("~");

  MarkerVisualizer marker_visualizer(nh);

  ros::spin();

  ROS_INFO("Shutting down marker visualization");

  return 0;
}
