#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <qualisys_msgs/Markers.h>
#include <geometry_msgs/Transform.h>
#include <marker_visualization_srvs/SetMarkerColor.h>
#include <marker_visualization_srvs/ResetMarkerColor.h>
#include <Eigen/Dense>
#include <iostream>
#include <ostream>
#include <fstream>
#include <string>
#include <map>
#include <utility>

struct Marker {
  int id;
  float x;
  float y;
  float z;
  std::string label;
};

class FrameMarkerExporter {
public:
  FrameMarkerExporter(ros::NodeHandle nh) : nh_(nh), listener_(buffer_)
  {
    // Using parameters instead of topic remappings to allow config using rosrun
    nh_.param("output_path", output_path_, std::string("markers.xml"));
    nh_.param("markers", markers_topic_, std::string("markers"));
    nh_.param("frames", frames_topic_, std::string("frames"));
    nh_.param("source_frame", source_frame_, std::string(""));
    nh_.param("transform_to_local_frame", transform_locally_, true);
    nh_.param("target_frame", target_frame_, std::string(""));
    if (transform_locally_)
    {
      std::cout << "Writing the transform based on the local frame.\n";
    }
    else if (target_frame_ == "")
    {
      std::cerr << "Global transformation but target frame is not set.\n";
      ros::shutdown();
      return;
    }
    markers_sub_ = nh_.subscribe<qualisys_msgs::Markers>(
      markers_topic_, 1, &FrameMarkerExporter::markers_callback, this
    );
    frames_sub_ = nh_.subscribe<tf::tfMessage>(
      frames_topic_, 1, &FrameMarkerExporter::transforms_callback, this
    );
    set_marker_color_ = nh_.serviceClient<marker_visualization_srvs::SetMarkerColor>(
      "/marker_visualization/set_marker_color"
    );
    reset_marker_color_ = nh_.serviceClient<marker_visualization_srvs::ResetMarkerColor>(
      "/marker_visualization/reset_marker_color"
    );
  }

  void markers_callback(const qualisys_msgs::Markers::ConstPtr& markers)
  {
    if (source_frame_ == target_frame_ || transform_locally_)
    {
      process_markers_without_transformation(markers);
    }
    else
    {
      process_markers_with_global_transformation(markers);
    }
    markers_received_ = true;
    handle_markers_and_frames();
  }

  void process_markers_with_global_transformation(const qualisys_msgs::Markers::ConstPtr& markers)
  {
    geometry_msgs::TransformStamped transform = buffer_.lookupTransform(
      source_frame_, target_frame_, ros::Time(0), ros::Duration(1.0)
    );

    Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);

    for (const qualisys_msgs::Marker& marker : markers->markers)
    {
      Eigen::Vector3d v(marker.point.x, marker.point.y, marker.point.z);
      Eigen::Vector3d res = eigen_transform.inverse() * v;
      add_or_update_marker(marker.index.data, res(0), res(1), res(2));
    }
  }

  void process_markers_without_transformation(const qualisys_msgs::Markers::ConstPtr& markers)
  {
    for (const qualisys_msgs::Marker& marker : markers->markers)
    {
      add_or_update_marker(marker.index.data, marker.point.x, marker.point.y, marker.point.z);
    }
  }

  void add_or_update_marker(int id, double x, double y, double z) 
  {
    bool marker_already_exists = false;
    for (Marker& existing_marker : markers_)
    {
      if (existing_marker.id == id)
      {
        marker_already_exists = true;
        update_marker(existing_marker, x, y, z);
        break; // do not add markers that already exist twice
      }
    }
    if (!marker_already_exists)
    {
      add_marker(id, x, y, z);
    }
  }

  void add_marker(int id, double x, double y, double z)
  {
      Marker simple_marker;
      simple_marker.id = id;
      update_marker(simple_marker, x, y, z);
      markers_.push_back(simple_marker);
  }

  void update_marker(Marker& marker, double x, double y, double z)
  {
      marker.x = x;
      marker.y = y;
      marker.z = z;
  }

  void transforms_callback(const tf::tfMessage::ConstPtr& transforms)
  {
    for (const geometry_msgs::TransformStamped& transform : transforms->transforms)
    {
      frames_.insert(std::string(transform.header.frame_id));
      frames_.insert(std::string(transform.child_frame_id));
    }
    frames_received_ = true;
    handle_markers_and_frames();
  }

  void handle_markers_and_frames()
  {
    if ((ros::Time::now().sec - init_time_.sec) < 5) // lets give it a few seconds
      return;
    if (markers_received_ && frames_received_ && !results_written_)
    {
      std_msgs::ColorRGBA highlight_color;
      highlight_color.r = 1.0f;
      highlight_color.g = 0.0f;
      highlight_color.b = 0.0f;
      highlight_color.a = 1.0f;

      std::cout << "Frames:\n";
      for (const std::string& frame : frames_)
      {
        std::cout << frame << '\n';
      }

      for (Marker& marker : markers_)
      {
        set_marker_color(marker.id, highlight_color);
        bool marker_frame_entered = enter_marker_frame(marker);
        reset_marker_color(marker.id);
        if (!marker_frame_entered)
          return;
      }

      std::cout << "Processed all markers received" << std::endl;
      write_markers();
      ros::shutdown();
    }
  }

private:
  void set_marker_color(int marker, const std_msgs::ColorRGBA& highlight_color)
  {
    marker_visualization_srvs::SetMarkerColor set_marker_color_srv;
    set_marker_color_srv.request.id.data = marker;
    set_marker_color_srv.request.color = highlight_color;
    set_marker_color_.call(set_marker_color_srv);
  }

  bool enter_marker_frame(Marker& marker)
  {
    std::string frame;
    std::string label;
    std::cout << "Marker id: " << marker.id << '\n'
              << "Enter frame id to assign to this marker:\n";
    std::getline(std::cin, frame);
    if (shutdown_if_exit(frame))
      return false;
    while (frames_.find(frame) == frames_.end())
    {
      std::cout << "The entered frame id does not exist.\n"
                << "Enter frame id to assign to this marker:\n";
      std::getline(std::cin, frame);
      if (shutdown_if_exit(frame))
        return false;
    }

    std::cout << "Enter label to assign to this marker:\n";
    std::getline(std::cin, label);
    if (shutdown_if_exit(label))
      return false;
    marker.label = label;
    if (transform_locally_)
    {
      std::cout << "Transforming to local frame.\n";
      transform_to_local_frame(marker, frame);
    }
    frame_markers_.insert(std::make_pair(frame, marker));
    return true;
  }

  void transform_to_local_frame(Marker& marker, const std::string& frame)
  {
    geometry_msgs::TransformStamped transform = buffer_.lookupTransform(
      source_frame_, frame, ros::Time(0), ros::Duration(1.0)
    );
    Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);
    Eigen::Vector3d v(marker.x, marker.y, marker.z);
    Eigen::Vector3d res = eigen_transform.inverse() * v;
    marker.x = res(0);
    marker.y = res(1);
    marker.z = res(2);
  }

  bool shutdown_if_exit(const std::string& frame)
  {
    if (frame == "exit")
    {
      ros::shutdown();
      return true;
    }
    return false;
  }

  void reset_marker_color(int marker)
  {
    marker_visualization_srvs::ResetMarkerColor reset_marker_color_srv;
    reset_marker_color_srv.request.id.data = marker;
    reset_marker_color_.call(reset_marker_color_srv);
  }

  void write_markers()
  {
    std::ofstream myfile(output_path_, std::ios::out);
    myfile << "<markerFrames>\n";
    std::string latched("");
    for (std::pair<std::string, Marker> frame_marker : frame_markers_)
    {
      std::string frame(frame_marker.first);
      Marker marker = frame_marker.second;
      if (latched != "" && frame != latched)
      {
        myfile << "  </frame>\n";
      }
      if (frame != latched)
        myfile << "  <frame id=\"" << frame << "\" joint_type=\"revolute\">\n";
      myfile << "    <marker mocap_id=\"" << marker.id << 
                "\" label=\"" << marker.label << "\">" << 
                marker.x << " " << marker.y << " " << marker.z << 
                "</marker>\n";
      latched = frame;
    }
    if (latched != "")
      myfile << "  </frame>\n";
    myfile << "</markerFrames>\n";
    myfile.close();
    results_written_ = true;
  }

  std::string markers_topic_;
  std::string frames_topic_;
  std::string output_path_;
  std::string source_frame_;
  std::string target_frame_;
  bool transform_locally_ = false;
  ros::Subscriber frames_sub_;
  ros::Subscriber markers_sub_;
  ros::ServiceClient set_marker_color_;
  ros::ServiceClient reset_marker_color_;
  ros::NodeHandle nh_;
  bool markers_received_ = false;
  bool frames_received_ = false;
  bool results_written_ = false;
  std::vector<Marker> markers_;
  std::set<std::string> frames_;
  std::multimap<std::string, Marker> frame_markers_;
  ros::Time init_time_ = ros::Time::now();
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frame_marker_exporter");
  ros::NodeHandle nh("~");
  std::cout << "Starting frame marker exporter.\n"
            << "Listening for five seconds for markers and transforms.\n"
            << "Enter 'save' to save and exit without considering every marker.\n"
            << "Enter 'exit' to quit without saving." << std::endl;
  FrameMarkerExporter exporter(nh);
  ros::spin();
  std::cout << "Finished" << std::endl;
}
