#include <ros/ros.h>
#include <tinyxml.h>
#include <mocap_lib/marker_lib.h>
#include <qualisys_msgs/Markers.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <string>
#include <cmath>
#include <map>

using namespace std;

struct MarkerDistance
{
  MarkerDistance(marker_lib::Marker origin, marker_lib::Marker destination, double distance) : origin(origin), destination(destination), distance(distance) {}
  marker_lib::Marker origin;
  marker_lib::Marker destination;
  double distance;
};

class FrameMarkerUpdater
{
public:

  FrameMarkerUpdater(ros::NodeHandle& nh, vector<shared_ptr<marker_lib::Marker>> local_markers) : nh_(nh), listener_(buffer_)
  {
    nh_.param("marker_path", input_path_, string("markers.xml"));
    nh_.param("output_path", output_path_, input_path_);
    nh_.param("markers", markers_topic_, string("markers"));
    nh_.param("source_frame", source_frame_, string(""));

    // transform markers to global frame
    for (auto marker_ptr : local_markers)
    {
      auto t = buffer_.lookupTransform(
        marker_ptr->frame, source_frame_, ros::Time(0), ros::Duration(2.0)
      );
      marker_lib::Marker marker(
        marker_ptr->id,
        marker_ptr->x - t.transform.translation.x,
        marker_ptr->y - t.transform.translation.y,
        marker_ptr->z - t.transform.translation.z,
        marker_ptr->label,
        marker_ptr->frame
      );
      saved_markers_.push_back( make_shared<marker_lib::Marker>( marker ) );
    }

    markers_sub_ = nh_.subscribe<qualisys_msgs::Markers>(
      markers_topic_, 1, &FrameMarkerUpdater::markers_callback, this
    );
  }

  void markers_callback(const qualisys_msgs::Markers::ConstPtr& markers)
  {
    // origin of MarkerDistance is the stored marker, destination is the measured marker
    vector<MarkerDistance> marker_distances;
    get_marker_distances(markers->markers, saved_markers_, marker_distances);

    vector<MarkerDistance> lowest_distance;
    get_lowest_distance(marker_distances, lowest_distance);

    map<int, int> marker_map;
    get_marker_map(lowest_distance, marker_map);

    //marker_lib::write_markers(output_path_, marker_map);

    ros::shutdown();
  }

private:
  void get_marker_distances(
    const vector<qualisys_msgs::Marker>& to, 
    const vector<shared_ptr<marker_lib::Marker>>& from,
    vector<MarkerDistance>& res
  )
  {
    for (const qualisys_msgs::Marker& marker_msg : to)
    {
      marker_lib::Marker measured_marker(
        marker_msg.index.data, marker_msg.point.x, marker_msg.point.y, marker_msg.point.z
      );
      for (const shared_ptr<marker_lib::Marker> stored_marker : from)
      {
        double distance = sqrt(
          pow(measured_marker.x - stored_marker->x, 2) +
          pow(measured_marker.y - stored_marker->y, 2) +
          pow(measured_marker.z - stored_marker->z, 2)
        );
        res.push_back(MarkerDistance(*stored_marker, measured_marker, distance));
      }
    }
  }

  void get_lowest_distance(
    const vector<MarkerDistance>& distances,
    //const map<pair<marker_lib::Marker, marker_lib::Marker>, double>& distance,
    vector<MarkerDistance>& res
  )
  {
    for (const MarkerDistance& d : distances)
    {
      MarkerDistance* found = nullptr;
      for (MarkerDistance& temp : res)
      {
        if (temp.origin == d.origin)
        {
          found = &temp;
          break;
        }
      }
      if (found != nullptr && found->distance > d.distance)
      {
        found->destination = d.destination;
        found->distance = d.distance;
      }
      else
      {
        res.push_back(d);
      }
    }
  }

  void get_marker_map(const vector<MarkerDistance>& distance, map<int, int>& res)
  {
    for (const MarkerDistance& d : distance)
    {
      res.insert(make_pair(d.origin.id, d.destination.id));
    }
  }

  ros::NodeHandle nh_;
  string input_path_;
  string output_path_;
  string markers_topic_;
  string source_frame_;
  ros::Subscriber markers_sub_;
  vector<shared_ptr<marker_lib::Marker>> saved_markers_;
  vector<marker_lib::Marker> real_time_markers_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frame_marker_updater");
  ros::NodeHandle nh("~");
  vector<shared_ptr<marker_lib::Marker>> loaded_markers;
  std::string marker_path;
  nh.param("marker_path", marker_path, std::string("markers.xml"));
  if (!parse_markers(marker_path, loaded_markers))
  {
    ROS_ERROR("Unable to parse marker definition");
    return 1;
  }
  cout << "Starting frame marker updater.\n";
  FrameMarkerUpdater updater(nh, loaded_markers);
  ros::spin();
  cout << "Finished\n";
  return 0;
}
