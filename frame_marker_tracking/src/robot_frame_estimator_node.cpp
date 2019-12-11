#include "robot_frame_estimator.cpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_frame_estimator");
  ros::NodeHandle nh("~");

  string urdf_path;
  nh.param("urdf_path", urdf_path, string("urdf.xml"));
  urdf::Model robot_model = load_urdf(urdf_path);

  for (auto& joint : robot_model.joints_) {
    print_joint(joint.second);
  }

  multimap<string, shared_ptr<marker_lib::Marker>> frames_and_markers;
  std::string marker_path;
  nh.param("marker_path", marker_path, std::string("markers.xml"));
  if (!parse_markers(marker_path, frames_and_markers))
  {
    ROS_ERROR("Unable to parse marker definition");
    return 1;
  }

  ROS_DEBUG("Starting robot frame estimator");
  RobotFrameEstimator estimator(nh, robot_model, frames_and_markers);
  ros::spin();
  ROS_DEBUG("Robot frame estimator shutting down");
  return 0;
}
