#ifndef MARKER_LIB_H
#define MARKER_LIB_H

#include <tf2_ros/buffer.h>
#include <map>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace marker_lib
{

enum JointType {
  FIXED,
  PRISMATIC,
  REVOLUTE
};

struct Marker {
  Marker(
    int id=-1, double x=0, double y=0, double z=0, std::string label="", std::string frame=""
  ) : id(id), x(x), y(y), z(z), label(label), frame(frame) {}

  int id;
  double x;
  double y;
  double z;
  std::string label;
  std::string frame;
  JointType joint_type;
  double norm()
  {
    return std::sqrt(x*x + y*y + z*z);
  }
};

bool operator==(const Marker& left, const Marker& right);
bool operator<(const Marker& left, const Marker& right);
std::shared_ptr<Marker> find_marker(int marker_id, const std::vector<std::shared_ptr<Marker>>& markers);
void transform_markers_to_frame(
  const std::string& frame,
  const std::multimap<std::string, Marker>& markers,
  tf2_ros::Buffer& buffer,
  std::multimap<std::string, Marker>& res
);
bool parse_markers(const std::string& input, std::multimap<std::string, std::shared_ptr<Marker>>& res);
bool parse_markers(const std::string& input, std::vector<std::shared_ptr<Marker>>& res);
bool write_markers(const std::string& output, const std::map<int, int>& markers);
void print_markers(const std::vector<Marker>& markers);
}

#endif
