#include <ros/ros.h>
#include "frame_marker_tracking/robot_frame_estimator_node.hpp"
#include <tf/tfMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <mocap_lib/marker_lib.h>
#include <mocap_lib/geometry_lib.h>
#include <mocap_lib/urdf_math.h>
#include <urdf/model.h>
#include <string>

using namespace std;

bool try_insert_joint_position(
  const string& frame_name,
  const urdf::Vector3& position,
  map<string, urdf::Vector3>& known_joint_positions,
  map<string, urdf::Vector3>::iterator& position_it
)
{
  auto inserted = known_joint_positions.insert( make_pair( frame_name, position ) );
  if ( !inserted.second )
  {
    ROS_ERROR_STREAM("Failed to insert known joint position for joint " << frame_name);
    return false;
  }
  position_it = inserted.first;
  return true;
}

/** Print information about the joint */
void print_joint( urdf::JointSharedPtr joint )
{
  ROS_DEBUG_STREAM("Joint");
  ROS_DEBUG_STREAM("Name: " << joint->name);
  ROS_DEBUG_STREAM("Axis x: " << joint->axis.x << ", y: " << joint->axis.y << ", z: " << joint->axis.z);
  ROS_DEBUG_STREAM("Joint type: ");
  switch ( joint->type ) {
    case urdf::Joint::REVOLUTE:
      ROS_DEBUG_STREAM("Revolute");
      break;
    case urdf::Joint::FIXED:
      ROS_DEBUG_STREAM("Fixed");
      break;
    default:
      ROS_DEBUG_STREAM("Unknown");
  }
  ROS_DEBUG_STREAM("Parent link name: " << joint->parent_link_name);
  ROS_DEBUG_STREAM("Child link name: " << joint->child_link_name);
  double roll, pitch, yaw;
  urdf::Pose& pose = joint->parent_to_joint_origin_transform;
  pose.rotation.getRPY( roll, pitch, yaw );
  ROS_DEBUG_STREAM("Parent to joint origin x: " << pose.position.x 
         << ", y: " << pose.position.y  
         << ", z: " << pose.position.z 
         << ", roll: " << roll 
         << ", pitch: " << pitch
         << ", yaw: " << yaw);
}

/** Print information about the link */
void print_link( urdf::LinkSharedPtr link )
{
    ROS_DEBUG_STREAM("Link: " << link->name);
    urdf::JointSharedPtr parent = link->parent_joint;
    if ( parent != nullptr )
    {
      print_joint( parent );
    }
    ROS_DEBUG_STREAM("Child joints:");
    for ( auto& joint : link->child_joints )
      ROS_DEBUG_STREAM(joint->name);
    ROS_DEBUG_STREAM("Child links:");
    for ( auto& child_link : link->child_links )
      ROS_DEBUG_STREAM(child_link->name);
    ROS_DEBUG_STREAM("================");
}

/** Print information about the vector */
void print_vector( urdf::Vector3 vec)
{
  ROS_DEBUG_STREAM("x: " << vec.x << ", y: " << vec.y << ", z: " << vec.z);
}

/** Print information about the rotation */
void print_rotation( urdf::Rotation rot )
{
  double roll, pitch, yaw;
  rot.getRPY( roll, pitch, yaw );
  ROS_DEBUG_STREAM("roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw);
}

/** Convenience function for extracting a vector of markers from a multimap of pairs of joint/link frame and marker
 * @param source The source multimap to extract the markers from
 * @param target The target vector to which the markers will be pushed back
 */
void extract_marker_list( const multimap<string, shared_ptr<marker_lib::Marker>>& source, vector<shared_ptr<marker_lib::Marker>>& target )
{
  for ( auto& pair : source )
  {
    target.push_back( pair.second );
  }
}

/** Constructor for a robot frame estimator
 * @param nh ROS nodehandle used to retrieve parameters and advertise a topic
 * @param robot_model URDF description of the robot which frames should be tracked
 * @param frames_and_markers Multimap that specifies which markers are rigidly attached to which frames
 */
RobotFrameEstimator::RobotFrameEstimator(
  ros::NodeHandle& nh, const urdf::Model& robot_model, 
  const multimap<string, shared_ptr<marker_lib::Marker>>& frames_and_markers
) : nh_( nh ), robot_model_( robot_model ), listener_( buffer_ )
{
  nh_.param( "markers", markers_topic_, string("markers") );
  nh_.param( "source_frame", source_frame_, string("base_link") );
  nh_.param( "global_frame", global_frame_, string("mocap") );
  stored_markers_ = frames_and_markers;
  // transform_markers_to_frame( source_frame_, frames_and_markers, buffer_, saved_markers_ );
  // keep a copy of the marker list
  extract_marker_list( stored_markers_, marker_list_ );
  markers_sub_ = nh_.subscribe<qualisys_msgs::Markers>(
    markers_topic_, 1, &RobotFrameEstimator::markers_callback, this
  );
}

/** Find the marker with the specified id
 * @param stored_markers Multimap of frames and their rigidly attached markers
 * @param frame Frame for which the marker should be retrieved
 * @param id id number of the marker that should be retrieved
 */
shared_ptr<marker_lib::Marker> find_stored_marker(
  const std::multimap<std::string, shared_ptr<marker_lib::Marker>>& stored_markers,
  const string& frame,
  int id
)
{
  auto markers = stored_markers.equal_range( frame );
  for ( auto marker_pair = markers.first; marker_pair != markers.second; marker_pair++ )
  {
    if ( (marker_pair->second)->id == id )
    {
      return marker_pair->second;
    }
  }
  return nullptr;
}

/** Forward kinematics to determine position of a joint
 * @param parent_joint_position The position of the joint's parent joint
 * @param parent_joint_rotation The rotation of the joint's parent joint
 * @param joint Joint of which the position should be determined
 * @param known_joint_positions Map of joint name and joint position vector (relative to a global frame)
 * @param position_it Iterator that specifies where to store the result
 */
bool get_joint_position(
  const urdf::Vector3& parent_joint_position,
  const urdf::Rotation& parent_joint_rotation,
  const urdf::JointSharedPtr joint,
  map<string, urdf::Vector3>& known_joint_positions,
  map<string, urdf::Vector3>::iterator& position_it
)
{
  urdf::Vector3 new_position = parent_joint_position + parent_joint_rotation * 
    joint->parent_to_joint_origin_transform.position;
  return try_insert_joint_position( joint->child_link_name, new_position, known_joint_positions, position_it );
}

/** Mapping function from our Point3D to urdf's Vector3 (URDF does not distinguish between vectors and points)
 * @param point Point of which we want to obtain a vector
 * @return New vector based on the point
 */
urdf::Vector3 to_urdf_vector3( const geometry_lib::Point3D& point )
{
  return urdf::Vector3( point.x, point.y, point.z );
}

/** Gets the position of the joint based on a vector of intersecting points
 * Assumes that the intersection points are the result of intersecting two spheres with a plane
 * @param intersection_points Possible points at which the joint could be located
 * @param parent_joint_position Position of the parent joint
 * @param joint Joint for which to determine the position
 * @param known_joint_positions Map of joint name and joint position vector (relative to a global frame)
 * @param joint_position_it Iterator that specifies where to store the result
 * @return Flag that specifies whether the joint position could be determined
 */
bool get_joint_position(
  const vector<geometry_lib::Point3D>& found_intersection_points,
  const urdf::Vector3& parent_joint_position,
  const urdf::JointSharedPtr joint,
  map<string, urdf::Vector3>& known_joint_positions,
  map<string, urdf::Vector3>::iterator& joint_position_it
)
{
  if ( found_intersection_points.size() == 0 )
  {
    ROS_ERROR_STREAM("No intersection points found");
    return false;
  }
  else if ( found_intersection_points.size() == 1 )
  {
    urdf::Vector3 joint_new_position = parent_joint_position + to_urdf_vector3( found_intersection_points[0] );
    return try_insert_joint_position(
      joint->child_link_name, joint_new_position, known_joint_positions, joint_position_it
    );
  }
  else
  {
    map<geometry_lib::Point3D, int> number_of_intersections;
    for ( int i = 0; i < found_intersection_points.size(); ++i )
    {
      for ( int j = 1; j < found_intersection_points.size(); ++j )
      {
        if ( i == j )
          continue;
        if ( found_intersection_points[i] == found_intersection_points[j] )
        {
          auto counter = number_of_intersections.find( found_intersection_points[i] );
          if ( counter == number_of_intersections.end() )
          {
            number_of_intersections.insert( make_pair( found_intersection_points[0], 1 ) );
          }
          else
          {
            counter->second++;
          }
        }
      }
    }
    if ( number_of_intersections.size() == 0 )
    {
      return false;
    }
    shared_ptr<geometry_lib::Point3D> point_with_most_intersections;
    int max_intersections = -1;
    for ( auto vector_counter_pair : number_of_intersections )
    {
      if ( vector_counter_pair.second > max_intersections )
      {
        max_intersections = vector_counter_pair.second;
        point_with_most_intersections = make_shared<geometry_lib::Point3D>(vector_counter_pair.first);
      }
    }
    urdf::Vector3 joint_new_position = parent_joint_position + to_urdf_vector3( *point_with_most_intersections );
    return try_insert_joint_position(
      joint->child_link_name, joint_new_position, known_joint_positions, joint_position_it
    );
  }
}

/** Get the rotation between two vectors
 * @param a Source vector of the rotation
 * @param b Target vector of the rotation
 * @return Rotation determined
 */
urdf::Rotation get_rotation( urdf::Vector3 a, urdf::Vector3 b )
{
  a = normalize(a);
  b = normalize(b);
  const urdf::Vector3 cross_product = cross( a, b );
  double a_length = norm( a );
  double b_length = norm( b );
  double w = sqrt( a_length * a_length * 
                   b_length * b_length ) +
             dot( a, b );
  urdf::Rotation rotation( cross_product.x, cross_product.y, cross_product.z, w );
  rotation.normalize();
  return rotation;
}

/** Gets the rotation of a joint based on the difference between the marker position
 * and the expected marker position if the joint was not rotated
 * @param mocap_markers Multimap of frame and marker as detected by the mocap system
 *        (translated to source frame)
 * @param stored_markers Multimap of frame and marker as stored in the markerfile (relative to joint origin)
 * @param parent_joint_rotation Rotation of the parent joint
 * @param position Position of the joint origin relative to the global frame
 * @param joint Joint for which to determine rotation
 * @param res Result map to insert the calculated rotation into
 */
bool get_joint_rotation(
  const multimap<string, shared_ptr<marker_lib::Marker>>& mocap_markers,
  const multimap<string, shared_ptr<marker_lib::Marker>>& stored_markers,
  const urdf::Rotation& parent_joint_rotation,
  const urdf::Vector3& position,
  const urdf::JointSharedPtr joint,
  map<string, urdf::Rotation>& res
)
{
  //double p_roll, p_pitch, p_yaw;
  //parent_joint_rotation.getRPY(p_roll, p_pitch, p_yaw);
  //cout << "Parent joint rotation roll, pitch, yaw: " << p_roll << ", " << p_pitch << ", " << p_yaw << '\n';
  auto markers = mocap_markers.equal_range( joint->child_link_name );
  if ( markers.first == markers.second )
    return false;
 
  vector<shared_ptr<urdf::Rotation>> rotations;
  for ( auto marker_pair = markers.first; marker_pair != markers.second; marker_pair++ )
  {
    shared_ptr<marker_lib::Marker> mocap_marker = marker_pair->second;
    shared_ptr<marker_lib::Marker> stored_marker = find_stored_marker( stored_markers,
                                                                       joint->child_link_name,
                                                                       mocap_marker->id );
    if ( stored_marker == nullptr )
      continue;
    urdf::Vector3 stored_marker_vec( stored_marker->x, stored_marker->y, stored_marker->z );
    //joint->parent_to_joint_origin_transform.rotation.getRPY(p_roll, p_pitch, p_yaw);
    //cout << "Parent to joint origin rotation roll, pitch, yaw: " << p_roll << ", " << p_pitch << ", " << p_yaw << '\n';
    urdf::Rotation stored_marker_rotation = parent_joint_rotation *
                                            joint->parent_to_joint_origin_transform.rotation;
    //stored_marker_rotation.getRPY(p_roll, p_pitch, p_yaw);
    //cout << "Stored marker rotation roll, pitch, yaw: " << p_roll << ", " << p_pitch << ", " << p_yaw << '\n';
    urdf::Vector3 stored_marker_rotated = stored_marker_rotation * stored_marker_vec;
    // vector to marker = marker position - frame position
    urdf::Vector3 vector_to_marker(
      mocap_marker->x - position.x,
      mocap_marker->y - position.y,
      mocap_marker->z - position.z
    );
    urdf::Rotation rotation = get_rotation( stored_marker_rotated , vector_to_marker );
    //  urdf::Rotation rotation = get_rotation( vector_to_marker, stored_marker_rotated );
    //rotation.getRPY(p_roll, p_pitch, p_yaw);
    //cout << "Evaluated rotation roll, pitch, yaw: " << p_roll << ", " << p_pitch << ", " << p_yaw << '\n';
    rotations.push_back( make_shared<urdf::Rotation>( rotation ) );
  }
  urdf::Rotation blended = blend( rotations );
  //blended.getRPY(p_roll, p_pitch, p_yaw);
  //cout << "blended rotation roll, pitch, yaw: " << p_roll << ", " << p_pitch << ", " << p_yaw << '\n';
  res.insert( make_pair( joint->child_link_name, parent_joint_rotation * blended ) );
  return true;
}

typedef shared_ptr<marker_lib::Marker> MarkerPtr;
typedef multimap<string, MarkerPtr> JointMarkerMultimap;
typedef multimap<string, MarkerPtr>::const_iterator JointMarkerMultimapConstIterator;
typedef std::pair<JointMarkerMultimapConstIterator,
                  const JointMarkerMultimapConstIterator> joint_marker_multimap_iterator_pair;

/** Function to find intersection points of marker spheres and joint's parent joint plane
 * @param mocap_markers Multimap of frame and marker as detected by the mocap system
 *        (translated to source frame)
 * @param stored_markers Multimap of frame and marker as stored in the markerfile (relative to joint origin)
 * @param plane Plane in which to detect the intersection points
 * @param circle Circle embedded in 3D that shows the reach of the parent link
 * @param local_origin Origin position of the parent joint (relative to source frame)
 * @param mocap_markers_for_joint Pair of begin and end iterators for the markers for which to 
 *        calculate intersection
 * @param joint Joint which has the marker(s) attached
 * @param found_intersection_points Vector to append found intersection points to
 */
bool find_intersection_points(
  const multimap<string, shared_ptr<marker_lib::Marker>>& mocap_markers,
  const multimap<string, shared_ptr<marker_lib::Marker>>& stored_markers,
  const geometry_lib::Plane& plane,
  const geometry_lib::Circle3D& circle,
  const urdf::Vector3& local_origin,
  const joint_marker_multimap_iterator_pair& mocap_markers_for_joint,
  urdf::JointSharedPtr joint,
  vector<geometry_lib::Point3D>& found_intersection_points
)
{
  for ( auto marker_pair = mocap_markers_for_joint.first; 
        marker_pair != mocap_markers_for_joint.second;
        marker_pair++ )
  {
    const shared_ptr<marker_lib::Marker> marker = marker_pair->second;
    auto stored_marker = find_stored_marker( stored_markers, joint->child_link_name, marker->id );
    geometry_lib::Sphere s( marker->x - local_origin.x,
              marker->y - local_origin.y,
              marker->z - local_origin.z,
              stored_marker->norm() );
    auto sphere_plane_intersection = intersect( s, plane );
    if ( sphere_plane_intersection.has_value() )
    {
      auto intersection_circle = sphere_plane_intersection.value();
      auto circle_circle_intersection = intersect( circle, intersection_circle );
      try
      {
        auto intersection_points = any_cast<pair<geometry_lib::Point3D, geometry_lib::Point3D>>( circle_circle_intersection );
        found_intersection_points.push_back( intersection_points.first );
        if ( intersection_points.first != intersection_points.second )
        {
          found_intersection_points.push_back( intersection_points.second );
        }
      }
      catch ( const bad_any_cast& ex )
      {
        ROS_ERROR_STREAM("No intersection detected between circle and circle");
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  return true;
}

/** Radians to degrees */
double deg( double rad )
{
  return rad * 180 / 3.1415926;
}

/** Update a "skipped link" followed by a link with two markers
 * @param mocap_markers Multimap of frame and marker as detected by the mocap system
 *        (translated to source frame)
 * @param stored_markers Multimap of frame and marker as stored in the markerfile (relative to joint origin)
 * @param j0 Root joint for computation
 * @param j1 Joint's parent joint
 * @param j2 Joint considered
 * @param known_joint_positions Map of joint name and joint position vector (relative to a global frame)
 * @param known_joint_rotations Map of joint name and joint rotation (relative to the rotation in the global frame)
 * @return Whether any changes were made
 */
bool update_skipped_link(
  const multimap<string, shared_ptr<marker_lib::Marker>>& mocap_markers,
  const multimap<string, shared_ptr<marker_lib::Marker>>& stored_markers,
  const urdf::JointSharedPtr j0,
  urdf::JointSharedPtr j1,
  urdf::JointSharedPtr j2,
  map<string, urdf::Vector3>& known_joint_positions, // absolute positions (from source frame)
  map<string, urdf::Rotation>& known_joint_rotations // absolute rotations (from source frame)
)
{
  auto j0_position_it = known_joint_positions.find( j0->child_link_name );
  auto j0_rotation_it = known_joint_rotations.find( j0->child_link_name );
  if ( j0_position_it == known_joint_positions.end() || j0_rotation_it == known_joint_rotations.end() )
  {
    // we need to know the position and rotation of j-2 in order for this procedure to work
    return false;
  }

  map<string, urdf::Vector3>::iterator j1_position_it;
  bool position_determined = get_joint_position(
    j0_position_it->second, j0_rotation_it->second, j1, known_joint_positions, j1_position_it
  );
  if ( !position_determined )
  { // if for some reason we couldn't determine the position of the parent joint, then this procedure won't work
    return false;
  }

  geometry_lib::Plane plane( j2->axis.x, j2->axis.y, j2->axis.z, 0 );
  geometry_lib::Circle3D circle( 0, 0, 0, norm( j2->parent_to_joint_origin_transform.position ), plane );
  auto mocap_markers_for_joint = mocap_markers.equal_range( j2->child_link_name );
  //auto mocap_markers_for_joint = mocap_markers.equal_range( j2->name );
  if ( mocap_markers_for_joint.first == mocap_markers_for_joint.second )
  { // the joint has no markers, hence we can't get the joint's position and rotation
    return true;
  }
  vector<geometry_lib::Point3D> found_intersection_points;
  bool intersection_points_found = find_intersection_points(
    mocap_markers, stored_markers, plane, circle, j1_position_it->second, mocap_markers_for_joint, j2, found_intersection_points
  );
  if ( !intersection_points_found )
  {
    return true;
  }
  
  map<string, urdf::Vector3>::iterator j2_position_it;
  bool joint_position_found = get_joint_position(
    found_intersection_points, j1_position_it->second, j2, known_joint_positions, j2_position_it
  );
  if ( !joint_position_found )
  {
    return true;
  }
  // get the rotation for j1
  urdf::Rotation j1_rotation = get_rotation(
    j0_rotation_it->second * j2->parent_to_joint_origin_transform.position,
    j2_position_it->second - j1_position_it->second
  );
  urdf::Rotation j1_global_rotation = j0_rotation_it->second * j1_rotation;
  auto inserted = known_joint_rotations.insert( make_pair( j1->child_link_name, j1_global_rotation ) );
  if ( !inserted.second )
  {
    ROS_ERROR_STREAM("Failed to insert known joint rotation for joint " << j1->name);
    return true;
  }

  // get the rotation for j2
  bool found_j2_rotation = get_joint_rotation( mocap_markers, stored_markers, j1_rotation, j2_position_it->second, j2, known_joint_rotations );
  if ( !found_j2_rotation )
  {
    ROS_ERROR_STREAM("Could not find rotation of j2");
  }

  // even if we couldn't find the rotation of this joint, we should have still found the parent joint rotation
  return true;
}

marker_lib::Marker mocap_marker_to_marker_lib_marker( const qualisys_msgs::Marker& in )
{
  return marker_lib::Marker( in.index.data, in.point.x, in.point.y, in.point.z );
}

urdf::Vector3 average_vector( const vector<urdf::Vector3>& vectors )
{
  urdf::Vector3 result;
  for (auto v : vectors)
  {
    result += v / vectors.size();
  }
  return result;
}

/** Callback function for when markers are received */
void RobotFrameEstimator::markers_callback( const qualisys_msgs::Markers::ConstPtr& markers )
{
  multimap<string, shared_ptr<marker_lib::Marker>> mocap_markers;
  map<string, int> num_markers_per_joint;
  for ( qualisys_msgs::Marker marker : markers->markers )
  {
    shared_ptr<marker_lib::Marker> found_marker = find_marker( marker.index.data, marker_list_ );
    if ( found_marker != nullptr )
    {
      auto num_markers_it = num_markers_per_joint.find( found_marker->frame );
      if ( num_markers_it == num_markers_per_joint.end() )
      {
        num_markers_per_joint.insert( make_pair( found_marker->frame, 1 ) );
      }
      else
      {
        num_markers_it->second++;
      }
      mocap_markers.insert(
        make_pair(
          found_marker->frame,
          make_shared<marker_lib::Marker>( mocap_marker_to_marker_lib_marker( marker ) ) 
        )
      );
    }
  }

  bool changed = true;
  map<string, urdf::Vector3> known_joint_positions;
  map<string, urdf::Rotation> known_joint_rotations;

  map<string, urdf::JointSharedPtr> joint_for_link;
  geometry_msgs::TransformStamped global_frame_to_source_frame_;
  bool transformFound = false;
  while ( !transformFound )
  {
    try
    {
      global_frame_to_source_frame_ = buffer_.lookupTransform(
        global_frame_, source_frame_, ros::Time(0)
      );
      transformFound = true;
    }
    catch ( tf2::TransformException& ex )
    {
      ROS_WARN( "%s", ex.what() );
      ros::Duration(1.0).sleep();
    }
  }

  // joints_ is a map of the joint name and a JointSharedPtr
  for ( const auto& joint : robot_model_.joints_ )
  {
    joint_for_link.insert( make_pair( ((joint.second)->child_link_name ), joint.second ) );
    if ((joint.second)->parent_link_name == source_frame_)
    {
      urdf::Vector3 transform_vec(
        global_frame_to_source_frame_.transform.translation.x,
        global_frame_to_source_frame_.transform.translation.y,
        global_frame_to_source_frame_.transform.translation.z
      );
      urdf::Rotation rotation_quat = geometry_msgs_quaternion_to_urdf(
        global_frame_to_source_frame_.transform.rotation
      );
      known_joint_positions.insert( make_pair( source_frame_, transform_vec ) );
      known_joint_rotations.insert( make_pair( source_frame_, rotation_quat ) );
    }
  }

  while ( changed )
  {
    changed = false;

    for ( const auto& joint_pair : robot_model_.joints_ )
    {
      const auto& joint = joint_pair.second;
      if (joint->child_link_name == "crane_x7_shoulder_revolute_part_link") {
        cout << "";
      }
      if (joint->child_link_name == "crane_x7_shoulder_fixed_part_link") {
        cout << "";
      }
      auto position_it = known_joint_positions.find( joint->child_link_name );
      auto rotation_it = known_joint_rotations.find( joint->child_link_name );
      bool position_known = position_it != known_joint_positions.end();
      bool rotation_known = rotation_it != known_joint_rotations.end(); 

      // if position and rotation of this joint are known, then we have nothing left to do
      if ( position_known && rotation_known )
        continue;

      urdf::JointSharedPtr parent_parent_joint;
      urdf::JointSharedPtr parent_joint;
      int num_markers = 0;
      auto num_markers_it = num_markers_per_joint.find( joint->child_link_name );
      if ( num_markers_it != num_markers_per_joint.end() )
      {
        num_markers = num_markers_it->second;
      }
      auto parent_joint_position_it = known_joint_positions.find( joint->parent_link_name );
      auto parent_joint_rotation_it = known_joint_rotations.find( joint->parent_link_name );
      bool parent_position_known = parent_joint_position_it != known_joint_positions.end();
      bool parent_rotation_known = parent_joint_rotation_it != known_joint_rotations.end();

      switch ( joint->type ) {
        case urdf::Joint::REVOLUTE:
          {
            bool parents_parent_position_known = false;
            bool parents_parent_rotation_known = false;
            auto parent_joint_it = joint_for_link.find( joint->parent_link_name );
            if ( parent_joint_it != joint_for_link.end() )
            {
              parent_joint = parent_joint_it->second;
              auto parent_parent_joint_it = joint_for_link.find( parent_joint->parent_link_name );
              if ( parent_parent_joint_it != joint_for_link.end() )
              {
                parent_parent_joint = parent_parent_joint_it->second;
                auto parent_parent_position_it = known_joint_positions.find(
                  parent_parent_joint->child_link_name
                );
                auto parent_parent_rotation_it = known_joint_rotations.find(
                  parent_parent_joint->child_link_name
                );
                parents_parent_position_known = parent_parent_position_it != known_joint_positions.end();
                parents_parent_rotation_known = parent_parent_rotation_it != known_joint_rotations.end();
              }
            }

            // check if we can update a joint pose using this marker
            // this is the case if one of the following holds:
            // 1. we know the child joint pose and the parent joint pose or
            // 2. we know the child joint pose or the parent joint pose and at least 1 marker attached or
            // 3. we know the parent joint's parent joint pose and at least 2 markers attached or
            // 4. we can solve for the position of the frame based on the geometry of the robot (not implemented here)
            if ( !position_known )
            {
              if ( !parent_position_known && !parent_rotation_known )
              {
                // if we don't know the parents joint position and orientation then we cannot calculate 
                // the current joint position using a single marker
                // otherwise we can use try the "skip joint" algorithm
                if ( parents_parent_position_known && parents_parent_rotation_known && num_markers == 2 )
                {
                  changed = update_skipped_link( 
                    mocap_markers, stored_markers_, parent_parent_joint, parent_joint, joint, 
                    known_joint_positions, known_joint_rotations
                  );
                }
                continue;
              }

              position_known = get_joint_position(
                parent_joint_position_it->second, parent_joint_rotation_it->second,
                joint, known_joint_positions, position_it
              );

              if ( !position_known )
              {
                continue;
              }

              changed = true;
            }

            if ( !rotation_known )
            {
              rotation_known = get_joint_rotation(
                mocap_markers, stored_markers_, parent_joint_rotation_it->second, 
                position_it->second, joint, known_joint_rotations
              );

              if ( !rotation_known )
              {
                continue;
              }

              changed = true;
            }
          }
          break;
        case urdf::Joint::FIXED:
          if ( parent_position_known && parent_rotation_known )
          {
            auto& joint_position = joint->parent_to_joint_origin_transform.position;
            auto& joint_rotation = joint->parent_to_joint_origin_transform.rotation;
            auto& parent_position = parent_joint_position_it->second;
            auto& parent_rotation = parent_joint_rotation_it->second;
            auto new_position = parent_position + parent_rotation * joint_position;
            auto new_rotation = parent_rotation * joint_rotation;
            new_rotation.normalize();
            known_joint_positions.insert( make_pair( joint->child_link_name, new_position ) );
            known_joint_rotations.insert( make_pair( joint->child_link_name, new_rotation ) );
            changed = true;
            continue;
          }
          break;
        case urdf::Joint::PRISMATIC:
          if (parent_position_known && parent_rotation_known )
          {
            auto& joint_position = joint->parent_to_joint_origin_transform.position;
            auto& joint_rotation = joint->parent_to_joint_origin_transform.rotation;
            auto& parent_position = parent_joint_position_it->second;
            auto& parent_rotation = parent_joint_rotation_it->second;
            auto mocap_markers_for_joint = mocap_markers.equal_range( joint->child_link_name );

            vector<urdf::Vector3> results;
            for ( auto marker_pair = mocap_markers_for_joint.first; 
                  marker_pair != mocap_markers_for_joint.second;
                  marker_pair++ )
            {
              const shared_ptr<marker_lib::Marker> mocap_marker = marker_pair->second;
              auto stored_marker = find_stored_marker(
                stored_markers_, joint->child_link_name, mocap_marker->id
              );
              if ( stored_marker == nullptr )
                continue;
              urdf::Vector3 base_position = parent_position + parent_rotation * joint_position;
              urdf::Vector3 stored_marker_vec(
                stored_marker->x, stored_marker->y, stored_marker->z
              );
              urdf::Vector3 rotated_marker = parent_rotation * stored_marker_vec;
              urdf::Vector3 vector_to_marker(
                mocap_marker->x - base_position.x,
                mocap_marker->y - base_position.y,
                mocap_marker->z - base_position.z
              );
              urdf::Vector3 displacement_vector(
                vector_to_marker.x - rotated_marker.x,
                vector_to_marker.y - rotated_marker.y,
                vector_to_marker.z - rotated_marker.z
              );
              urdf::Rotation actual_rotation = parent_rotation * joint_rotation;
              urdf::Vector3 actual_position = actual_rotation * joint_position;
              urdf::Vector3 new_position = parent_position + actual_position + displacement_vector;
              results.push_back( new_position );
            }
            if (results.size() == 0)
              continue;
            auto average = average_vector( results );
            known_joint_positions.insert( make_pair( joint->child_link_name, average ) );
            known_joint_rotations.insert( make_pair( joint->child_link_name, parent_rotation ) );
            changed = true;
            continue;
          }
          break;
        default: // switch joint->type
          ROS_DEBUG_STREAM("Unsupported joint type");
      }
    }
  }

  //map<string, urdf::Vector3> known_joint_positions;
  //map<string, urdf::Rotation> known_joint_rotations;
  for (auto& pos_pair : known_joint_positions)
  {
    auto rot_pair = known_joint_rotations.find( pos_pair.first );
    if (rot_pair != known_joint_rotations.end())
    {
      const urdf::Vector3& position = pos_pair.second;
      const urdf::Rotation& rotation = rot_pair->second;
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = ros::Time::now();
      transform.header.frame_id = global_frame_;
      //urdf::Vector3 transform_vec(
      //  global_frame_to_source_frame_.transform.translation.x,
      //  global_frame_to_source_frame_.transform.translation.y,
      //  global_frame_to_source_frame_.transform.translation.z
      //);
      //urdf::Rotation rotation_quat(
      //  global_frame_to_source_frame_.transform.rotation.x, global_frame_to_source_frame_.transform.rotation.y,
      //  global_frame_to_source_frame_.transform.rotation.z, global_frame_to_source_frame_.transform.rotation.w
      //);
      transform.child_frame_id = pos_pair.first + "_estimated";
      transform.transform.translation = urdf_to_geometry_msg(
        position
      );
      transform.transform.rotation = urdf_to_geometry_msg(
        rotation
      );
      br_.sendTransform(transform);
    }
  }
}

/** Load the urdf file from the specified path */
urdf::Model load_urdf( const string& urdf_path )
{
  urdf::Model robot_model;
  robot_model.initFile( urdf_path );
  return robot_model;
}
