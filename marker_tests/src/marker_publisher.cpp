#include <ros/ros.h>
#include <qualisys_msgs/Markers.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mocap_lib/marker_lib.h>
#include <mocap_lib/urdf_math.h>
#include <string>
#include <vector>
#include <map>

using namespace std;

class MarkerPublisher
{
  public:
    MarkerPublisher( ros::NodeHandle& nh, vector<shared_ptr<marker_lib::Marker>> markers ) : 
      nh_( nh ), markers_( markers ), listener_( buffer_ )
    {
      nh_.param( "markers", markers_topic_, string( "markers" ) );
      nh_.param( "source_frame", source_frame_, string( "" ) );
      sub_ = nh_.subscribe<tf2_msgs::TFMessage>(
        string("/tf"), 1, &MarkerPublisher::tf_callback, this
      );
      pub_ = nh_.advertise<qualisys_msgs::Markers>( markers_topic_, 1000 );
    }

    void tf_callback( const tf2_msgs::TFMessage::ConstPtr& transforms )
    {
      /*map<string, geometry_msgs::TransformStamped> transform_map;
      for ( auto& transform_stamped : transforms->transforms )
      {
        transform_map.insert( make_pair( transform_stamped.header.frame_id, transform_stamped) );
      }*/

      qualisys_msgs::Markers msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = source_frame_;
      for ( auto marker_ptr : markers_ )
      {
        /*auto transform_it = transform_map.find( marker_ptr->frame );
        if ( transform_it != transform_map.end() )
        {
          auto& local_transform = transform_it->second;
          */
          auto global_transform = buffer_.lookupTransform(
            source_frame_, marker_ptr->frame,
            ros::Time( 0 ), ros::Duration(5)
          );
          urdf::Vector3 translation = geometry_msgs_translation_to_urdf(
            global_transform.transform.translation
          );
          urdf::Rotation rotation = geometry_msgs_quaternion_to_urdf(
            global_transform.transform.rotation
          );
          urdf::Vector3 marker_vec( marker_ptr->x, marker_ptr->y, marker_ptr->z );

          urdf::Vector3 position = translation + rotation * marker_vec;

          qualisys_msgs::Marker m;
          m.index.data = marker_ptr->id;
          m.point.x = position.x;
          m.point.y = position.y;
          m.point.z = position.z;
          msg.markers.push_back( m );
        //}
      }
      pub_.publish( msg );
    }

  private:
    ros::NodeHandle nh_;
    vector<shared_ptr<marker_lib::Marker>> markers_;
    string markers_topic_;
    string source_frame_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
};

int main(int argc, char** argv)
{
  ros::init( argc, argv, "marker_publisher" );
  ros::NodeHandle nh("~");
  vector<shared_ptr<marker_lib::Marker>> loaded_markers;
  string marker_path;
  nh.param( "marker_path", marker_path, string( "markers.xml" ) );
  if ( !parse_markers( marker_path, loaded_markers ) )
  {
    ROS_ERROR( "Unable to parse marker definition" );
    return 1;
  }
  cout << "Starting virtual marker publisher.\n";
  MarkerPublisher mp( nh, loaded_markers );
  ros::spin();
  cout << "Finished\n";
  return 0;
}
