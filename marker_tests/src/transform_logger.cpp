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

void tf_callback( const tf2_msgs::TFMessage::ConstPtr& transforms )
{
  for ( auto& t : transforms->transforms )
  {
    cout << t.header.stamp << '\t'
         << t.header.frame_id << '\t'
         << t.transform.translation.x << '\t' 
         << t.transform.translation.y << '\t' 
         << t.transform.translation.z << '\t'
         << t.transform.rotation.x << '\t'
         << t.transform.rotation.y << '\t'
         << t.transform.rotation.z << '\t'
         << t.transform.rotation.w << '\n';
  }
}

void timerCallback( const ros::TimerEvent& )
{
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init( argc, argv, "transform_logger" );
  ros::NodeHandle nh("~");
  double duration = nh.param( "duration", 10.0 );
  string topic = nh.param( "input", string( "/tf" ) );
  ros::Subscriber sub = nh.subscribe<tf2_msgs::TFMessage>(
    topic, 1, tf_callback
  );
  ros::Timer timer = nh.createTimer( ros::Duration( duration ), timerCallback );
  ros::spin();
  return 0;
}
