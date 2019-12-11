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

class TransformEstimator
{
  public:
    TransformEstimator( ros::NodeHandle& nh, vector<shared_ptr<marker_lib::Marker>> markers ) : 
      nh_( nh ), markers_( markers ), listener_( buffer_ )
    {
      nh_.param( "output", transforms_topic_, string( "output" ) );
      nh_.param( "suffix", suffix_, string( "_estimated" ) );
      sub_ = nh_.subscribe<tf2_msgs::TFMessage>(
        string("/tf"), 1, &TransformEstimator::tf_callback, this
      );
      pub_ = nh_.advertise<tf2_msgs::TFMessage>( transforms_topic_, 1000 );
    }

    void tf_callback( const tf2_msgs::TFMessage::ConstPtr& transforms )
    {
      tf2_msgs::TFMessage msg;
      for ( auto marker_ptr : markers_ )
      {
        geometry_msgs::TransformStamped transform;
        try {
          transform = buffer_.lookupTransform(
            marker_ptr->frame + suffix_, marker_ptr->frame, ros::Time(0), ros::Duration(1.0)
          );
        }
        catch ( const tf2::LookupException& ex )
        {
          continue;
        }
        catch ( const tf2::ExtrapolationException e )
        {
          continue;
        }
        msg.transforms.push_back( transform );
      }
      pub_.publish( msg );
    }

  private:
    ros::NodeHandle nh_;
    vector<shared_ptr<marker_lib::Marker>> markers_;
    string transforms_topic_;
    string suffix_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
};

int main(int argc, char** argv)
{
  ros::init( argc, argv, "transform_estimator" );
  ros::NodeHandle nh("~");
  vector<shared_ptr<marker_lib::Marker>> loaded_markers;
  string marker_path;
  nh.param( "marker_path", marker_path, string( "markers.xml" ) );
  if ( !parse_markers( marker_path, loaded_markers ) )
  {
    ROS_ERROR( "Unable to parse marker definition" );
    return 1;
  }
  cout << "Starting transform estimator.\n";
  TransformEstimator mp( nh, loaded_markers );
  ros::spin();
  cout << "Finished\n";
  return 0;
}
