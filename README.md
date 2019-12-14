= Marker Tracking =
This is a collection of packages that support determining the configuration of a robot using motion capture markers.

== Goal ==
Typically to determine the configuration of a rigid body using motion capture, at least three markers need to be applied to the body that is to be tracked.
In the case of a robot with rigid links this can be reduced to one marker per link by using the kinematic model of the robot.

== Contents ==
This repository contains the following packages:
* frame_marker_tracking: Contains various nodes for using the marker tracker (see below)
* marker_tests: Utilities for testing the frame_marker_tracker, by publishing the expected marker position based on the actual robot configuration
* marker_visualization: Contains a node for visualizing marker positions in rviz
* marker_visualization_srvs: Contains ROS service definitions for interacting with markers
* mocap_base: Base motion capture utilities (from KumarRobotics)
* mocap_lib: Library for working with the markerfile
* mocap_qualisys: Qualisys motion capture system driver (from KumarRobotics)
* qualisys_msgs: Message definitions for the motion capture system
* workcell: Launch files

== Nodes ==

=== frame_marker_tracking/frame_marker_exporter ===
Interactive tool for exporting a marker file that specifies which markers are assigned to which link. Best to start this with rosrun or when embedded in a roslaunch, ran in a separate terminal.

==== Subscribed topics ====
* `markers` (qualisys_msgs/Markers): Topic to which the motion capture system publishes raw marker data.
* `frames` (tf/tfMessage): Topic to which tf frames are published.

==== Used services ====
* `/marker_visualization/set_marker_color`: Called to highlight the marker which is currently being editted.
* `/marker_visualization/reset_marker_color`: Called after a marker is editted to reset its color.

==== Arguments ====
* `output_path` (default: _markers.xml_): Path to which the marker file should be written.
* `source_frame` (default: empty string): tf frame which markers are stored in, i.e. the motion capture system origin frame
* `transform_to_local_frame` (default: true): Specifies whether transforms should be calculated based on the origin of the link which a marker is attached to or based on a global frame.
* `target_frame` (default: empty string): tf frame which markers should be stored in. Required if transform_to_local_frame is true.

=== frame_marker_tracking/frame_marker_updater ===
Because Qualisys uses anonymous markers, when the motion capture system is restarted (or the system lost track of a marker) markers will be reassigned an id.
This node, which should be started using rosrun, determines based on the robot model and euclidian distance what the id is of the detected markers.
