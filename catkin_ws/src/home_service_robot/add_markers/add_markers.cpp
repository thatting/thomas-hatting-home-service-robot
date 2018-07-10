/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


// ****************************************************************************
// *********Project: Home Service Robot - Thomas Hatting 29th May 2018*********
// ****************************************************************************

// This C++ script has been developed by means of following: 
// a ) Tutorial on ros.org: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
// b ) Udacity Classroom material term 2

//  --------This project has been created on the Jetson TX2-------------------------


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>  


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);


  // Set our initial shape type to be an arrow
  uint32_t shape = visualization_msgs::Marker::CUBE;


  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();


  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;


  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;



  // ***Publish the marker at the pick-up zone***

  // Define quaternion
  tf::Quaternion q1 = tf::createQuaternionFromRPY(0, 0, 1.57);  //Create quaternion from yaw = +90 degrees
  q1.normalize();						//Source ros.org: http://wiki.ros.org/tf2/Tutorials/Quaternions

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 3.75;
  marker.pose.position.y = 4.0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = q1[0];
  marker.pose.orientation.y = q1[1];
  marker.pose.orientation.z = q1[2];
  marker.pose.orientation.w = q1[3];


  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;


  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);

  //***Pause for 5 seconds***
  // Source ros.org: http://wiki.ros.org/roscpp/Overview/Time
  ros::Duration(5).sleep(); 

  // ***Hide marker***
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);

  // ***Pause for 5 seconds***
  // Source ros.org: http://wiki.ros.org/roscpp/Overview/Time
  ros::Duration(5).sleep(); 


  // ***Publish the marker at the drop-off zone***

  // Define quaternion
  tf::Quaternion q2 = tf::createQuaternionFromRPY(0, 0, -1.57);   //Create quaternion from yaw = -90 degrees
  q2.normalize();						  //Source ros.org: http://wiki.ros.org/tf2/Tutorials/Quaternions
 
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 2.75;
  marker.pose.position.y = 0.25;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = q2[0];
  marker.pose.orientation.y = q2[1];
  marker.pose.orientation.z = q2[2];
  marker.pose.orientation.w = q2[3];

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);

  // Create an infinite loop to avoid x-term closing immediately after successful goal reached
  // http://wiki.ros.org/roscpp/Overview/Time
  ros::Duration time_between_ros_wakeups(0.001);
  while (ros::ok()) {
       ros::spinOnce();
       time_between_ros_wakeups.sleep();
  }

}

