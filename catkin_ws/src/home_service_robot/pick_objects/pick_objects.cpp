
// ****************************************************************************
// *********Project: Home Service Robot - Thomas Hatting 29th May 2018*********
// ****************************************************************************

// This C++ script has been developed by means of following: 
// a) Tutorial on ros.org: http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
// b) Udacity Classroom material term 2

//  --------This project has been created on the Jetson TX2-------------------------


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>  


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the node
  ros::init(argc, argv, "pick_objects");  //node name changed to pick_objects

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";  //frame_id changed to map
  goal.target_pose.header.stamp = ros::Time::now();


  // Destination 1: pick-up zone

  // Define quaternion
  tf::Quaternion q1 = tf::createQuaternionFromRPY(0, 0, -0.79);  //Create quaternion from yaw = -45 degrees
  q1.normalize();						 //Source ros.org: http://wiki.ros.org/tf2/Tutorials/Quaternions

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 3.75;
  goal.target_pose.pose.position.y = 4.0;
  goal.target_pose.pose.position.z = 0;  

  goal.target_pose.pose.orientation.x = q1[0];  
  goal.target_pose.pose.orientation.y = q1[1];   
  goal.target_pose.pose.orientation.z = q1[2];   
  goal.target_pose.pose.orientation.w = q1[3];   

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("The robot is travelling to the pick-up zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot picked up the object");
  else
    ROS_INFO("The robot failed to pick up object");

  // Pause for 5 seconds
  // Source ros.org: http://wiki.ros.org/roscpp/Overview/Time
  ros::Duration(5).sleep(); 

  // Destination 2: drop-off zone
  tf::Quaternion q2 = tf::createQuaternionFromRPY(0, 0, -3.14);  //Create quaternion from yaw = -180 degrees
  q2.normalize();						 //Source ros.org: http://wiki.ros.org/tf2/Tutorials/Quaternions

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 2.75;	
  goal.target_pose.pose.position.y = 0.25;	
  goal.target_pose.pose.position.z = 0;		

  goal.target_pose.pose.orientation.x = q2[0];  
  goal.target_pose.pose.orientation.y = q2[1];  
  goal.target_pose.pose.orientation.z = q2[2];  
  goal.target_pose.pose.orientation.w = q2[3];  

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("The robot is travelling to the drop-off zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot dropped off the object");
  else
    ROS_INFO("The robot failed to drop off object");

  // Create an infinite loop to avoid x-term closing immediately after goal completion
  // http://wiki.ros.org/roscpp/Overview/Time
  ros::Duration time_between_ros_wakeups(0.001);
  while (ros::ok()) {
       ros::spinOnce();
       time_between_ros_wakeups.sleep();
  }

  return 0;
}




