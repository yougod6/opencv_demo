#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Pose.h"
#include "aruco_msgs/MarkerArray.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "end_effector_listener");

  ros::NodeHandle node;
  ros::Publisher end_pose =
    node.advertise<geometry_msgs::Pose>("end_effector_pose", 1);

  tf::TransformListener listener; //TransformListener 

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform1;
    tf::StampedTransform transform2;
    try{
      listener.waitForTransform("aruco_marker", "camera_link", ros::Time(0),ros::Duration(10.0));
      listener.waitForTransform("end_effector", "aruco_marker", ros::Time(0),ros::Duration(10.0));
      listener.lookupTransform("aruco_marker", "camera_link",
                               ros::Time(0), transform1);
      listener.lookupTransform("end_effector", "aruco_marker",
                               ros::Time(0), transform2);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Pose pose_msg;
    tf::StampedTransform transform;
    transform = transform1;
    transform.mult(transform1,transform2);
    pose_msg.position.x = transform.getOrigin().x();
    pose_msg.position.y = transform.getOrigin().y();
    pose_msg.position.z = transform.getOrigin().z();
    pose_msg.orientation.x = transform.getRotation().x();
    pose_msg.orientation.y = transform.getRotation().y();
    pose_msg.orientation.z = transform.getRotation().z();
    pose_msg.orientation.w = transform.getRotation().w();

    end_pose.publish(pose_msg);

    rate.sleep();
  }
  return 0;
};