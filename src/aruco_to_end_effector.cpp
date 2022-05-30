#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <aruco_msgs/MarkerArray.h>

class MarkerTF{
public:
    MarkerTF(){
        sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&MarkerTF::poseCallback,this);
    }

    void poseCallback(const aruco_msgs::MarkerArray& input){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        geometry_msgs::Pose marker_pose;
        tf::Vector3 marker_position;
        tf::Vector3 marker_orientation_vector;
        tfScalar w;
 
        transform.setOrigin(tf::Vector3(0.275,0.125,0.0));

        q.setRPY(0,0,0);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "aruco_marker", "end_effector"));
        }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"aruco_to_end_effector");
    MarkerTF marker;
    ros::spin();
    return 0;
}