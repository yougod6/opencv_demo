#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"


class Marker{
public:
    Marker(){
        sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&Marker::callback,this);
        pub_ = nh_.advertise<geometry_msgs::Pose>("/marker_pose",1);
    }

    void callback(const aruco_msgs::MarkerArray& input){
        if(input.markers.size()!=0 && input.markers[0].id==54){
            geometry_msgs::Pose marker_pose;
            marker_pose = input.markers[0].pose.pose;

            pub_.publish(marker_pose);
            }
        }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"marker_pose_pub");
    Marker marker;
    ros::spin();
    
    return 0;
}