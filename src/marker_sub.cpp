#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"

class Marker{
public:
    Marker(){
        sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&+allback,this);
    }

    void callback(const aruco_msgs::MarkerArray& input){
        if(input.markers.size()!=0 && input.markers[0].id==54){
                ROS_INFO("x = %f",input.markers[0].pose.pose.position.x);
                ROS_INFO("y = %f",input.markers[0].pose.pose.position.y);
                ROS_INFO("z = %f",input.markers[0].pose.pose.position.z);
            }
        }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"marker_sub");
    Marker marker;
    ros::spin();
    
    return 0;
}