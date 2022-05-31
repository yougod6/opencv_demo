#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"
#include <geometry_msgs/Quaternion.h>


class Marker{
public:
    Marker(){
        sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&Marker::callback,this);
        pub_ = nh_.advertise<bool>("/is_girpper_aligned",1);
    }

    void callback(const aruco_msgs::MarkerArray& input){
        if(input.markers.size()!=0 && input.markers[0].id==54){
            geometry_msgs::Quaternion marker_orientation;
            marker_orientation = input.markers[0].pose.pose.orientation;
            tf::Quaternion q(marker_orientation);
            tf::Matrix3x3 m(q);
            // double roll, pitch, yaw;
            tfScalar roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            if(abs(yaw)<0.1){
                is_alligned_ = true;
                pub(is_alligned_);
            }
            else{
                is_alligned_ = false;
                pub(is_alligned_);
            }
        
        }
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    bool is_alligned_=false;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"girpper_alignment");
    Marker marker;
    ros::spin();
    
    return 0;
}