#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "std_msgs/Float64.h"

const _Float64 RAD2REV = 0.1591549431;
const _Float32 IS_ALINGED = 0.1;

class Marker{
public:
    Marker(){
        sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&Marker::callback,this);
        pub_ = nh_.advertise<std_msgs::Float64>("/error_revolution",1);
    }

    void callback(const aruco_msgs::MarkerArray& input){
        if(input.markers.size()!=0 && input.markers[0].id==54){
            tf::Quaternion marker_orientation;
            marker_orientation.setX(input.markers[0].pose.pose.orientation.x);
            marker_orientation.setY(input.markers[0].pose.pose.orientation.y);
            marker_orientation.setZ(input.markers[0].pose.pose.orientation.z);
            marker_orientation.setW(input.markers[0].pose.pose.orientation.w);
            tf::Matrix3x3 matrix(marker_orientation);
            // double roll, pitch, yaw;
            tfScalar roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            ROS_INFO_STREAM("yaw : "<<yaw<<" rad");
            rev_.data = yaw*RAD2REV;
            ROS_INFO_STREAM("rev : "<<rev_);
            if(abs(yaw)>IS_ALINGED){
                pub_.publish(rev_);
            }
            else{
                rev_.data = 0;
                pub_.publish(rev_);
            }
        }
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std_msgs::Float64 rev_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"girpper_alignment");
    Marker marker;
    ros::spin();
    
    return 0;
}
