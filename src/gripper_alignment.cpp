#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "std_msgs/Bool.h"


class Marker{
public:
    Marker(){
        is_alligned_.data = false;
        sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&Marker::callback,this);
        pub_ = nh_.advertise<std_msgs::Bool>("/is_girpper_aligned",1);
    }

    void callback(const aruco_msgs::MarkerArray& input){
        if(input.markers.size()!=0 && input.markers[0].id==54){
            tf::Quaternion marker_orientation;
            marker_orientation.setX(input.markers[0].pose.pose.orientation.x);
            marker_orientation.setY(input.markers[0].pose.pose.orientation.x);
            marker_orientation.setZ(input.markers[0].pose.pose.orientation.x);
            marker_orientation.setW(input.markers[0].pose.pose.orientation.x);
            tf::Matrix3x3 matrix(marker_orientation);
            // double roll, pitch, yaw;
            tfScalar roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            ROS_INFO_STREAM(yaw);
            if(abs(yaw)<0.1){
                is_alligned_.data = true;
                pub_.publish(is_alligned_);
            }
            else{
                is_alligned_.data = false;
                pub_.publish(is_alligned_);
            }
        
        }
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std_msgs::Bool is_alligned_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"girpper_alignment");
    Marker marker;
    ros::spin();
    
    return 0;
}
