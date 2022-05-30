#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <aruco_msgs/MarkerArray.h>
const int bufSize = 10;

class MarkerTF{
public:
    MarkerTF(){
        sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1, &MarkerTF::poseCallback,this);
    }
    geometry_msgs::Pose movAvgFilter(geometry_msgs::Pose pose){
        
        for (int i = 0; i < bufSize; i++)
            buf[i] = buf[i + 1];

        buf[bufSize].position.x = pose.position.x;
        buf[bufSize].position.y = pose.position.y;
        buf[bufSize].position.z = pose.position.z;
        buf[bufSize].orientation.x = pose.orientation.x;
        buf[bufSize].orientation.y = pose.orientation.y;
        buf[bufSize].orientation.z = pose.orientation.z;
        buf[bufSize].orientation.w = pose.orientation.w;

        geometry_msgs::Pose tmp;
        tmp.position.x = (pose.position.x - buf[0].position.x)/bufSize;
        tmp.position.y = (pose.position.y - buf[0].position.y)/bufSize;
        tmp.position.z = (pose.position.z - buf[0].position.z)/bufSize;
        tmp.orientation.x = (pose.orientation.x - buf[0].orientation.x)/bufSize;
        tmp.orientation.y = (pose.orientation.y - buf[0].orientation.y)/bufSize;
        tmp.orientation.z = (pose.orientation.z - buf[0].orientation.z)/bufSize;
        tmp.orientation.w = (pose.orientation.w - buf[0].orientation.w)/bufSize;

        avg_.position.x = preAvg_.position.x + tmp.position.x;
        avg_.position.y = preAvg_.position.y + tmp.position.y;
        avg_.position.z = preAvg_.position.z + tmp.position.z;
        avg_.orientation.x = preAvg_.orientation.x + tmp.orientation.x;
        avg_.orientation.y = preAvg_.orientation.y + tmp.orientation.y;
        avg_.orientation.z = preAvg_.orientation.z + tmp.orientation.z;
        avg_.orientation.w = preAvg_.orientation.w + tmp.orientation.w;

        //avg_ = preAvg_ + (tmp) / bufSize;

        preAvg_ = avg_;
        return avg_;
    }
    void poseCallback(const aruco_msgs::MarkerArray& input){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        geometry_msgs::Pose marker_pose;

        tfScalar w;
        if(input.markers.size()!=0 && input.markers[0].id==54){
            marker_pose = input.markers[0].pose.pose;
            // marker_pose = movAvgFilter(marker_pose);

            marker_position_.setX(marker_pose.position.x);
            marker_position_.setY(marker_pose.position.y);
            marker_position_.setZ(marker_pose.position.z);
            
            transform.setOrigin(marker_position_);

            marker_orientation_.setX(marker_pose.orientation.x);
            marker_orientation_.setY(marker_pose.orientation.y);
            marker_orientation_.setZ(marker_pose.orientation.z);
            w = marker_pose.orientation.w;
            q.setValue(marker_pose.orientation.x,marker_pose.orientation.y,marker_pose.orientation.z,w);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "aruco_marker"));
            }
        }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    geometry_msgs::Pose buf[bufSize + 1] = {};
    geometry_msgs::Pose avg_;
    geometry_msgs::Pose preAvg_;
    tf::Vector3 marker_position_;
    tf::Vector3 marker_orientation_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"camera_to_aruco");
    MarkerTF marker;
    ros::spin();
    return 0;
}