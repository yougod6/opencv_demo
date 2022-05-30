#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <aruco_msgs/MarkerArray.h>
const int bufSize = 10;

class MarkerTF{
public:
    MarkerTF(){
        sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&MarkerTF::poseCallback,this);
    }
    tf::Vector3 movAvgFilter(tf::Vector3 vector){
        
        for (int i = 0; i < bufSize; i++)
            buf[i] = buf[i + 1];

        buf[bufSize] = vector;

        avg_ = preAvg_ + (vector - buf[0]) / bufSize;

        preAvg_ = avg_;
        return avg_;
    }
    void poseCallback(const aruco_msgs::MarkerArray& input){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        geometry_msgs::Pose marker_pose;
        tf::Vector3 marker_position;
        tf::Vector3 marker_orientation;
        tfScalar w;
        if(input.markers.size()!=0 && input.markers[0].id==54){
            marker_pose = input.markers[0].pose.pose;


            marker_position.setX(marker_pose.position.x);
            marker_position.setY(marker_pose.position.y);
            marker_position.setZ(marker_pose.position.z);
            marker_position = movAvgFilter(marker_position);
            transform.setOrigin(marker_position);

            marker_orientation.setX(marker_pose.orientation.x);
            marker_orientation.setY(marker_pose.orientation.y);
            marker_orientation.setZ(marker_pose.orientation.z);
            marker_orientation = movAvgFilter(marker_orientation);
            w = marker_pose.orientation.w;
            q.setValue(marker_pose.orientation.x,marker_pose.orientation.y,marker_pose.orientation.z,w);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "aruco_marker"));
            }
        }
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf::Vector3 buf[bufSize + 1] = {};
    tf::Vector3 avg_;
    tf::Vector3 preAvg_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"camera_to_aruco");
    MarkerTF marker;
    ros::spin();
    return 0;
}