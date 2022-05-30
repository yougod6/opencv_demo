#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>


static const std::string OPENCV_WINDOW = "Result Image";
using namespace std;
using namespace cv;
int threshold1=100;
int threshold2=150;

void change_threshold1(int val,void* ){
    threshold1=val;
}

void change_threshold2(int val,void* ){
    threshold2=val;
}


class CannyEdgeDetector
{
public:
    CannyEdgeDetector(): it_ (nh_)
    {
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &CannyEdgeDetector::imageCb, this);
        image_pub_ = it_.advertise("/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~CannyEdgeDetector()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void setLabel(cv_bridge::CvImagePtr& img,const vector<Point>& pts, const String& label){
	       // Rect rc = boundingRect(pts);
	       // rectangle(img->image,rc,Scalar(0,0,255),3);
	       // putText(img->image,label,rc.tl(),FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));
	       RotatedRect rotatedRect = minAreaRect(pts);
	       Point2f points[4];
	       rotatedRect.points(points);
	    //    for(int i=0; i<4; i++)
	    //          line(img->image, points[i], points[(i + 1) % 4], Scalar(0), 4);
	        putText(img->image,label,rotatedRect.center,FONT_HERSHEY_SIMPLEX,2,Scalar(0,0,255),2);
		    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
		        Mat img = cv_ptr->image;

        cv::Mat gray_img;
        cv::Mat dst, detected_edges;

  
         
        cv::cvtColor(cv_ptr->image, gray_img, cv::COLOR_BGR2GRAY);
        cv::blur(gray_img, detected_edges, cv::Size(3,3));
        cv::Canny(detected_edges, detected_edges, threshold1, threshold2);
		
        std::vector<vector<Point>> contours;
        cv::findContours(detected_edges,contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);


        dst = cv::Scalar::all(0);
        cv_ptr->image.copyTo(dst, detected_edges);
        for(vector<Point>& pts : contours){
		            if(contourArea(pts) < 400)
		                continue;
		            vector<Point> approx;
		            approxPolyDP(pts,approx,arcLength(pts,true)*0.02,true);
		
		            int vtc = (int)approx.size();
		
		            if(vtc==4){
		                RotatedRect rotatedRect = minAreaRect(approx);
                        if(rotatedRect.size.area()<920000 && rotatedRect.size.area()>200000){
                            float angle = rotatedRect.angle;
		                    // string msg = to_string(round(angle*10)/10);
		                    string msg = to_string(angle);
		                    if(abs(angle) <2)
		                        msg = "It's OK";
		                    setLabel(cv_ptr,pts,msg);
                        }
		                
		            }
		    
		        }
        namedWindow("EdgeImg");
        createTrackbar("Threshold1","EdgeImg",0,255,change_threshold1);
        createTrackbar("Threshold2","EdgeImg",0,255,change_threshold2);
        cv::imshow("EdgeImg", dst);
        cv::imshow("GrayImg", gray_img);
        cv::imshow(OPENCV_WINDOW, img);
        cv::waitKey(1);

        image_pub_.publish(cv_ptr->toImageMsg());
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "canny_edge_detector");
    CannyEdgeDetector n;
    ros::spin();
    return 0;
}