// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Eigen>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace calibration {

class SimpleCalibrator
{
    private:
        // node handles
        ros::NodeHandle nh_;
        ros::NodeHandle priv_nh_;

        // services
        ros::ServiceServer srv_calibrate_;

        // it is very useful to have a listener and broadcaster to know where all frames are
        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster tf_broadcaster_;

        // the looked transform
        tf::Transform transform_;

        std::string asus_frame_;
        std::string ar_marker_frame_;
        std::string phase_space_frame_;
        std::string object_frame_;

        std::vector<double> kps_translation_;
        std::vector<double> kps_rotation_;

    public:
        bool calibrate(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        void publishTf();

        // constructor
        SimpleCalibrator(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
        {

            // load the parameters
            nh_.param<std::vector<double> >("kps_translation", kps_translation_, {0, 0, 0});
            nh_.param<std::vector<double> >("kps_rotation", kps_rotation_, {0, 0, 0, 1});
            nh_.param<std::string>("asus_frame", asus_frame_, "/camera_depth_optical_frame");
            nh_.param<std::string>("ar_marker_frame", ar_marker_frame_, "/ar_marker_60");
            nh_.param<std::string>("phase_space_frame", phase_space_frame_, "/phase_space_world");
            nh_.param<std::string>("object_frame", object_frame_, "/calibrator");
            
            // init the calibration to the identity to publish something
            transform_.setOrigin( tf::Vector3( kps_translation_.at(0), kps_translation_.at(1), kps_translation_.at(2) ) );
            transform_.setRotation( tf::Quaternion( kps_rotation_.at(0), kps_rotation_.at(1), kps_rotation_.at(2), kps_rotation_.at(3) ) );

            // advertise service
            srv_calibrate_ = nh_.advertiseService(nh_.resolveName("calibrate"), &SimpleCalibrator::calibrate, this);
        }

        //! Empty stub
        ~SimpleCalibrator() {}

};

bool SimpleCalibrator::calibrate( std_srvs::Empty::Request &request, std_srvs::Empty::Response &response )
{

    // 1. get the ar marker frame in the asus frame
    tf::StampedTransform ar_marker_frame;
    try{
        tf_listener_.lookupTransform(asus_frame_, ar_marker_frame_,  
                               ros::Time(0), ar_marker_frame);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // 2. get the calibrator object in the phase space frame
    tf::StampedTransform calibrator_frame;
    try{
        tf_listener_.lookupTransform(phase_space_frame_, object_frame_,  
                               ros::Time(0), calibrator_frame);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // 3. since the reference frame is the same, we just need to inverse and multiply
    transform_ = ar_marker_frame*calibrator_frame.inverse();

    //write this transform to a yaml file
    std::string path = ros::package::getPath("calibration");
    std::string file = path + "/config/kinect_phasespace.yaml";
    std::cout<<file.c_str()<<std::endl;
    std::ofstream f;
    f.open(file.c_str());
    if (f.is_open())
    {
      f << "kps_translation: ["<< transform_.getOrigin()[0]<<", "<< transform_.getOrigin()[1]<<", "<<transform_.getOrigin()[2]<<"]"
        <<std::endl;
      f << "kps_rotation: ["<<transform_.getRotation().getX()<<", "<<transform_.getRotation().getY()<<", "<<transform_.getRotation().getZ()<<", "<<transform_.getRotation().getW()<<"]"
        <<std::endl;
      f.close();
    }
}

// this function is called as fast as ROS can from the main loop directly
void SimpleCalibrator::publishTf()
{
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), asus_frame_, phase_space_frame_)); //from asus to phase space
}

} // namespace calibration

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_calibrator_node");
    ros::NodeHandle nh;

    calibration::SimpleCalibrator node(nh);
    ros::Rate rate(100.0); //go at 100Hz

    while(nh.ok())
    {
        node.publishTf();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
