// ROS headers
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Eigen>

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

    public:
        bool calibrate(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        void publishTf();

        // constructor
        SimpleCalibrator(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
        {
            // init the calibration to the identity to publish something
            transform_.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
            tf::Quaternion q;
            q.setRPY( 0.0, 0.0, 0.0 );
            transform_.setRotation( q );

            // load the parameters
            nh_.param<std::string>("asus_frame", asus_frame_, "/camera_rgb_optical_frame");
            nh_.param<std::string>("ar_marker_frame", ar_marker_frame_, "/ar_marker_60");
            nh_.param<std::string>("phase_space_frame", phase_space_frame_, "/world");
            nh_.param<std::string>("object_frame", object_frame_, "/calibrator");

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
}

// this function is called as fast as ROS can from the main loop directly
void SimpleCalibrator::publishTf()
{
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), asus_frame_, phase_space_frame_));
}

} // namespace calibration

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_calibrator_node");
    ros::NodeHandle nh;

    calibration::SimpleCalibrator node(nh);

    while(ros::ok())
    {
        node.publishTf();
        ros::spinOnce();
    }

    return 0;
}
