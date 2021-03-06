// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

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

        std::string frame_id_;
        std::string ar_marker_frame_;
        std::string child_frame_id_;
        std::string calibrator_frame_;
        std::string calibration_name_;
        std::string obj_name;

        std::vector<double> translation_;
        std::vector<double> rotation_;
        bool pub_markers;
        //publisher to broadcast obj mesh during obj/tracker calib
        ros::Publisher rviz_marker_pub;
        bool start_broadcast;

    public:
        bool calibrate(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        void publishTf();

        // constructor
        SimpleCalibrator(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
        {
            // load the parameters
            nh_.param<std::vector<double> >("translation", translation_, {0, 0, 0});
            nh_.param<std::vector<double> >("rotation", rotation_, {0, 0, 0, 1});
            nh_.param<std::string>("frame_id", frame_id_, "/camera_rgb_optical_frame");
            nh_.param<std::string>("ar_marker_frame", ar_marker_frame_, "/ar_marker_60");
            nh_.param<std::string>("child_frame_id", child_frame_id_, "/phase_space_world");
            nh_.param<std::string>("calibrator_frame", calibrator_frame_, "/calibrator");
            nh_.param<std::string>("calibration_name", calibration_name_, "asus_phase_space");
            nh_.param<std::string>("tracked_object", obj_name, "object");
            nh_.param<bool>("start_broadcast", start_broadcast, false);
            if (obj_name.compare("object") != 0)
            {
              pub_markers=true;
              rviz_marker_pub = nh_.advertise<visualization_msgs::Marker>("object", 1);
            }
            else 
              pub_markers=false;
            
            // init the calibration to the identity to publish something
            transform_.setOrigin( tf::Vector3( translation_.at(0), translation_.at(1), translation_.at(2) ) );
            transform_.setRotation( tf::Quaternion( rotation_.at(0), rotation_.at(1), rotation_.at(2), rotation_.at(3) ) );

            // advertise service
            srv_calibrate_ = nh_.advertiseService(nh_.resolveName("calibrate"), &SimpleCalibrator::calibrate, this);
        }

        //! Empty stub
        ~SimpleCalibrator() {}

};

bool SimpleCalibrator::calibrate( std_srvs::Empty::Request &request, std_srvs::Empty::Response &response )
{
    nh_.setParam("start_broadcast", false);

    // 1. get the ar marker frame in the asus frame
    tf::StampedTransform ar_marker_frame;
    try{
        tf_listener_.lookupTransform(frame_id_, ar_marker_frame_,  
                               ros::Time(0), ar_marker_frame);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // 2. get the calibrator object in the phase space frame
    tf::StampedTransform calibrator_frame;
    try{
        tf_listener_.lookupTransform(child_frame_id_, calibrator_frame_,  
                               ros::Time(0), calibrator_frame);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // 3. since the reference frame is the same, we just need to inverse and multiply
    transform_ = ar_marker_frame*calibrator_frame.inverse();

    // 4. write the transform on a yaml file
    std::string path = ros::package::getPath("calibration");
    std::string file = path + "/config/" + calibration_name_ + ".yaml";
    //std::cout << file.c_str() << std::endl;
    std::ofstream f;
    f.open(file.c_str());
    if (f.is_open())
    {
      f << "# This file contains the values obtained by the calibration package" << std::endl;
      f << "# Results are written in the form that they can be directly sent to a static_transform_publisher node" << std::endl;
      f << "# Recall its usage use static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds)" << std::endl;
      f << "translation: [" << transform_.getOrigin()[0] << ", " << 
                          transform_.getOrigin()[1] << ", " << 
                          transform_.getOrigin()[2]<<"]" << std::endl;
      f << "rotation: [" << transform_.getRotation().getX() << ", " << 
                            transform_.getRotation().getY() << ", " << 
                            transform_.getRotation().getZ() << ", " << 
                            transform_.getRotation().getW() << "]" << std::endl;
      f << "frame_id: " << frame_id_.c_str() << std::endl;
      f << "child_frame_id: " << child_frame_id_.c_str() << std::endl;
      if (pub_markers)
        f << "tracked_object: " << obj_name.c_str() << std::endl;
      f.close();
    }
    nh_.setParam("start_broadcast", true);
    return true;
}

// this function is called as fast as ROS can from the main loop directly
void SimpleCalibrator::publishTf()
{
  nh_.getParam("start_broadcast", start_broadcast);
  if (start_broadcast)
  {
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), frame_id_, child_frame_id_)); //from asus to phase space
    if (pub_markers)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/star";
      marker.header.stamp = ros::Time();
      marker.ns=obj_name.c_str();
      marker.id=0;
      marker.scale.x=1;
      marker.scale.y=1;
      marker.scale.z=1;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      std::string mesh_path ("package://asus_scanner_models/" + obj_name + "/" + obj_name + ".stl");
      marker.mesh_resource = mesh_path.c_str();
      marker.action = visualization_msgs::Marker::ADD;
      geometry_msgs::Pose pose;
      pose.position.x = transform_.getOrigin()[0];
      pose.position.y = transform_.getOrigin()[1];
      pose.position.z = transform_.getOrigin()[2];
      pose.orientation.x = transform_.getRotation().getX();
      pose.orientation.y = transform_.getRotation().getY();
      pose.orientation.z = transform_.getRotation().getZ();
      pose.orientation.w = transform_.getRotation().getW();
      marker.pose = pose;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.lifetime = ros::Duration(1);
      rviz_marker_pub.publish(marker);
    }
  }
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
