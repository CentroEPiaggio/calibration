// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>

#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/thread/thread.hpp>

namespace calibration {

class AverageCalibrator
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
        std::string child_frame_id_;
        std::string ar_marker_frame_;
        std::string calibrator_frame_;
        
        std::string calibration_name_;

        int samples_; //how many samples to keep
  
        //vectors to store published translation and rotations (in axis angle, to avoid averaging quaternions)
        std::vector<double> kept_translation_x_first_;
        std::vector<double> kept_translation_y_first_;
        std::vector<double> kept_translation_z_first_;
        std::vector<double> kept_rot_axis_x_first_;
        std::vector<double> kept_rot_axis_y_first_;
        std::vector<double> kept_rot_axis_z_first_;
        std::vector<double> kept_rot_angle_first_;
        std::vector<double> kept_translation_x_second_;
        std::vector<double> kept_translation_y_second_;
        std::vector<double> kept_translation_z_second_;
        std::vector<double> kept_rot_axis_x_second_;
        std::vector<double> kept_rot_axis_y_second_;
        std::vector<double> kept_rot_axis_z_second_;
        std::vector<double> kept_rot_angle_second_;
        //translation and rotation to be published (as the average)
        std::vector<double> published_translation_;
        std::vector<double> published_rotation_;

    public:
        bool calibrate(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        void publishTf();
        void recordTf();

        // constructor
        AverageCalibrator(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
        {

            // load the parameters
            nh_.param<std::vector<double> >("translation", published_translation_, {0, 0, 0});
            nh_.param<std::vector<double> >("rotation", published_rotation_, {0, 0, 0, 1});
            nh_.param<std::string>("frame_id", frame_id_, "/camera_depth_optical_frame");
            nh_.param<std::string>("ar_marker_frame", ar_marker_frame_, "/ar_marker_60");
            nh_.param<std::string>("child_frame_id", child_frame_id_, "/phase_space_world");
            nh_.param<std::string>("calibrator_frame", calibrator_frame_, "/calibrator");
            nh_.param<std::string>("calibration_name", calibration_name_, "asus_phase_space");
            
            samples_ = 50;
            kept_translation_x_first_.resize(samples_, 0);
            kept_translation_y_first_.resize(samples_, 0);
            kept_translation_z_first_.resize(samples_, 0);
            kept_rot_axis_x_first_.resize(samples_, 0);
            kept_rot_axis_y_first_.resize(samples_, 0);
            kept_rot_axis_z_first_.resize(samples_, 0);
            kept_rot_angle_first_.resize(samples_, 0);
            kept_translation_x_second_.resize(samples_, 0);
            kept_translation_y_second_.resize(samples_, 0);
            kept_translation_z_second_.resize(samples_, 0);
            kept_rot_axis_x_second_.resize(samples_, 0);
            kept_rot_axis_y_second_.resize(samples_, 0);
            kept_rot_axis_z_second_.resize(samples_, 0);
            kept_rot_angle_second_.resize(samples_, 0);

            // init the calibration to the identity to publish something
            transform_.setOrigin( tf::Vector3( published_translation_.at(0), published_translation_.at(1), published_translation_.at(2) ) );
            transform_.setRotation( tf::Quaternion( published_rotation_.at(0), published_rotation_.at(1), published_rotation_.at(2), published_rotation_.at(3) ) );

            // advertise service
            srv_calibrate_ = nh_.advertiseService(nh_.resolveName("calibrate"), &AverageCalibrator::calibrate, this);
        }

        //! Empty stub
        ~AverageCalibrator() {}

};
void AverageCalibrator::recordTf()
{
    // 1. get the first transformation
    tf::StampedTransform first_transformation;
    try
    {
      tf_listener_.lookupTransform(frame_id_, ar_marker_frame_,ros::Time(0), first_transformation);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // 2. get the second transformation
    tf::StampedTransform second_transformation;
    try
    {
      tf_listener_.lookupTransform(child_frame_id_, calibrator_frame_,ros::Time(0), second_transformation);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    //store transforms
    //erase first element
    kept_translation_x_first_.erase(kept_translation_x_first_.begin());
    kept_translation_y_first_.erase(kept_translation_y_first_.begin());
    kept_translation_z_first_.erase(kept_translation_z_first_.begin());
    kept_rot_axis_x_first_.erase(kept_rot_axis_x_first_.begin());     
    kept_rot_axis_y_first_.erase(kept_rot_axis_y_first_.begin());     
    kept_rot_axis_z_first_.erase(kept_rot_axis_z_first_.begin());
    kept_rot_angle_first_.erase(kept_rot_angle_first_.begin());     
    kept_translation_x_second_.erase(kept_translation_x_second_.begin());
    kept_translation_y_second_.erase(kept_translation_y_second_.begin());
    kept_translation_z_second_.erase(kept_translation_z_second_.begin());
    kept_rot_axis_x_second_.erase(kept_rot_axis_x_second_.begin());     
    kept_rot_axis_y_second_.erase(kept_rot_axis_y_second_.begin());     
    kept_rot_axis_z_second_.erase(kept_rot_axis_z_second_.begin());
    kept_rot_angle_second_.erase(kept_rot_angle_second_.begin());     

    //populate by pushing back size is now again == samples_
    kept_translation_x_first_.push_back(first_transformation.getOrigin()[0]);
    kept_translation_y_first_.push_back(first_transformation.getOrigin()[1]);
    kept_translation_z_first_.push_back(first_transformation.getOrigin()[2]);
    Eigen::AngleAxisd first_ax ( Eigen::Quaterniond(first_transformation.getRotation().getW(), first_transformation.getRotation().getX(), 
          first_transformation.getRotation().getY(), first_transformation.getRotation().getZ()));
    kept_rot_axis_x_first_.push_back(first_ax.axis()[0]);     
    kept_rot_axis_y_first_.push_back(first_ax.axis()[1]);
    kept_rot_axis_z_first_.push_back(first_ax.axis()[2]);
    kept_rot_angle_first_.push_back(first_ax.angle());
    kept_translation_x_second_.push_back(second_transformation.getOrigin()[0]);
    kept_translation_y_second_.push_back(second_transformation.getOrigin()[1]);
    kept_translation_z_second_.push_back(second_transformation.getOrigin()[2]);
    Eigen::AngleAxisd second_ax ( Eigen::Quaterniond(second_transformation.getRotation().getW(), second_transformation.getRotation().getX(), 
          second_transformation.getRotation().getY(), second_transformation.getRotation().getZ()));
    kept_rot_axis_x_second_.push_back(second_ax.axis()[0]);
    kept_rot_axis_y_second_.push_back(second_ax.axis()[1]);
    kept_rot_axis_z_second_.push_back(second_ax.axis()[2]);
    kept_rot_angle_second_.push_back(second_ax.angle());

}

bool AverageCalibrator::calibrate( std_srvs::Empty::Request &request, std_srvs::Empty::Response &response )
{
    tf::Transform first_tran, second_tran;
    // 1. get the frist transformation as the average of those in memory
    double tx(0),ty(0),tz(0),ax(0),ay(0),az(0),angle(0);
    for (int i =0; i<samples_; ++i)
    {
      tx += kept_translation_x_first_[i];
      ty += kept_translation_y_first_[i];
      tz += kept_translation_z_first_[i];
      ax += kept_rot_axis_x_first_[i];
      ay += kept_rot_axis_y_first_[i];
      az += kept_rot_axis_z_first_[i];
      angle += kept_rot_angle_first_[i];
    }
    tx /= samples_;
    ty /= samples_;
    tz /= samples_;
    ax /= samples_;
    ay /= samples_;
    az /= samples_;
    angle /= samples_;
            
    first_tran.setOrigin( tf::Vector3( tx,ty,tz) );
    Eigen::Vector3d axis (ax,ay,az);
    axis.normalize();
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
    first_tran.setRotation( tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

    // 2. get the second transformation as the average f those in memory
    tx=ty=tz=ax=ay=az=angle=0;
    for (int i =0; i<samples_; ++i)
    {
      tx += kept_translation_x_second_[i];
      ty += kept_translation_y_second_[i];
      tz += kept_translation_z_second_[i];
      ax += kept_rot_axis_x_second_[i];
      ay += kept_rot_axis_y_second_[i];
      az += kept_rot_axis_z_second_[i];
      angle += kept_rot_angle_second_[i];
    }
    tx /= samples_;
    ty /= samples_;
    tz /= samples_;
    ax /= samples_;
    ay /= samples_;
    az /= samples_;
    angle /= samples_;
            
    second_tran.setOrigin( tf::Vector3( tx,ty,tz) );
    Eigen::Vector3d axis2 (ax,ay,az);
    axis2.normalize();
    Eigen::Quaterniond q2(Eigen::AngleAxisd(angle, axis2));
    second_tran.setRotation( tf::Quaternion(q2.x(), q2.y(), q2.z(), q2.w()));

    // 3. since the reference frame is the same, we just need to inverse and multiply
    transform_ = first_tran*second_tran.inverse();

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
      f.close();
    }
    return true;
}

// this function is called as fast as ROS can from the main loop directly
void AverageCalibrator::publishTf()
{
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), frame_id_, child_frame_id_)); //from asus to phase space
}

} // namespace calibration

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_calibrator_node");
    ros::NodeHandle nh;

    calibration::AverageCalibrator node(nh);
    ros::Rate rate(10.0); //go at 100Hz
  
    //wait a bit to let tf start publishing
    //boost::this_thread::sleep (boost::posix_time::microseconds (3000000));
    usleep(3000000);
    while(nh.ok())
    {
      node.recordTf();
      node.publishTf();
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
