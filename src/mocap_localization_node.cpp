#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <cmath>
#include <nlopt.hpp>
#include "../include/optitrack/or_pose_estimator_state.h"
#include <tf/tf.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <std_srvs/Empty.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

using namespace std;

class MocapLocalization
{
private:
    ros::NodeHandle node_;
    tf::TransformListener tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;
    ros::Publisher ref_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher joint_pub_;
    //ros::Subscriber mocap_sub_;
    ros::Subscriber offset_sub_;
    geometry_msgs::Pose currentPose_;
    geometry_msgs::Pose originPose_;
    tf::MessageFilter<geometry_msgs::PoseStamped> *tfFilter_;
    
    ros::Subscriber pose_sub_;
    ros::Subscriber reference_sub_;
    ros::Timer loca_timer_;

    tf::Transform tfWorld2Map_; // provided by /tf (tf static broadcaster)
    tf::Transform tfOdom2Footprint_; // provided by /tf (robot state publisher)
    tf::Transform tfWorld2Optitrack_;  // inverse of appart marker
    tf::Transform tfOptitrack2Footprint_; // robot marker
    tf::Transform tfMap2Odom_; // localization to compute
    tf::Transform offsetTf_; // init offset

public:
    MocapLocalization(ros::NodeHandle node) : tfListener_(),
                                              tfBroadcaster_(),
                                              node_(node)
    {
        //Subscribers

        reference_sub_ = node_.subscribe("/optitrack/bodies/reference", 10, &MocapLocalization::updateMocapOrigin,this);

        pose_sub_ = node_.subscribe("/optitrack/bodies/robot", 10, &MocapLocalization::updateMocapPepperPose,this);

        offset_sub_ = node_.subscribe("/initialpose", 1, &MocapLocalization::initPoseReceived,this);
        
        loca_timer_ = node_.createTimer(ros::Duration(0.2), &MocapLocalization::update,this);

        tfWorld2Map_.setIdentity(); // provided by /tf (tf static broadcaster)
        tfOdom2Footprint_.setIdentity(); // provided by /tf (robot state publisher)
        tfWorld2Optitrack_.setIdentity();  // inverse of appart marker
        tfOptitrack2Footprint_.setIdentity(); // pepper marker
        tfMap2Odom_.setIdentity(); // localization to computes
        offsetTf_.setIdentity(); // init offset

        ROS_INFO("[mocap_localization] Motion capture localization ready");
    };
private:

    void update(const ros::TimerEvent&)
    {
        try
        {
            std::string footprint_frame_id, global_frame_id, odom_frame_id, world_frame_id;
            tf::StampedTransform tfOdom2Footprint;
            tf::StampedTransform tfWorld2Map;

            if (!node_.getParam("/global_frame_id", global_frame_id)) {
                global_frame_id="/map";
            }
            if (!node_.getParam("/footprint_frame_id", footprint_frame_id)) {
                footprint_frame_id="/base_footprint";
            }
            if (!node_.getParam("/odom_frame_id", odom_frame_id)) {
                odom_frame_id="/odom";
            }
            if (!node_.getParam("/world_frame_id", world_frame_id)) {
                world_frame_id="/world";
            }

            if (tfListener_.canTransform(world_frame_id, global_frame_id,  ros::Time(0))) {
                tfListener_.lookupTransform(world_frame_id, global_frame_id, ros::Time(0), tfWorld2Map);
                tfWorld2Map_.setOrigin(tfWorld2Map.getOrigin());
                tfWorld2Map_.setRotation(tfWorld2Map.getRotation());
            }
            
            if (tfListener_.canTransform(odom_frame_id, footprint_frame_id, ros::Time(0))) {
                tfListener_.lookupTransform(odom_frame_id, footprint_frame_id, ros::Time(0), tfOdom2Footprint);
                tfOdom2Footprint_.setOrigin(tfOdom2Footprint.getOrigin());
                tfOdom2Footprint_.setRotation(tfOdom2Footprint.getRotation());
            }
            
            tfBroadcaster_.sendTransform(tf::StampedTransform((tfOptitrack2Footprint_), ros::Time::now(), "optitrack", "base_footprint_ground_truth"));
            
            tfBroadcaster_.sendTransform(tf::StampedTransform((tfWorld2Optitrack_), ros::Time::now(), "world", "optitrack"));


            tfMap2Odom_ = tfWorld2Map_.inverse() * tfWorld2Optitrack_ * tfOptitrack2Footprint_ * tfOdom2Footprint_.inverse();

            tfBroadcaster_.sendTransform(tf::StampedTransform(tfMap2Odom_* offsetTf_, ros::Time::now(), "map", "odom"));
            
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR("[mocap_localization] Error occur : %s\n", ex.what());
        }
    }

    void updateMocapPepperPose(const optitrack::or_pose_estimator_state::ConstPtr &msg)
    {
        geometry_msgs::Pose pose_received;
        tf::Transform tf_received; 
        if(!msg->pos.empty())
        {
            //Setting position
            pose_received.position.x=msg->pos[0].x;
            pose_received.position.y=msg->pos[0].y;
            pose_received.position.z=msg->pos[0].z;
            //Setting orientation
            pose_received.orientation.x=msg->pos[0].qx;
            pose_received.orientation.y=msg->pos[0].qy;
            pose_received.orientation.z=msg->pos[0].qz;
            pose_received.orientation.w=msg->pos[0].qw;

            tf::poseMsgToTF(pose_received, tf_received);

            tfOptitrack2Footprint_.setOrigin(tf_received.getOrigin());
            tfOptitrack2Footprint_.setRotation(tf_received.getRotation());
        }
    };

    void updateMocapOrigin(const optitrack::or_pose_estimator_state::ConstPtr &msg)
    {
        geometry_msgs::Pose pose_received;
        tf::Transform tf_received;
        if(!msg->pos.empty())
        {
            //Setting position
            pose_received.position.x=msg->pos[0].x;
            pose_received.position.y=msg->pos[0].y;
            pose_received.position.z=msg->pos[0].z;
            //Setting orientation
            pose_received.orientation.x=msg->pos[0].qx;
            pose_received.orientation.y=msg->pos[0].qy;
            pose_received.orientation.z=msg->pos[0].qz;
            pose_received.orientation.w=msg->pos[0].qw;

            tf::poseMsgToTF(pose_received, tf_received);

            tf_received = tf_received.inverse();

            tfWorld2Optitrack_.setOrigin(tf_received.getOrigin());
            tfWorld2Optitrack_.setRotation(tf_received.getRotation());
        }
    };

    void initPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {
        tf::Pose pose;
        tf::poseMsgToTF(msg->pose.pose, pose);
        tf::StampedTransform tfOdom2Base;
        tf::StampedTransform tfBaseInMap;
        std::string footprint_frame_id, global_frame_id;
        tfOdom2Base.setIdentity();
        
        if (!node_.getParam("/global_frame_id", global_frame_id)){
            global_frame_id="/map";
        }
        if (!node_.getParam("/footprint_frame_id", footprint_frame_id)) {
            footprint_frame_id="/base_footprint";
        }
        if (tfListener_.canTransform(footprint_frame_id, global_frame_id, ros::Time(0)))
            tfListener_.lookupTransform(footprint_frame_id, global_frame_id, ros::Time(0), tfBaseInMap);
        
        tf::Transform delta;
        delta = pose * tfBaseInMap;
        offsetTf_ = delta * offsetTf_;
    };

};

int main(int argc, char** argv){
  ros::init(argc, argv, "mocap_localization");
  ros::NodeHandle node;
  MocapLocalization mcl(node);
  ros::spin();
  return 0;
};
