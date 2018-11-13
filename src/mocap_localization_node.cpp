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

#include <mocap_localization/MocapLocalizationConfig.h>

#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/ReconfigureResponse.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>

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

    tf::Transform tfChessboardImage2Footprint_, tfChessboardMocap2FootprintGt_;

    tf::Transform tfWorld2Map_; // provided by /tf (tf static broadcaster)
    tf::Transform tfOdom2Footprint_; // provided by /tf (robot state publisher)
    tf::Transform tfWorld2Optitrack_;  // inverse of appart marker
    tf::Transform tfOptitrack2Footprint_; // robot marker
    tf::Transform tfMap2Odom_; // localization to compute
    tf::Transform offsetBaseGT2BaseFootprint_; // init offset base_gt to base_footprint
    tf::Transform tempOffset_;

    dynamic_reconfigure::Server<mocap_localization::MocapLocalizationConfig> dynCfgServer;
    //dynamic_reconfigure::Client<mocap_localization::MocapLocalizationConfig> dynCfgClient_;

public:
    MocapLocalization(ros::NodeHandle node)
            : tfListener_(),
              tfBroadcaster_(),
              node_(node){
        //Subscribers

        reference_sub_ = node_.subscribe("/optitrack/bodies/world", 10, &MocapLocalization::updateMocapOrigin,this);

        pose_sub_ = node_.subscribe("/optitrack/bodies/robot", 10, &MocapLocalization::updateMocapPepperPose,this);

        offset_sub_ = node_.subscribe("/initialpose", 1, &MocapLocalization::initPoseReceived,this);
        
        loca_timer_ = node_.createTimer(ros::Duration(1/30.0), &MocapLocalization::update,this);

        tfWorld2Map_.setIdentity(); // provided by /tf (tf static broadcaster)
        tfOdom2Footprint_.setIdentity(); // provided by /tf (robot state publisher)
        tfWorld2Optitrack_.setIdentity();  // inverse of appart marker
        tfOptitrack2Footprint_.setIdentity(); // pepper marker
        tfMap2Odom_.setIdentity(); // localization to computes
        offsetBaseGT2BaseFootprint_.setIdentity(); // init offset

        dynamic_reconfigure::Server<mocap_localization::MocapLocalizationConfig>::CallbackType f;

        f = boost::bind(&MocapLocalization::onNewReconfigure, this, _1, _2);
        dynCfgServer.setCallback(f);

        ROS_INFO("[mocap_localization] Motion capture localization ready");
    };
private:

    void update(const ros::TimerEvent&)
    {
        try
        {
            std::string footprint_frame_id, global_frame_id, odom_frame_id, world_frame_id;
            tf::StampedTransform tfOdom2Footprint, tfWorld2Map, tfChessboardImage2Footprint, tfChessboardMocap2FootprintGt, offsetBaseGT2BaseFootprint;

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

//            if (tfListener_.canTransform("chessboard_image_frame", footprint_frame_id, ros::Time(0))) {
//                tfListener_.lookupTransform("chessboard_image_frame", footprint_frame_id, ros::Time(0), tfChessboardImage2Footprint);
//                tfChessboardImage2Footprint_.setOrigin(tfChessboardImage2Footprint.getOrigin());
//                tfChessboardImage2Footprint_.setRotation(tfChessboardImage2Footprint.getRotation());
//                ROS_INFO("chess2footprint %f %f %f", tfChessboardImage2Footprint_.getOrigin().x(), tfChessboardImage2Footprint_.getOrigin().y(), tfChessboardImage2Footprint_.getOrigin().z());
//
//                if (tfListener_.canTransform(world_frame_id, "base_footprint_ground_truth", ros::Time(0))) {
//                    tfListener_.lookupTransform(world_frame_id, "base_footprint_ground_truth", ros::Time(0), tfChessboardMocap2FootprintGt);
//                    tfChessboardMocap2FootprintGt_.setOrigin(tfChessboardMocap2FootprintGt.getOrigin());
//                    tfChessboardMocap2FootprintGt_.setRotation(tfChessboardMocap2FootprintGt.getRotation());
//                    ROS_INFO("chess2mocapGT %f %f %f", tfChessboardMocap2FootprintGt_.getOrigin().x(), tfChessboardMocap2FootprintGt_.getOrigin().y(), tfChessboardMocap2FootprintGt_.getOrigin().z());
//                    tempOffset_ = tfChessboardMocap2FootprintGt_ * tfChessboardImage2Footprint_.inverse();
//                    offsetBaseGT2BaseFootprint_.setOrigin(tempOffset_.getOrigin());
//                }
//
//            }
            
            if (tfListener_.canTransform(odom_frame_id, footprint_frame_id, ros::Time(0))) {
                tfListener_.lookupTransform(odom_frame_id, footprint_frame_id, ros::Time(0), tfOdom2Footprint);
                tfOdom2Footprint_.setOrigin(tfOdom2Footprint.getOrigin());
                tfOdom2Footprint_.setRotation(tfOdom2Footprint.getRotation());
            }

            if (tfListener_.canTransform(world_frame_id, global_frame_id, ros::Time(0))) {
                tfListener_.lookupTransform(world_frame_id, global_frame_id, ros::Time(0), tfWorld2Map);
                tfWorld2Map_.setOrigin(tfWorld2Map.getOrigin());
                tfWorld2Map_.setRotation(tfWorld2Map.getRotation());
            }

            //ROS_INFO("%f %f %f", offsetBaseGT2BaseFootprint_.getOrigin().x(), offsetBaseGT2BaseFootprint_.getOrigin().y(), offsetBaseGT2BaseFootprint_.getOrigin().z());

            tfMap2Odom_ = tfWorld2Map_.inverse() * tfWorld2Optitrack_ * tfOptitrack2Footprint_ * tfOdom2Footprint_.inverse();

            //tfBroadcaster_.sendTransform(tf::StampedTransform((tfWorld2Map_), ros::Time::now(), "world", "map"));

            tfBroadcaster_.sendTransform(tf::StampedTransform((tfOptitrack2Footprint_), ros::Time::now(), "optitrack", "base_footprint_ground_truth"));
            
            tfBroadcaster_.sendTransform(tf::StampedTransform((tfWorld2Optitrack_), ros::Time::now(), "world", "optitrack"));

            tfBroadcaster_.sendTransform(tf::StampedTransform(tfMap2Odom_ * offsetBaseGT2BaseFootprint_.inverse(), ros::Time::now(), "map", "odom"));
            
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
        
        tf::Transform delta, offset;
        delta = pose * tfBaseInMap;
        offset = delta * offsetBaseGT2BaseFootprint_;

        tf::Vector3 origin = offset.getOrigin();
        tf::Matrix3x3 rotation(offset.getRotation());
        double roll, pitch, yaw;
//        mocap_localization::MocapLocalizationConfig config;
//        config.world2map_x = origin.x();
//        config.world2map_y = origin.y();
//        config.world2map_z = origin.z();
        rotation.getRPY(roll, pitch, yaw);
//        config.world2map_roll = roll;
//        config.world2map_pitch = pitch;
//        config.world2map_yaw = yaw;
        ROS_INFO("2D Pose estimate set 1");
        dynamic_reconfigure::DoubleParameter gt2fX, gt2fY, gt2fZ, gt2fR, gt2fP, gt2fYaw;
        dynamic_reconfigure::Config conf;
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        gt2fX.name = "gt2footprint_x";
        gt2fX.value = origin.x();
        gt2fY.name = "gt2footprint_y";
        gt2fY.value = origin.y();
        gt2fZ.name = "gt2footprint_z";
        gt2fZ.value = origin.z();
        gt2fR.name = "gt2footprint_roll";
        gt2fR.value = roll;
        gt2fP.name = "gt2footprint_pitch";
        gt2fP.value = pitch;
        gt2fYaw.name = "gt2footprint_yaw";
        gt2fYaw.value = yaw;
        conf.doubles.push_back(gt2fX);
        conf.doubles.push_back(gt2fY);
        conf.doubles.push_back(gt2fZ);
        conf.doubles.push_back(gt2fR);
        conf.doubles.push_back(gt2fP);
        conf.doubles.push_back(gt2fYaw);

        srv_req.config = conf;
        ros::service::call("/mocap_localization_node/set_parameters", srv_req, srv_resp);
        //dynCfgClient_.setConfiguration(config);

        ROS_INFO("2D Pose estimate set");

    };

    void onNewReconfigure(mocap_localization::MocapLocalizationConfig &config, uint32_t level){
        ROS_INFO("Reconfigured");
        tfWorld2Map_.setOrigin(tf::Vector3(config.world2map_x, config.world2map_y, config.world2map_z));
        tfWorld2Map_.setRotation(tf::createQuaternionFromRPY(config.world2map_roll, config.world2map_pitch, config.world2map_yaw));

        offsetBaseGT2BaseFootprint_.setOrigin(tf::Vector3(config.gt2footprint_x, config.gt2footprint_y, config.gt2footprint_z));
        offsetBaseGT2BaseFootprint_.setRotation(tf::createQuaternionFromRPY(config.gt2footprint_roll,
                                                                    config.gt2footprint_pitch, config.gt2footprint_yaw));

    }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "mocap_localization");
  ros::NodeHandle node;
  MocapLocalization mcl(node);
  ros::AsyncSpinner spinner(2);
  spinner.start();
//    ros::spin();
  ros::waitForShutdown();
  return 0;
};
