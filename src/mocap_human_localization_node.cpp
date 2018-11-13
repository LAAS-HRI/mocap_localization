#include <ros/ros.h>
#include "../include/optitrack/or_pose_estimator_state.h"
#include <tf/tf.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define NODE_NAME "mocap_human_localization"
#define HUMAN_SUB_PARAM_NAME "optitrack_human_topic_names"
#define HUMAN_SUB_PREPEND "/optitrack/bodies/"
#define TF_FRAME_PREPEND "mocap_human-"

using namespace std;

class MocapHumanLocalization{
    private:
    ros::NodeHandle node_;
    tf::TransformListener tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;
    vector<ros::Subscriber> personSubs_;
    ros::Timer locaTimerTf_;

    vector<pair<std::string, tf::Transform> > tfOptitrack2Humans_;

    public:
    MocapHumanLocalization(ros::NodeHandle node): tfListener_(), tfBroadcaster_(), node_(node){
        map<string, string> subscribedNodeNames;
        map<string, string>::iterator it;
        if (node_.getParam(HUMAN_SUB_PARAM_NAME, subscribedNodeNames)){
            for (it = subscribedNodeNames.begin(); it != subscribedNodeNames.end(); it++){
                tf::Transform transform;
                transform.setIdentity();
                string fullTopicName = HUMAN_SUB_PREPEND + it->first;
                personSubs_.push_back(node_.subscribe<optitrack::or_pose_estimator_state>(fullTopicName, 10,
                                                     boost::bind(&MocapHumanLocalization::updateMocapPersonPose, this, _1, it->second)));
                tfOptitrack2Humans_.push_back(make_pair(it->second, transform));

            }
            locaTimerTf_ = node_.createTimer(ros::Duration(1/30.0), &MocapHumanLocalization::updateTf,this);
        }else{
            ROS_ERROR_NAMED(NODE_NAME, "Can't find humans topic lists to subscribe to in the parameters");
        }


    }

    private:
    void updateTf(const ros::TimerEvent&){
        try {
            vector<pair<string, tf::Transform> >::iterator it;
            for (it = tfOptitrack2Humans_.begin(); it != tfOptitrack2Humans_.end(); it++) {
//                stringstream ss;
//                ss << TF_FRAME_PREPEND << it->first;
//                string fullTfFrameName = ss.str();
                tfBroadcaster_.sendTransform(
                        tf::StampedTransform(it->second, ros::Time::now(), "optitrack", it->first));
                geometry_msgs::TransformStamped t_footprint;
                t_footprint.header.frame_id = "optitrack";
                t_footprint.header.stamp = ros::Time::now();
                t_footprint.child_frame_id = it->first + "_footprint";
                t_footprint.transform.translation.x = it->second.getOrigin().x();
                t_footprint.transform.translation.y = it->second.getOrigin().y();
                t_footprint.transform.translation.z = 0;
                tf::Matrix3x3 m(it->second.getRotation());
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                m.setEulerYPR(yaw + M_PI_2, 0., 0.);
                tf::Quaternion q;
                m.getRotation(q);
                t_footprint.transform.rotation.x = q.x();
                t_footprint.transform.rotation.y = q.y();
                t_footprint.transform.rotation.z = q.z();
                t_footprint.transform.rotation.w = q.w();
                tfBroadcaster_.sendTransform(t_footprint);
            }
        }catch (tf::TransformException &ex){
            ROS_ERROR_NAMED(NODE_NAME, "Error occur : %s\n", ex.what());
        }
    };

    void updateMocapPersonPose(const optitrack::or_pose_estimator_state::ConstPtr& msg, const string frameName){
        geometry_msgs::Pose pose_received;
        tf::Transform tf_received;
        if(!msg->pos.empty()){
            //Setting position
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
            vector<pair<string, tf::Transform> >::iterator it;
            for (it = tfOptitrack2Humans_.begin(); it != tfOptitrack2Humans_.end(); it++){
                if (it->first == frameName){
                    it->second.setOrigin(tf_received.getOrigin());
                    it->second.setRotation(tf_received.getRotation());
                }
            }
        }
    };

};

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle node("~");
    MocapHumanLocalization mchl(node);
    ros::spin();
    return 0;
};
