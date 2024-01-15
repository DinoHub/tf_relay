#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cassert>

std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
std::string g_reference_frame, g_child_frame;

void pose_cb(geometry_msgs::PoseStampedConstPtr a_msg)
{
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = g_reference_frame;
    msg.header.stamp = a_msg->header.stamp;

    msg.transform.translation.z = a_msg->pose.position.x;
    msg.transform.translation.y = a_msg->pose.position.y;
    msg.transform.translation.z = a_msg->pose.position.z;

    msg.transform.rotation = a_msg->pose.orientation;

    tf_broadcaster->sendTransform(msg);
}

void transform_cb(geometry_msgs::TransformStampedConstPtr a_msg)
{
    tf_broadcaster->sendTransform(*a_msg);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tf_relay_publisher");
    ros::NodeHandle nh("~");

    std::string pose_topic, transform_topic;

    bool sub_pose = nh.param<bool>("sub_pose", false);
    bool sub_transform = nh.param<bool>("sub_transform", true);

    ros::Subscriber pose_sub, transform_sub;

    if (sub_pose)
    {
        pose_topic = nh.param<std::string>("pose_topic", "pose");
        bool got_ref_frame = nh.getParam("reference_frame", g_reference_frame);
        bool got_child_frame = nh.getParam("child_frame", g_child_frame);

        ROS_ASSERT_MSG(got_child_frame && got_ref_frame, "Frames must be specified if subscribing to pose.");

        pose_sub = nh.subscribe(pose_topic, 10, pose_cb);
    }

    if (sub_transform)
    {
        transform_topic = nh.param<std::string>("transform_topic", "transform");
        transform_sub = nh.subscribe(transform_topic, 10, transform_cb);
    }

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();

    ros::spin();
}