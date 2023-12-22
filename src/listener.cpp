#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tf_relay_listener");
    ros::NodeHandle nh;

    float hz = nh.param<float>("hz", 1.0);
    std::string pose_topic, transform_topic, source_frame, target_frame;

    nh.getParam("source_frame", source_frame);
    nh.getParam("target_frame", target_frame);

    bool pub_pose = nh.param<bool>("pub_pose", false);
    bool pub_transform = nh.param<bool>("pub_transform", true);

    ros::Publisher pose_pub, transform_pub;

    if (pub_pose)
    {
        nh.getParam("pose_topic", pose_topic);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
    }

    if (pub_transform)
    {
        nh.getParam("transform_topic", transform_topic);
        transform_pub = nh.advertise<geometry_msgs::TransformStamped>(transform_topic, 10);
    }

    tf2_ros::Buffer tf_buf;
    tf2_ros::TransformListener tf_listener(tf_buf);

    ros::Rate rate(hz);

    geometry_msgs::TransformStamped transform_msg;
    geometry_msgs::PoseStamped pose_msg;

    while (ros::ok())
    {
        ros::Time stamp = ros::Time::now();

        transform_msg = tf_buf.lookupTransform(target_frame, source_frame, stamp);

        if (pub_pose)
        {
            pose_msg.header = transform_msg.header;
            pose_msg.pose.orientation = transform_msg.transform.rotation;
            pose_msg.pose.position.x = transform_msg.transform.translation.x;
            pose_msg.pose.position.y = transform_msg.transform.translation.y;
            pose_msg.pose.position.z = transform_msg.transform.translation.z;

            pose_pub.publish(pose_msg);
        }

        if (pub_transform)
            pose_pub.publish(transform_msg);

        rate.sleep();
    }

}