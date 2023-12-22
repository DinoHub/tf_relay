#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tf_relay_listener");
    ros::NodeHandle nh("~");

    float hz = nh.param<float>("hz", 1.0);
    std::string pose_topic, transform_topic, reference_frame, child_frame;

    nh.getParam("reference_frame", reference_frame);
    nh.getParam("child_frame", child_frame);

    bool pub_pose = nh.param<bool>("pub_pose", false);
    bool pub_transform = nh.param<bool>("pub_transform", true);

    ros::Publisher pose_pub, transform_pub;

    pose_topic = nh.param<std::string>("pose_topic", "pose");
    transform_topic = nh.param<std::string>("transform_topic", "transform");

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>(transform_topic, 10);

    tf2_ros::Buffer tf_buf;
    tf2_ros::TransformListener tf_listener(tf_buf);

    ros::Rate rate(hz);

    geometry_msgs::TransformStamped transform_msg;
    geometry_msgs::PoseStamped pose_msg;

    while (ros::ok())
    {
        ros::Time stamp = ros::Time::now();

        try
        {
            transform_msg = tf_buf.lookupTransform(reference_frame, child_frame, stamp);
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
            {
                transform_pub.publish(transform_msg);
            }
        }
        catch(tf2::LookupException e)
        {
            ROS_WARN(e.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

}