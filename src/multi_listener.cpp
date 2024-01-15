#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tf_relay_multi_listener");
    ros::NodeHandle nh("~");

    float hz = nh.param<float>("hz", 1.0);
    std::string transform_topic;
    std::map<std::string, std::string> frame_map;

    ROS_ASSERT_MSG(nh.getParam("frame_map", frame_map), "Must provide {ref_frame: child_frame} mapping as ROS param.");

    ros::Publisher transform_pub;

    transform_topic = nh.param<std::string>("transform_topic", "transform");

    transform_pub = nh.advertise<geometry_msgs::TransformStamped>(transform_topic, 10);

    tf2_ros::Buffer tf_buf;
    tf2_ros::TransformListener tf_listener(tf_buf);

    ros::Rate rate(hz);

    geometry_msgs::TransformStamped transform_msg;
    geometry_msgs::PoseStamped pose_msg;

    while (ros::ok())
    {
        for (const auto& item : frame_map)
        {
            const auto& reference_frame = item.first;
            const auto& child_frame = item.second;

            try
            {
                transform_msg = tf_buf.lookupTransform(reference_frame, child_frame, ros::Time(0));
                transform_pub.publish(transform_msg);
            }
            catch(tf2::TransformException e)
            {
                ROS_WARN(e.what());
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

}