#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>

class LivoxConverter
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::string input_topic_;
    std::string output_topic_;
    ros::Timer check_timer_;
    ros::Time last_message_time_;
    bool input_topic_available_;
    bool output_topic_advertised_;

public:
    LivoxConverter() : private_nh_("~"), input_topic_available_(false), output_topic_advertised_(false)
    {
        private_nh_.param<std::string>("input_topic", input_topic_, "/livox/lidar");
        private_nh_.param<std::string>("output_topic", output_topic_, "/livox/pointcloud2");

        ROS_INFO("Initialized with input_topic: %s, output_topic: %s", input_topic_.c_str(), output_topic_.c_str());

        check_timer_ = nh_.createTimer(ros::Duration(1.0), &LivoxConverter::checkTopicAvailability, this);
        ROS_INFO("Timer created");
    }

    void checkTopicAvailability(const ros::TimerEvent& event)
    {
        ros::master::V_TopicInfo topic_info;
        ros::master::getTopics(topic_info);

        bool found = false;
        for (const auto& topic : topic_info)
        {
            if (topic.name == input_topic_ && topic.datatype == "livox_ros_driver/CustomMsg")
            {
                found = true;
                break;
            }
        }

        if (found && !input_topic_available_)
        {
            input_topic_available_ = true;
            ROS_INFO("Input topic %s is now available. Advertising output topic %s", input_topic_.c_str(), output_topic_.c_str());
            advertiseOutputTopic();
        }
        else if (!found && input_topic_available_)
        {
            input_topic_available_ = false;
            ROS_INFO("Input topic %s is no longer available. Stopping advertisement of %s", input_topic_.c_str(), output_topic_.c_str());
            unadvertiseOutputTopic();
        }
    }

    void advertiseOutputTopic()
    {
        if (!output_topic_advertised_)
        {
            pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1, 
                boost::bind(&LivoxConverter::connectCallback, this, _1),
                boost::bind(&LivoxConverter::disconnectCallback, this, _1));
            output_topic_advertised_ = true;
        }
    }

    void unadvertiseOutputTopic()
    {
        if (output_topic_advertised_)
        {
            pub_.shutdown();
            sub_.shutdown();
            output_topic_advertised_ = false;
        }
    }

    void connectCallback(const ros::SingleSubscriberPublisher&)
    {
        if (!sub_ && input_topic_available_)
        {
            ROS_INFO("First subscriber connected. Subscribing to input topic %s", input_topic_.c_str());
            sub_ = nh_.subscribe(input_topic_, 1, &LivoxConverter::livoxCallback, this);
        }
    }

    void disconnectCallback(const ros::SingleSubscriberPublisher&)
    {
        if (pub_.getNumSubscribers() == 0)
        {
            ROS_INFO("Last subscriber disconnected. Unsubscribing from input topic %s", input_topic_.c_str());
            sub_.shutdown();
            // We no longer unadvertise the output topic here
        }
    }

    void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg)
    {
        last_message_time_ = ros::Time::now();
        ROS_DEBUG("Received input message with %zu points", msg->point_num);

        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.points.resize(msg->point_num);

        for (size_t i = 0; i < msg->point_num; ++i)
        {
            cloud.points[i].x = msg->points[i].x;
            cloud.points[i].y = msg->points[i].y;
            cloud.points[i].z = msg->points[i].z;
            cloud.points[i].intensity = msg->points[i].reflectivity;
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header = msg->header;

        pub_.publish(output);
        ROS_DEBUG("Published PointCloud2 message with %zu points", msg->point_num);
    }

    void run()
    {
        ROS_INFO("LivoxConverter running");
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_converter");
    LivoxConverter converter;
    converter.run();
    return 0;
}