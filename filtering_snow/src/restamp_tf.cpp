#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <vector>
#include <tf/transform_broadcaster.h>
#include<tf2_msgs/TFMessage.h>

std::string inputTopic, outputTopic;
tf2_msgs::TFMessage tf_out;
ros::Publisher pub_new_tf;

void tf_cb (const tf2_msgs::TFMessage::ConstPtr & tf_msg)
{
  tf_out = *tf_msg;
  tf_out.transforms[0].header.stamp = ros::Time::now();
  pub_new_tf.publish(tf_out);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "restamp_node");
  ros::NodeHandle nh;
  ROS_INFO("RESTAMP_NODE: Node Initialize");

  ros::param::get("/tf_input_topic", inputTopic);
  ros::param::get("/tf_output_topic", outputTopic);

  ROS_INFO("RESTAMP_NODE: The input topic is: %s",inputTopic.c_str());

  ros::Subscriber subTF = nh.subscribe (inputTopic, 100, tf_cb);
  pub_new_tf = nh.advertise<tf2_msgs::TFMessage> (outputTopic, 100);
  //ros::Time::init();
  ros::spin();
  return 0;
}
