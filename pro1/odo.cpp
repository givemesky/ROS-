#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
 
int main(int argc, char** argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "odo");
  ros::NodeHandle nh;
 
  // 打开 ROSBAG 文件
  rosbag::Bag bag;
  bag.open("/mnt/pro1/catkin_ws/src/all.bag", rosbag::bagmode::Read);
 
  // 创建一个 View，用于获取指定话题的消息
  rosbag::View view(bag, rosbag::TopicQuery("/odom"));
 
  // 遍历 ROSBAG 中的消息
  for (const rosbag::MessageInstance& msg : view)
  {
    // 检查消息类型是否为 Odometry 消息
    if (msg.isType<nav_msgs::Odometry>())
    {
      // 将消息转换为 Odometry 消息类型
      nav_msgs::Odometry::ConstPtr odom_msg = msg.instantiate<nav_msgs::Odometry>();
 
      // 在命令行中打印 Odometry 数据
      std::cout << "Timestamp: " << odom_msg->header.stamp << std::endl;
      std::cout << "Position (x, y, z): " << odom_msg->pose.pose.position.x << ", "
                << odom_msg->pose.pose.position.y << ", " << odom_msg->pose.pose.position.z << std::endl;
      std::cout << "Orientation (x, y, z, w): " << odom_msg->pose.pose.orientation.x << ", "
                << odom_msg->pose.pose.orientation.y << ", " << odom_msg->pose.pose.orientation.z << ", "
                << odom_msg->pose.pose.orientation.w << std::endl;
      std::cout << "Linear Velocity (x, y, z): " << odom_msg->twist.twist.linear.x << ", "
                << odom_msg->twist.twist.linear.y << ", " << odom_msg->twist.twist.linear.z << std::endl;
      std::cout << "Angular Velocity (x, y, z): " << odom_msg->twist.twist.angular.x << ", "
                << odom_msg->twist.twist.angular.y << ", " << odom_msg->twist.twist.angular.z << std::endl;
      std::cout << "----------------------------------" << std::endl;
    }
  }
 
  // 关闭 ROSBAG 文件
  bag.close();
 
  return 0;
}