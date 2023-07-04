#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
 
int main(int argc, char** argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "imu");
  ros::NodeHandle nh;
 
  // 打开 ROSBAG 文件
  rosbag::Bag bag;
  bag.open("/mnt/pro1/catkin_ws/src/all.bag", rosbag::bagmode::Read);  //修改bag地址
 
  // 创建一个 View，用于获取指定话题的消息
  rosbag::View view(bag, rosbag::TopicQuery("/imu/data_raw"));  //订阅的话题名称
 
  // 遍历 ROSBAG 中的消息
  for (const rosbag::MessageInstance& msg : view)
  {
    // 检查消息类型是否为 IMU 消息
    if (msg.isType<sensor_msgs::Imu>())
    {
      // 将消息转换为 IMU 消息类型
      sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
      if (imu_msg != nullptr)
      {
      // 在命令行中打印 IMU 数据
      std::cout << "Timestamp: " << imu_msg->header.stamp << std::endl;
      std::cout << "Angular Velocity (x, y, z): " << imu_msg->angular_velocity.x << ", "
                << imu_msg->angular_velocity.y << ", " << imu_msg->angular_velocity.z << std::endl;
      std::cout << "Linear Acceleration (x, y, z): " << imu_msg->linear_acceleration.x << ", "
                << imu_msg->linear_acceleration.y << ", " << imu_msg->linear_acceleration.z << std::endl;
      std::cout << "Orientation (x, y, z, w): " << imu_msg->orientation.x << ", "
                << imu_msg->orientation.y << ", " << imu_msg->orientation.z << ", "
                << imu_msg->orientation.w << std::endl;
      std::cout << "----------------------------------" << std::endl;
      }
    }
  }
 
  // 关闭 ROSBAG 文件
  bag.close();
 
  return 0;
}