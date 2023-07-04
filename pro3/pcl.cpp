
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
 
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pcl");
    ros::NodeHandle nh;
 
    // 创建ROSbag对象并打开文件
    rosbag::Bag bag;
    bag.open("/mnt/p3/catkin_ws/src/pcl_display/src/all.bag", rosbag::bagmode::Read);
 
    // 定义需要读取的话题
    //std::vector<std::string> topics = {1,"/camera/depth/image_rect_raw"};
    std::vector<std::string> topics;
    topics.push_back("/rslidar_points");


    // 创建ROSbag视图对象，并设置话题过滤器
    rosbag::View view(bag, rosbag::TopicQuery(topics));
 
    // 创建PCL可视化对象
    pcl::visualization::PCLVisualizer viewer("Laser Scan");
 
    // 遍历ROSbag中的消息
    for (const rosbag::MessageInstance& msg : view)
    {
        // 检查消息类型是否为PointCloud2
        if (msg.getDataType() == "sensor_msgs/PointCloud2")
        {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
 
            // 创建PCL点云对象并从ROS消息中填充数据
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);
 
            // 在PCL可视化对象中添加点云数据
            viewer.addPointCloud(cloud, "cloud");

            //viewer.removeAllPointClouds();
 
            // 更新PCL可视化窗口
            viewer.spinOnce();
        }
    }
 
    // 关闭ROSbag文件
    bag.close();
 
    // 显示点云并保持窗口打开
    viewer.spin();
 
    return 0;
}
