#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

class LidarProject{

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh;

public:
    LidarProject();

    void Voxelization();
    void voxel_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};

LidarProject::LidarProject(){
    pub = nh.advertise<sensor_msgs::PointCloud2>("Voxelization",1);
    sub = nh.subscribe("/carla/ego_vehicle/lidar",1, &LidarProject::voxel_callback, this);
}
void LidarProject::Voxelization(){
}

void LidarProject::voxel_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 filter_cloud;

  pcl_conversions::toPCL(*msg,*cloud); // ROS sensor_msg -> PCL pointcloud2 type

  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_cloud;
  voxel_cloud.setInputCloud(cloudPtr);
  voxel_cloud.setLeafSize(0.15,0.15,0.15);
  voxel_cloud.filter(filter_cloud);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(filter_cloud , output); //PCL pointcloud2 -> ROS sensor_msg
  pub.publish(output);

  }


int main(int argc, char ** argv){
  ros::init(argc,argv,"pointcloud");

  LidarProject project;
  project.Voxelization();

  ros::spin();
}