#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/point_cloud.h>

class LidarProject{

    ros::Publisher voxel_pub;
    ros::Subscriber voxel_sub;
    ros::NodeHandle nh;

    ros::Publisher ransac_pub;
    ros::Subscriber ransac_sub;

public:
    LidarProject();

    void Voxelization(); //아직 안 씀
    void voxel_callback(const sensor_msgs::PointCloud2ConstPtr& voxel_msg);
    void ransac_callback(const sensor_msgs::PointCloud2ConstPtr& ransac_msg);
};

LidarProject::LidarProject(){
    voxel_pub = nh.advertise<sensor_msgs::PointCloud2>("Voxelization",1);
    voxel_sub = nh.subscribe("/carla/ego_vehicle/lidar",1, &LidarProject::voxel_callback, this);

    ransac_sub = nh.subscribe("/carla/ego_vehicle/lidar",1, &LidarProject::ransac_callback, this);
    ransac_pub = nh.advertise<sensor_msgs::PointCloud2>("RANSAC",1);
}

void LidarProject::voxel_callback(const sensor_msgs::PointCloud2ConstPtr& voxel_msg){
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2 filter_cloud;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  pcl_conversions::toPCL(*voxel_msg,*cloud); // ROS sensor_msg -> PCL pointcloud2 type

  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_cloud;
  voxel_cloud.setInputCloud(cloudPtr);
  voxel_cloud.setLeafSize(0.04,0.04,0.04);
  voxel_cloud.filter(filter_cloud);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(filter_cloud , output); //PCL pointcloud2 -> ROS sensor_msg
  voxel_pub.publish(output);

  }

// ------------------- ransac(ground removal) -------------------------------
void LidarProject::ransac_callback(const sensor_msgs::PointCloud2ConstPtr& ransac_msg){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*ransac_msg, cloud);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_Ptr=cloud;           // pcl::PointCloud 와 pcl::PointCloud ::Ptr 연결

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints_neg(new pcl::PointCloud<pcl::PointXYZ>());

  // SACSegmentation 을 위해서 seg 를 만들고 방법과 모델과 기준을 정함
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setMaxIterations(100);

  seg.setInputCloud(cloud_Ptr);
  seg.segment(*inliers,*coefficients);


  // input cloud에서 평면 inlier를 추출함.
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_Ptr);
  extract.setIndices(inliers);
  extract.setNegative(true);           //바닥제거하기 위해 inlier를 없앤다


  extract.filter(*inlierPoints_neg);
  
  

  // 결과값을 포인터로 받음
  // 포인터를 PCLPointCloud2 로 변경 그리고 다시 sensor_msgs 로 변경 후 publish
  pcl::PCLPointCloud2 outlier_cloud;
  sensor_msgs::PointCloud2 outlier_cloud_msg;
  pcl::toPCLPointCloud2(*inlierPoints_neg,outlier_cloud);
  pcl_conversions::fromPCL(outlier_cloud,outlier_cloud_msg);


  ransac_pub.publish(outlier_cloud_msg);
}


int main(int argc, char ** argv){
  ros::init(argc,argv,"pointcloud");

  LidarProject project;

  ros::spin();
}