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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

// #include "project/recFitting.h"

class LidarProject{
    ros::NodeHandle nh;

    ros::Publisher output_pub;
    ros::Subscriber lidar_sub;


public:
    LidarProject();

    void voxel_cb(const sensor_msgs::PointCloud2ConstPtr& ransac_msg);
    void ransac(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& filter_cloud);
    void Clustering(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inlierPoint_neg);
};

LidarProject::LidarProject(){

    lidar_sub = nh.subscribe("/carla/ego_vehicle/lidar",1, &LidarProject::voxel_cb, this);
    output_pub = nh.advertise<sensor_msgs::PointCloud2>("output",1);

}

/*
void LidarProject::voxel_callback(const sensor_msgs::PointCloud2ConstPtr& voxel_msg){
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2 filter_cloud;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  pcl_conversions::toPCL(*voxel_msg,*cloud); // ROS sensor_msg -> PCL pointcloud2 type

  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_cloud;
  voxel_cloud.setInputCloud(cloudPtr);
  voxel_cloud.setLeafSize(0.03,0.03,0.03);
  voxel_cloud.filter(filter_cloud);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(filter_cloud , output); //PCL pointcloud2 -> ROS sensor_msg
  voxel_pub.publish(output);

  }
*/

// ------------------- Voxel+Ransac+clustering -------------------------------
void LidarProject::voxel_cb(const sensor_msgs::PointCloud2ConstPtr& ransac_msg){  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*ransac_msg, cloud);

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud.makeShared()); //point cloud 객체에 shared_pointer 생성
  vg.setLeafSize(0.3, 0.3, 0.3);
  vg.filter(*filter_cloud);

  ransac(filter_cloud);
}

void LidarProject::ransac(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& filter_cloud){
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints_neg(new pcl::PointCloud<pcl::PointXYZ>());

  // SACSegmentation 을 위해서 seg 를 만들고 방법과 모델과 기준을 정함
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);
  seg.setMaxIterations(100);

  seg.setInputCloud(filter_cloud);
  seg.segment(*inliers,*coefficients);

  // input cloud에서 평면 inlier를 추출함.
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(filter_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);           //바닥제거하기 위해 inlier를 없앤다

  extract.filter(*inlierPoints_neg);

  Clustering(inlierPoints_neg);
}

void LidarProject::Clustering(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inlierPoints_neg){
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (inlierPoints_neg);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.8);
  ec.setMinClusterSize (15);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (inlierPoints_neg);
  ec.extract (cluster_indices);
 
  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
 
  pcl::PointCloud<pcl::PointXYZI> TotalCloud; 
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
        pcl::PointXYZ pt = inlierPoints_neg->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j + 1);

            TotalCloud.push_back(pt2);
    }
    j++;
  }
  
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "ego_vehicle/lidar";


  output_pub.publish(output); 

}

/*
// ------------------------------Clustering-----------------------------

void LidarProject::cluster_callback(const sensor_msgs::PointCloud2ConstPtr& cluster_msg){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cluster_msg, cloud);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_Ptr=cloud;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_Ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.5);
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_Ptr);
  ec.extract (cluster_indices);
 

  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
 
  pcl::PointCloud<pcl::PointXYZI> TotalCloud; 
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
        pcl::PointXYZ pt = cloud_Ptr->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j + 1);

            TotalCloud.push_back(pt2);
    }
    j++;
  }
  
    // Convert To ROS data type 
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "ego_vehicle";   
  cluster_pub.publish(output); 
}
*/

int main(int argc, char ** argv){
  ros::init(argc,argv,"pointcloud");
  
  LidarProject project;
  
  ros::spin();
}