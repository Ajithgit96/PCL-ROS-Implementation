#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> cloud_baseRemoved;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (cloud_filtered);

  //convert to pcl::PointCloud<pcl::PointXYZ>
  pcl::fromPCLPointCloud2(cloud_filtered,*cloud_XYZ);

  //Variables and parameters for planar_segentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  //seg.setDistanceThreshold (0.01);
  seg.setDistanceThreshold (0.05);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_XYZ);
  seg.segment (*inliers, *coefficients);
  //if (inliers->indices.size () == 0){
    //std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    //break;
  //}

  // Extract the inliers
  extract.setInputCloud (cloud_XYZ);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (cloud_baseRemoved);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  //pcl_conversions::moveFromPCL(cloud_filtered, output);
  pcl::toROSMsg(cloud_baseRemoved,output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}