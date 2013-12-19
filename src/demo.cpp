#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

ros::Publisher pub_input;
ros::Publisher pub_filtered;
ros::Publisher pub_downsampled;
ros::Publisher pub_planar;
ros::Publisher pub_objects;
ros::Publisher pub_projected;
ros::Publisher pub_hull;

using namespace pcl;

void
process_cloud(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planar (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_objects (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);

	sensor_msgs::PointCloud2::Ptr cloud2 (new sensor_msgs::PointCloud2 ());
	sensor_msgs::PointCloud2 filtered2,downsampled2,planar2,objects2,projected2,hull2;

	fromROSMsg(*input, *cloud);
	pub_input.publish(*input);

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.4, 0.4);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);
	toROSMsg(*cloud_filtered,filtered2);
	pub_filtered.publish (filtered2);

	// Create the filtering object
	//*cloud2 = filtered2;
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (input);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (downsampled2);
	fromROSMsg(downsampled2,*downsampled);
	pub_downsampled.publish (downsampled2);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	seg.setInputCloud (downsampled);
	seg.segment (*inliers, *coefficients);
	extract.setInputCloud (downsampled);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_planar);
	toROSMsg(*cloud_planar,planar2);
	pub_planar.publish (planar2);

	extract.setNegative (true);
	extract.filter (*cloud_objects);
	toROSMsg(*cloud_objects,objects2);
	pub_objects.publish (objects2);

	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients ());
	coefficients2->values.resize (4);
	coefficients2->values[0] = coefficients2->values[1] = 0;
	coefficients2->values[2] = 1.0;
	coefficients2->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud_objects);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);
	toROSMsg(*cloud_projected,projected2);
	pub_projected.publish (projected2);

	// Create a Concave Hull representation of the projected inliers
	pcl::ConcaveHull<pcl::PointXYZRGB> chull;
	chull.setInputCloud (cloud_projected);
	chull.setAlpha (0.1);
	chull.reconstruct (*cloud_hull);
	toROSMsg(*cloud_hull,hull2);
	pub_hull.publish (hull2);

	return;
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/camera/rgb/points", 1, process_cloud);

	// Create a ROS publisher for the output point cloud
	pub_input = nh.advertise<sensor_msgs::PointCloud2> ("new_pcd", 1);
	pub_filtered = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);
	pub_downsampled = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_cloud", 1);
	pub_planar = nh.advertise<sensor_msgs::PointCloud2> ("planar_cloud", 1);
	pub_objects = nh.advertise<sensor_msgs::PointCloud2> ("objects_cloud", 1);
	pub_projected = nh.advertise<sensor_msgs::PointCloud2> ("projected_cloud", 1);
	pub_hull = nh.advertise<sensor_msgs::PointCloud2> ("concave_hull", 1);

	// Spin
	ros::spin ();
}
