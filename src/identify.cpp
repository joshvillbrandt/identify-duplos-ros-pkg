#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <identify_duplos/Duplos.h>
#include <sstream>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include "DuploModel.cpp"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>

DuploModel duplo_model;
ros::Publisher pub_input;
ros::Publisher pub_objects;
ros::Publisher pub_duplos;
sensor_msgs::PointCloud2 ros_cloud;
bool verbose = false;
bool quick = false;
pcl::ExtractIndices<pcl::PointXYZRGB> extract;

void
processCloud(const sensor_msgs::PointCloud2ConstPtr& input)
{
	// ROS to PCL
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	fromROSMsg(*input, *cloud);

	// Narrow camera range because my chair is in the way
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.4, 0.4);
	pass.filter (*cloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.3, 0.3);
	pass.filter (*cloud);
	toROSMsg(*cloud,ros_cloud);
	pub_input.publish (ros_cloud);

	// Separate largest contiguous plane (floor) via RANSAC
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);
	seg.setOptimizeCoefficients (true); // optional

	// Filter out surface
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*cloud);

	/*std::cerr << "Model coefficients: " << coefficients->values[0] << " "
									  << coefficients->values[1] << " "
									  << coefficients->values[2] << " "
									  << coefficients->values[3] << std::endl;*/
	//std::cout << cloud->points[0] << std::endl;

	// Remove anything past surface
	pcl::PointIndices::Ptr behindPlane = duplo_model.filterBehindPlane(cloud, coefficients);
	extract.setInputCloud (cloud);
	extract.setIndices (behindPlane);
	extract.setNegative (false);
	extract.filter (*cloud);

	// Statistical outlier filter (Useful for carpet)
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (2.0);
	sor.filter (*cloud);

	// Segment by color
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> colorClusters = duplo_model.segmentByColor(cloud);

	// Segment by distance
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(size_t i = 0; i < colorClusters.size(); i++)
	{
		if(i == 0) *objects_cloud = *colorClusters[i];
		else *objects_cloud += *colorClusters[i];
		duplo_model.vectorAppend(duplo_model.segmentByDistance(colorClusters[i]), clusters);
	}
	toROSMsg(*objects_cloud,ros_cloud);
	pub_objects.publish (ros_cloud);

	// Generate values for each cluster
	identify_duplos::Duplos duplos;
	duplos.header.frame_id = input->header.frame_id;
	//clusters.push_back(surface);
	for (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::size_type i = 0; i < clusters.size(); i++)
	{
		// Block basics
		identify_duplos::Duplo duplo;
		uint8_t r, g, b;
		pcl::PointXYZRGB avg = duplo_model.averageCloud(clusters[i], r, g, b);

		// shape
		if(quick)
		{
			// use a voxel grid if I want these numbers to mean anything
			if(clusters[i]->points.size() > 425) duplo.shape = "2x4";
			else duplo.shape = "2x2";
		}
		else
		{
			int shape = duplo_model.identifyShape(clusters[i]);
			if(shape >= 0) duplo.shape = duplo_model.template_align.templates_[shape].name;
			else duplo.shape = "unknown";
		}

		// color
		int color = duplo_model.identifyColor(r, g, b);
		if(color >= 0) duplo.color = duplo_model.duplo_colors[color].name;
		else duplo.color = "unknown";

		// position
		duplo.pose.position.x = avg.x;
		duplo.pose.position.y = avg.y;
		duplo.pose.position.z = avg.z;

		// orientation
		pcl::ModelCoefficients::Ptr normal = duplo_model.identifyOrientation(clusters[i]);
		Ogre::Vector3 v1(1, 0, 0); // identity orientation
		Ogre::Vector3 v2(normal->values[0] * -1, normal->values[1] * -1, normal->values[2] * -1); // *-1 to flip direction
		Ogre::Quaternion orient = v1.getRotationTo( v2 );
		duplo.pose.orientation.x = orient.x;
		duplo.pose.orientation.y = orient.y;
		duplo.pose.orientation.z = orient.z;
		duplo.pose.orientation.w = orient.w;

		// add to duplos vector
		duplos.duplos.push_back(duplo);
	}

	// Verbose
	if(verbose) {
		std::stringstream ss;
		ss << "Found " << clusters.size() << " clusters: ";
		for(size_t i = 0; i < duplos.duplos.size(); i++)
		{
			ss << duplos.duplos[i].color << " " << duplos.duplos[i].shape << ", ";
		}
		ROS_INFO("%s", ss.str().c_str());
	}

	// Publish results
	pub_duplos.publish (duplos);

	return;
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "identify");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1); // 1 hz
    char *config_filename = (char *)"";

	// Check for color calibration
	for (int i = 1; i < argc; i++)
	{
		if(strcmp(argv[i], "-verbose") == 0) verbose = true;
		else if(strcmp(argv[i], "-quick") == 0) {
			quick = true;
			ROS_INFO("Quick mode enabled.");
		}
		else if(strcmp(config_filename, "") == 0) {
			config_filename = argv[i];
		}
	}

	// Check for config filename
	if(strcmp(config_filename, "") == 0) {
		ROS_ERROR("Config file name not provided.");
	}
	else {
		// Load config
		duplo_model.loadConfig(config_filename);

		// Create a ROS subscriber for the input point cloud
		ros::Subscriber sub = nh.subscribe ("/camera/rgb/points", 1, processCloud);

		// Create a ROS publisher for the output point cloud
		pub_input = nh.advertise<sensor_msgs::PointCloud2> ("identify/input_cloud", 1);
		pub_objects = nh.advertise<sensor_msgs::PointCloud2> ("identify/objects_cloud", 1);
		pub_duplos = nh.advertise<identify_duplos::Duplos> ("identify/duplos", 1);

		// Spin
		while(ros::ok())
		{
			ros::spinOnce ();
			loop_rate.sleep();
		}
    }
}
