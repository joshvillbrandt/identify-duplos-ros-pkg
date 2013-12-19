#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <identify_duplos/Duplos.h>
#include <std_msgs/String.h>
#include <sstream>
#include <LinearMath/btVector3.h>
#include "DuploModel.cpp"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>

DuploModel duplo_model;
ros::Publisher pub_input;
ros::Publisher pub_target;
ros::Publisher pub_template;
bool halt = false;
bool verbose = false;
bool auto_keep = false;
bool color_filter = false;
bool shape_training = false;
bool color_training = false;

// Cluster Extraction vars
pcl::ExtractIndices<pcl::PointXYZRGB> extract;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr lastCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2 ros_cloud;

// Shape Training vars
bool first_shape = true;
TemplateAlignment template_align;
FeatureCloud template_cloud;
FeatureCloud halt_cloud;

void
processCloud(const sensor_msgs::PointCloud2ConstPtr& input)
{
	if(!halt) {
		// ROS to PCL
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		fromROSMsg(*input, *cloud);

		// Narrow camera range for easier training
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (-0.2, 0.2);
		pass.filter (*cloud);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (-0.2, 0.2);
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

		// Remove anything past surface
		pcl::PointIndices::Ptr behindPlane = duplo_model.filterBehindPlane(cloud, coefficients);
		extract.setInputCloud (cloud);
		extract.setIndices (behindPlane);
		extract.setNegative (false);
		extract.filter (*cloud);

		// Filter glove
		if(color_filter) {
			// black glove: 30, 30, 20, 20, 20, 20
			pcl::PointIndices::Ptr gloveIndicies = duplo_model.filterColor(cloud, 210, 70, 165, 95, 100, 100); // yellow
			extract.setInputCloud (cloud);
			extract.setIndices (gloveIndicies);
			extract.setNegative (false);
			extract.filter (*cloud);
		}

		if(cloud->points.size() > 100) {
			// Statistical outlier filter (Useful for carpet)
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
			sor.setInputCloud (cloud);
			sor.setMeanK (50);
			sor.setStddevMulThresh (2.0);
			sor.filter (*cloud);

			// Segment by distance
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
			clusters = duplo_model.segmentByDistance(cloud);

			// Work with largest cluster
			if(clusters.size() >= 1) {
				lastCloud = clusters[0];
				toROSMsg(*clusters[0],ros_cloud);
				pub_target.publish (ros_cloud);
			}
		}
	}
}

void
processControl(const std_msgs::String msg)
{
	if(strcmp(msg.data.c_str(), "halt") == 0) {
		halt = true;

		if(shape_training) {
			if(first_shape) {
				if(auto_keep) {
					template_cloud.setInputCloud(lastCloud);
					halt = false;
				}
				else halt_cloud.setInputCloud(lastCloud);
			}
			else {
				// build target cloud from input
				FeatureCloud target_cloud;
				target_cloud.setInputCloud(lastCloud);
				template_align.setTargetCloud(target_cloud);
				//ROS_INFO("Last capture size: %f", result.fitness_score);

				// align target to template
				TemplateAlignment::Result result;
				template_align.align(template_cloud, result);
				ROS_INFO("Fitness score: %f", result.fitness_score);

				// transform template to align with target
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::transformPointCloud (*template_cloud.getPointCloud (), *transformed_cloud, result.final_transformation);

				// combine transformed cloud with target for new sum
				*transformed_cloud += *lastCloud;

				// reduce with a voxel grid
				const float voxel_grid_size = 0.001f;
				pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
				vox_grid.setInputCloud (transformed_cloud);
				vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
				vox_grid.filter (*transformed_cloud);

				// set to halt cloud
				if(auto_keep) {
					template_cloud.setInputCloud(transformed_cloud);
					halt = false;

					// output
					toROSMsg(*template_cloud.getPointCloud (),ros_cloud);
					pub_template.publish (ros_cloud);
				}
				else {
					halt_cloud.setInputCloud(transformed_cloud);

					// output for approval
					toROSMsg(*transformed_cloud,ros_cloud);
					pub_target.publish (ros_cloud);
				}
			}
		}

		ROS_INFO("Halted. Toss or keep? (Or write.)");
	}
	else if(strcmp(msg.data.c_str(), "toss") == 0) {
		ROS_INFO("Cluster tossed.");
		halt = false;
	}
	else if(strcmp(msg.data.c_str(), "keep") == 0) {
		if(color_training) {
			duplo_model.colorStats(lastCloud);
		}
		else if(shape_training) {
			if(first_shape) first_shape = false;
			template_cloud = halt_cloud;

			// output
			toROSMsg(*template_cloud.getPointCloud (),ros_cloud);
			pub_template.publish (ros_cloud);
		}
		ROS_INFO("Cluster kept.");
		halt = false;
	}
	else if(strcmp(msg.data.c_str(), "write") == 0) {
		pcl::io::savePCDFileBinary ("output.pcd", *lastCloud);
		ROS_INFO("Cloud saved to output.pcd.");
	}
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "identify");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10); // hz

	// Check for input options
	for (int i = 1; i < argc; i++)
	{
		if(strcmp(argv[i], "-verbose") == 0) verbose = true;
		else if(strcmp(argv[i], "-auto") == 0) {
			auto_keep = true;
			ROS_INFO("Auto-keep clouds enabled.");
		}
		else if(strcmp(argv[i], "-filter") == 0) {
			color_filter = true;
			ROS_INFO("Color filter enabled.");
		}
		else if(strcmp(argv[i], "-color") == 0 && !shape_training) {
			color_training = true;
			ROS_INFO("Color training enabled.");
		}
		else if(strcmp(argv[i], "-shape") == 0 && !color_training) {
			shape_training = true;
			ROS_INFO("Shape training enabled.");
		}
	}

	// Create a ROS subscriber for the input duplo message
	ros::Subscriber sub = nh.subscribe ("identify/trainer", 1, processControl);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub2 = nh.subscribe ("/camera/rgb/points", 1, processCloud);

	// Create a ROS publisher for the output point cloud
	pub_input = nh.advertise<sensor_msgs::PointCloud2> ("identify/input_cloud", 1);
	pub_target = nh.advertise<sensor_msgs::PointCloud2> ("identify/target_cloud", 1);
	pub_template = nh.advertise<sensor_msgs::PointCloud2> ("identify/template_cloud", 1);

	// Spin
	while(ros::ok())
	{
		ros::spinOnce ();
		loop_rate.sleep();
	}
}
