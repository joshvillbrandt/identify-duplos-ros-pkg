#include <ros/ros.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

ros::Publisher pub;

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "read_pcd");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/rgb/points", 1);

	// Read in file
	if(argc < 2) ROS_ERROR("no file argument given");
	sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
	pcl::PCDReader reader;
	reader.read(argv[1], *cloud);
	cloud->header.frame_id = "/openni_camera";
	std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << argv[1] << std::endl;

	//TODO: upgrade to
	//xyz_ = PointCloud::Ptr (new PointCloud);
	//pcl::io::loadPCDFile (pcd_file, *xyz_);

	// Hack to convert to RGB point cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
	//fromROSMsg(*cloud, *cloud_xyz);
	//pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
	//copyPointCloud(*cloud_xyz, cloud_xyzrgb);
	//toROSMsg(cloud_xyzrgb, *cloud);

	// Send and spin
	while(ros::ok())
	{
		pub.publish (cloud);
		ros::spinOnce ();
		loop_rate.sleep();
	}
}
