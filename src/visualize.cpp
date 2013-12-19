#include <ros/ros.h>
#include <identify_duplos/Duplos.h>
#include <identify_duplos/Duplo.h>
#include <visualization_msgs/Marker.h>
// PCL specific includes

ros::Publisher marker_pub;
size_t lastSize = 0;

void
visualize_duplos(const identify_duplos::Duplos duplos)
{
	// should be a marker array, but diamondback rviz doesn't like this

	// Create rviz marker for each duplo
	for (size_t i = 0; i < duplos.duplos.size(); i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = duplos.header.frame_id;
		marker.header.stamp = duplos.header.stamp;

		marker.ns = "duplos";
		marker.id = i;

		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose = duplos.duplos[i].pose;

		if(duplos.duplos[i].shape == "2x4")
		{
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
		}
		else
		{
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
		}

		if(duplos.duplos[i].color == "red")
		{
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
		}
		else if(duplos.duplos[i].color == "orange")
		{
			marker.color.r = 1.0f;
			marker.color.g = 0.5f;
			marker.color.b = 0.0f;
		}
		else if(duplos.duplos[i].color == "yellow")
		{
			marker.color.r = 1.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
		}
		else if(duplos.duplos[i].color == "green")
		{
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
		}
		else if(duplos.duplos[i].color == "blue")
		{
			marker.color.r = 0.0f;
			marker.color.g = 0.0f;
			marker.color.b = 1.0f;
		}
		else
		{
			marker.color.r = 0.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
		}
		marker.color.a = 1.0f;

		marker.lifetime = ros::Duration();

		marker_pub.publish(marker);
	}

	// Clear old markers
	for(size_t i = duplos.duplos.size(); i < lastSize; i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = duplos.header.frame_id;
		marker.header.stamp = duplos.header.stamp;

		marker.ns = "duplos";
		marker.id = i;

		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::DELETE;

		marker.lifetime = ros::Duration();

		marker_pub.publish(marker);
	}

	// save size for next time
	lastSize = duplos.duplos.size();

	return;
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "visualize");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	// Create a ROS subscriber for the input duplo message
	ros::Subscriber sub = nh.subscribe ("identify/duplos", 1, visualize_duplos);

	// Create a ROS publisher for the markers
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualize/marker", 20);

	// Spin
	while(ros::ok())
	{
		ros::spinOnce ();
		loop_rate.sleep();
	}
}
