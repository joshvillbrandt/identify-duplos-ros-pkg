identify-duplos-ros-pkg
=======================

A ROS package to identify LEGO DUPLO blocks from point cloud data.

## Summary

Name:	Josh Villbrandt
Email:	josh@javconcepts.com
Date:	Fall 2011
Class:	USC CSCI 547

This program attempts to identify LEGO Duplo blocks on a table on send out identification information so that a robot arm may pick up the pieces. Specific quantities measured for each block include color, shape, and 6DOF pose.

## Implementation Details
See final report PDF.

## Installation / Usage Preparation
svn co http://svn.javconcepts.com/cs547/trunk/identify_duplos
make eclipse-project
rosmake identify_duplos
roscore
rosrun rviz rviz
roscd identify_duplos
roslaunch openni_camera openni_node.launch #for kinect
rosrun identify_duplos data/read_pcd group2_3.pcd #for pcd file

## Usage [trainer]
rosrun identify_duplos trainer -color [-auto] [-filter] #for color training
rosrun identify_duplos trainer -shape [-auto] [-filter] #for shape training
rostopic pub -1 identify/trainer std_msgs/String "halt"
rostopic pub -1 identify/trainer std_msgs/String "keep"
rostopic pub -1 identify/trainer std_msgs/String "toss"
rostopic pub -1 identify/trainer std_msgs/String "write"

## Usage [identify, visualize]
rosrun identify_duplos identify confile_file [-quick] [-verbose]
rosrun identify_duplos visualize
rostopic echo identify/duplos

## Revision History
1.0 - midterm project update (associated PDF)
1.1 - branching away from simple identification
1.2 - final project data, works great but still room for improvement (associated PDF)