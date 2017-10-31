The filter_point_cloud package takes a point cloud that is being published to /cloud_pcd and filters it using a radius outlier filter nodelet and also a voxel filter nodelet.

We recommend exporting your point cloud to a pcl file using pcl_ros bag_to_pcd:

	rosrun pcl_ros bag_to_pcd enter_bag_name.bag /enter_cloud_topic ./enter_directory /enter_frame_id

   ex:

	rosrun pcl_ros bag_to_pcd Clearpath_workshop_and_lab_2017-06-26-16-56-53.bag /metascan ./pcd_files /odom

Then run your point cloud using pcl_ros pcd_to_pointcloud:

	rosrun pcl_ros pcd_to_pointcloud <file.pcd> [ <interval> ] _frame_id:=/enter_frame

   ex:

	rosrun pcl_ros pcd_to_pointcloud ./pcd_files/Clearpath_workshop_and_lab_2017-06-26-16-56-53.pcd 0.1 _frame_id:=/odom

See following tutorial: http://wiki.ros.org/pcl_ros

Also, refer to: http://wiki.ros.org/pcl_ros/Tutorials

