#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
	get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
	return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
	yaml_dict = {}
	yaml_dict["test_scene_num"] = test_scene_num.data
	yaml_dict["arm_name"]  = arm_name.data
	yaml_dict["object_name"] = object_name.data
	yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
	yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
	return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
	data_dict = {"object_list": dict_list}
	with open(yaml_filename, 'w') as outfile:
		yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

	# TODO: Convert ROS msg to PCL data
	cloud = ros_to_pcl(pcl_msg)
	
	# TODO: Statistical Outlier Filtering
	outlier_filter = cloud.make_statistical_outlier_filter()
	outlier_filter.set_mean_k(100)
	outlier_filter.set_std_dev_mul_thresh(0.003)
	cloud_filtered = outlier_filter.filter()
	# pcl.save(cloud_filtered, "project_table_scene.pcd")


# 	# TODO: Voxel Grid Downsampling
	vox = cloud_filtered.make_voxel_grid_filter()
	LEAF_SIZE = 0.002   
	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
	cloud_filtered = vox.filter()
	# pcl.save(cloud_filtered, "voxel_downsampled.pcd")

# 	# TODO: PassThrough Filter
	passthrough_z= cloud_filtered.make_passthrough_filter()
	passthrough_z.set_filter_field_name("z")
	passthrough_z.set_filter_limits(0.62, 1.2)
	cloud_filtered_z = passthrough_z.filter()
	passthrough_y= cloud_filtered_z.make_passthrough_filter()
	passthrough_y.set_filter_field_name("y")
	passthrough_y.set_filter_limits(-0.50, 0.3)
	cloud_filtered = passthrough_y.filter()
	# pcl.save(cloud_filtered, "pass_through_filtered.pcd")

# 	# TODO: RANSAC Plane Segmentation
	seg = cloud_filtered.make_segmenter()
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	max_distance = 0.08
	seg.set_distance_threshold(max_distance)
	inliers, coefficients = seg.segment()

# 	# TODO: Extract inliers and outliers
	cloud_objects = cloud_filtered.extract(inliers, negative=False)
	# pcl.save(cloud_objects, "extracted_inliers.pcd")

# 	# Extract outliers
	cloud_table = cloud_filtered.extract(inliers, negative=True)
	# pcl.save(cloud_table, "extracted_outliers.pcd")

# 	# # TODO: Euclidean Clustering
	white_cloud = XYZRGB_to_XYZ(cloud_objects)
	tree = white_cloud.make_kdtree()

	# TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
	ec = white_cloud.make_EuclideanClusterExtraction()
	ec.set_ClusterTolerance(0.02)
	ec.set_MinClusterSize(300)
	ec.set_MaxClusterSize(10000)
	# Search the k-d tree for clusters
	ec.set_SearchMethod(tree)
	# Extract indices for each of the discovered clusters
	cluster_indices = ec.Extract()
	#Assign a color corresponding to each segmented object in scene
	cluster_color = get_color_list(len(cluster_indices))

	color_cluster_point_list = []

	for j, indices in enumerate(cluster_indices):
		for i, indice in enumerate(indices):
			color_cluster_point_list.append([white_cloud[indice][0],
											white_cloud[indice][1],
											white_cloud[indice][2],
											 rgb_to_float(cluster_color[j])])

	#Create new cloud containing all clusters, each with unique color
	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)
	# pcl.save(cluster_cloud, "cluster_cloud.pcd")

	# TODO: Convert PCL data to ROS messages
	ros_cloud_objects = pcl_to_ros(cloud_objects)
	ros_cloud_table = pcl_to_ros(cloud_table)
	ros_cluster_cloud = pcl_to_ros(cluster_cloud)

	# TODO: Publish ROS messages
	pcl_objects_pub.publish(ros_cloud_objects)
	pcl_table_pub.publish(ros_cloud_table)
	pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

	detected_objects_labels = []
	detected_objects = []
	for index, pts_list in enumerate(cluster_indices):
		# Grab the points for the cluster from the extracted outliers (cloud_objects)
		pcl_cluster = cloud_objects.extract(pts_list)
		# TODO: convert the cluster from pcl to ROS using helper function
		ros_cluster = pcl_to_ros(pcl_cluster)
		# Extract histogram features
		# TODO: complete this step just as is covered in capture_features.py

		chists = compute_color_histograms(ros_cluster, using_hsv=True)
		normals = get_normals(ros_cluster)
		nhists = compute_normal_histograms(normals)
		feature = np.concatenate((chists, nhists))

		# hist_bins=64
		# chists = compute_color_histograms(ros_cluster, nbins=hist_bins, using_hsv=True)
		# normals = get_normals(ros_cluster)
		# nhists = compute_normal_histograms(normals, nbins=hist_bins)
		# feature = np.concatenate((chists, nhists))

		# Make the prediction, retrieve the label for the result
		# and add it to detected_objects_labels list
		prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
		label = encoder.inverse_transform(prediction)[0]
		detected_objects_labels.append(label)

		# Publish a label into RViz
		label_pos = list(white_cloud[pts_list[0]])
		label_pos[2] += .4
		object_markers_pub.publish(make_label(label,label_pos, index))

		# Add the detected object to the list of detected objects.
		do = DetectedObject()
		do.label = label
		do.cloud = ros_cluster
		detected_objects.append(do)

	rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

	# Publish the list of detected objects
	# This is the output you'll need to complete the upcoming project!
	detected_objects_pub.publish(detected_objects)
	# detected_objects_pub.publish(pcl_msg)

	# Suggested location for where to invoke your pr2_mover() function within pcl_callback()
	# Could add some logic to determine whether or not your object detections are robust
	# before calling pr2_mover()
	# try:
	# 	pr2_mover(detected_objects_list)
	# except rospy.ROSInterruptException:
	# 	pass

# function to load parameters and request PickPlace service
# def pr2_mover(object_list):

	# TODO: Initialize variables
	test_scene_num = Int32()
	object_name = String()
	arm_name = String()
	pick_pose = Pose()
	place_pose = Pose()
	
	dict_list = []
	yaml_filename = 'output_1.yaml'
	test_scene_num.data = 1

	labels = []
	centroids = []
	for objects in detected_objects:
		labels.append(objects.label)
		points_arr = ros_to_pcl(objects.cloud).to_array()
		centroids.append(np.mean(points_arr, axis=0)[:3])

# 	# TODO: Get/Read parameters
	object_list_param = rospy.get_param('/object_list')
	dropbox_param = rospy.get_param('/dropbox')

# 	# TODO: Rotate PR2 in place to capture side tables for the collision map

# 	# TODO: Loop through the pick list
# 	# TODO: Parse parameters into individual variables
	for i in range(0, len(object_list_param)):
		object_name.data = object_list_param[i]['name']
		object_group = object_list_param[i]['group']
   
# 		# TODO: Get the PointCloud for a given object and obtain it's centroid
		for j in range(0, len(labels)):
			if object_name.data == labels[j]:
			   pick_pose.position.x = np.asscalar(centroids[j][0])
			   pick_pose.position.y = np.asscalar(centroids[j][1])
			   pick_pose.position.z = np.asscalar(centroids[j][2])
# 		# TODO: Create 'place_pose' for the object
		for j in range(0, len(dropbox_param)):
			if object_group == dropbox_param[j]['group']:
			   place_pose.position.x = dropbox_param[j]['position'][0]
			   place_pose.position.y = dropbox_param[j]['position'][1]
			   place_pose.position.z = dropbox_param[j]['position'][2]
# 		# TODO: Assign the arm to be used for pick_place
		if object_group == 'green':
		   arm_name.data = 'right'
		elif object_group == 'red':
		   arm_name.data = 'left'
# 		# TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
		yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
		dict_list.append(yaml_dict)


# 		# Wait for 'pick_place_routine' service to come up
# 		rospy.wait_for_service('pick_place_routine')

# 		try:
# 			pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

# 			# TODO: Insert your message variables to be sent as a service request
# 			resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

# 			print ("Response: ",resp.success)

# 		except rospy.ServiceException, e:
# 			print "Service call failed: %s"%e

# 	# TODO: Output your request parameters into output yaml file
	send_to_yaml(yaml_filename, dict_list)


if __name__ == '__main__':

	# TODO: ROS node initialization
	rospy.init_node('PerceptionProject', anonymous=True)
	# TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
	# TODO: Create Publishers
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
	pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
	object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
	detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
	# TODO: Load Model From disk
	model = pickle.load(open('model2.sav', 'rb'))
	clf = model['classifier']
	encoder = LabelEncoder()
	encoder.classes_ = model['classes']
	scaler = model['scaler']
	# Initialize color_list
	get_color_list.color_list = []

	# TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()