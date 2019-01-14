#!/usr/bin/env python

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

def get_normals(cloud):
	get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
	return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

	# TODO: Convert ROS msg to PCL data
	cloud = ros_to_pcl(pcl_msg)

	# TODO: Voxel Grid Downsampling
	# Create a VoxelGrid filter object for our input point cloud
	vox = cloud.make_voxel_grid_filter()

	# Choose a voxel (also known as leaf) size
	LEAF_SIZE = 0.002   

	# Set the voxel (or leaf) size  
	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

	# Call the filter function to obtain the resultant downsampled point cloud
	cloud_filtered = vox.filter()
	# filename = 'voxel_downsampled.pcd'
	# pcl.save(cloud_filtered, filename)

#################################################################################################

	# TODO: PassThrough Filter
	# Create a PassThrough filter object.
	passthrough = cloud_filtered.make_passthrough_filter()

	# Assign axis and range to the passthrough filter object.
	filter_axis = 'z'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = 0.6
	axis_max = 1.2
	passthrough.set_filter_limits(axis_min, axis_max)

	# Finally use the filter function to obtain the resultant point cloud. 
	cloud_filtered = passthrough.filter()
	# filename = 'pass_through_filtered.pcd'
	# pcl.save(cloud_filtered, filename)

#################################################################################################

	# TODO: RANSAC Plane Segmentation
	# Create the segmentation object
	seg = cloud_filtered.make_segmenter()

	# Set the model you wish to fit 
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)

	# Max distance for a point to be considered fitting the model
	max_distance = 0.02
	seg.set_distance_threshold(max_distance)

	# Call the segment function to obtain set of inlier indices and model coefficients
	inliers, coefficients = seg.segment()

################################################################################################

	# TODO: Extract inliers and outliers
	cloud_table = cloud_filtered.extract(inliers, negative=False)
	# filename = 'extracted_inliers.pcd'
	# pcl.save(cloud_table, filename)

	# Extract outliers
	cloud_objects = cloud_filtered.extract(inliers, negative=True)
	# filename = 'extracted_outliers.pcd'
	# pcl.save(cloud_objects, filename)

################################################################################################

	# TODO: Euclidean Clustering
	white_cloud = XYZRGB_to_XYZ(cloud_objects)
	tree = white_cloud.make_kdtree()

	# TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
	# Create a cluster extraction object
	ec = white_cloud.make_EuclideanClusterExtraction()
	# Set tolerances for distance threshold 
	# as well as minimum and maximum cluster size (in points)
	# NOTE: These are poor choices of clustering parameters
	# Your task is to experiment and find values that work for segmenting objects.
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
  

	# TODO: Convert PCL data to ROS messages
	ros_cloud_objects = pcl_to_ros(cloud_objects)
	ros_cloud_table = pcl_to_ros(cloud_table)
	ros_cluster_cloud = pcl_to_ros(cluster_cloud)

	# TODO: Publish ROS messages
	pcl_objects_pub.publish(ros_cloud_objects)
	pcl_table_pub.publish(ros_cloud_table)
	pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs: 

	# Classify the clusters! (loop through each detected cluster one at a time)
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

if __name__ == '__main__':

	# Create Publishers
	# TODO: here you need to create two publishers
	# Call them object_markers_pub and detected_objects_pub
	# Have them publish to "/object_markers" and "/detected_objects" with 
	# Message Types "Marker" and "DetectedObjectsArray" , respectively

	# Load Model From disk
	model = pickle.load(open('model.sav', 'rb'))
	clf = model['classifier']
	encoder = LabelEncoder()
	encoder.classes_ = model['classes']
	scaler = model['scaler']
	# TODO: ROS node initialization
	rospy.init_node('objrecog', anonymous=True)
	# TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
	# TODO: Create Publishers
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
	pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
	object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
	detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

	# Initialize color_list
	get_color_list.color_list = []
	# TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()