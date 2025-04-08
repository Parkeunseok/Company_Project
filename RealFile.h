#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/io.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <opencv2/opencv.hpp>

class RealFile
{
	cv::Mat realDepthMapBottom_;
	cv::Mat realDepthMapTop_;
	cv::Mat realDepthMapSide_;

	int dataFullSizeBottom_;
	int dataFullSizeTop_;
	int dataFullSizeSide_;

	// vector->point cloud
	pcl::PointCloud<pcl::PointXYZ> realPointBottom_;
	pcl::PointCloud<pcl::PointXYZ> realPointTop_;
	pcl::PointCloud<pcl::PointXYZ> realPointSide_;

public:
	RealFile();
	~RealFile();
	void loadFileReal();

	void vectorBottom2pointCloud(std::vector<cv::Point3f> vec_bot_);
	void vectorTop2pointCloud(std::vector<cv::Point3f> vec_top_);
	void vectorSide2pointCloud(std::vector<cv::Point3f> vec_side_);


	pcl::PointCloud<pcl::PointXYZ> returnRealPointBottom();
	pcl::PointCloud<pcl::PointXYZ> returnRealPointTop();
	pcl::PointCloud<pcl::PointXYZ> returnRealPointSide();
};

