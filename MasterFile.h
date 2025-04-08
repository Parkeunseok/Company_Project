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


class MasterFile
{
	// tiff형식의 파일을 받기 위한 cv::Mat
	cv::Mat idealDepthMapBottom_; 
	cv::Mat idealDepthMapTop_;
	cv::Mat idealDepthMapSide_;

	int dataFullSizeBottom_;
	int dataFullSizeTop_;
	int dataFullSizeSide_;

	// point cloud vector
	pcl::PointCloud<pcl::PointXYZ> idealPointBottom_;
	pcl::PointCloud<pcl::PointXYZ> idealPointTop_;
	pcl::PointCloud<pcl::PointXYZ> idealPointSide_;


public:
	MasterFile();
	~MasterFile();
	
	void loadFileMaster();

	pcl::PointCloud<pcl::PointXYZ> returnIdealPointBottom();
	pcl::PointCloud<pcl::PointXYZ> returnIdealPointTop();
	pcl::PointCloud<pcl::PointXYZ> returnIdealPointSide();
};

