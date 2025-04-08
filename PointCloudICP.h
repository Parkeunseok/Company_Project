#pragma once
#define _CRT_SECURE_NO_WARNINGS

#include <pcl/registration/icp.h>

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
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/conversions.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "RealFile.h"
#include "MasterFile.h"

class PointCloudICP
{
	Eigen::Matrix4f tf_master_side_;
	Eigen::Matrix4f tf_master_top_;
	Eigen::Matrix4f tf_master_bottom_;

	RealFile icp_real_;
	MasterFile icp_master_;


	// icp result origin-real
	Eigen::Matrix4f input2output_bottom;
	Eigen::Matrix4f input2output_top;
	Eigen::Matrix4f input2output_side;

public:
	PointCloudICP();
	~PointCloudICP();

	void Origin2RealICP();


};

	