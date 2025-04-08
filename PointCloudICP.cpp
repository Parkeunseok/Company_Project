#include "PointCloudICP.h"


PointCloudICP::PointCloudICP()
{
    // merged 3D를 생성하기 위한 3개의 ideal point cloud에 대한 origin frame기준 transformation matrix 초기화
    tf_master_side_.setZero();
    tf_master_top_.setZero();
    tf_master_bottom_.setZero();

    tf_master_side_ << 
        0.0, 0.0, 1.0, 0.0,
        0.0, -1.0, 0.0, 537.0,
        1.0, 0.0, 0.0, -3.0,
        0.0, 0.0, 0.0, 1.0;

    tf_master_top_ <<
        -1.0, 0.0, 0.0, 7.0,
        0.0, -1.0, 0.0, 537.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    tf_master_bottom_ <<
        1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 537.0,
        0.0, 0.0, -1.0, 2.0,
        0.0, 0.0, 0.0, 1.0;
    std::cout << "Matrix Initialization" << std::endl;
}

PointCloudICP::~PointCloudICP()
{
   
}

void PointCloudICP::Origin2RealICP()
{
    icp_real_.loadFileReal(); // real 데이터 load함수 실행
    icp_master_.loadFileMaster(); // master 데이터 load함수 실행
    std::cout << "ICP on" << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_bottom_; // 데이터 voxelization수행
    voxel_bottom_.setLeafSize(0.5f, 0.5f, 0.5f);

    // Origin - ideal bottom
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_bottom(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_bottom(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered_bottom(new pcl::PointCloud<pcl::PointXYZ>);

    *input_bottom = icp_master_.returnIdealPointBottom();
    voxel_bottom_.setInputCloud(input_bottom);
    voxel_bottom_.filter(*input_filtered_bottom);
 

    //std::cout << input_bottom->size() << std::endl;
    pcl::transformPointCloud(*input_filtered_bottom, *output_bottom, tf_master_bottom_);

    // icp bottom
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_bottom_;
    icp_bottom_.setMaxCorrespondenceDistance(1.0);
    icp_bottom_.setTransformationEpsilon(0.001);
    icp_bottom_.setMaximumIterations(1000);

    pcl::PointCloud<pcl::PointXYZ>::Ptr align_bottom(new pcl::PointCloud <pcl::PointXYZ>);

    // registration bottom -> Origin - master
    icp_bottom_.setInputSource(input_filtered_bottom);
    icp_bottom_.setInputTarget(output_bottom);
    icp_bottom_.align(*align_bottom);

    // set outputs bottom
    input2output_bottom = icp_bottom_.getFinalTransformation();

    std::cout << input2output_bottom << std::endl;


    // Origin - ideal top
    pcl::VoxelGrid<pcl::PointXYZ> voxel_top_; // 데이터 voxelization수행
    voxel_top_.setLeafSize(0.5f, 0.5f, 0.5f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_top(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_top(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered_top(new pcl::PointCloud<pcl::PointXYZ>);

    *input_top = icp_master_.returnIdealPointTop();
    voxel_top_.setInputCloud(input_top);
    voxel_top_.filter(*input_filtered_top);

    pcl::transformPointCloud(*input_filtered_top, *output_top, tf_master_top_);


    // icp top
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_top_;
    icp_top_.setMaxCorrespondenceDistance(0.1);
    icp_top_.setTransformationEpsilon(0.001);
    icp_top_.setMaximumIterations(10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr align_top(new pcl::PointCloud <pcl::PointXYZ>);

    // registration top  -> Origin - master
    icp_top_.setInputSource(input_filtered_top);
    icp_top_.setInputTarget(output_top);
    icp_top_.align(*align_top);

    // set outputs top
    input2output_top = icp_top_.getFinalTransformation();

    std::cout << input2output_top << std::endl;


    // Origin - ideal side
    pcl::VoxelGrid<pcl::PointXYZ> voxel_side_; // 데이터 voxelization수행
    voxel_side_.setLeafSize(0.5f, 0.5f, 0.5f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_side(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_side(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered_side(new pcl::PointCloud<pcl::PointXYZ>);

    *input_side = icp_master_.returnIdealPointSide();
    voxel_side_.setInputCloud(input_side);
    voxel_side_.filter(*input_filtered_side);

    pcl::transformPointCloud(*input_filtered_side, *output_side, tf_master_side_);


    // icp side
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_side_;
    icp_side_.setMaxCorrespondenceDistance(0.1);
    icp_side_.setTransformationEpsilon(0.001);
    icp_side_.setMaximumIterations(10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr align_side(new pcl::PointCloud <pcl::PointXYZ>);

    // registration side  -> Origin - master
    icp_side_.setInputSource(input_filtered_side);
    icp_side_.setInputTarget(output_side);
    icp_side_.align(*align_side);

    // set outputs side
    input2output_side = icp_side_.getFinalTransformation();

    std::cout << input2output_side << std::endl;

    // 최종 변환 행렬 (origin → real)
    Eigen::Matrix4f tf_origin_real_bottom_ = tf_master_bottom_ * input2output_bottom;
    Eigen::Matrix4f tf_origin_real_top_ = tf_master_top_ * input2output_top;
    Eigen::Matrix4f tf_origin_real_side_ = tf_master_side_ * input2output_side;

    std::cout << "Origin to Real (Bottom): \n" << tf_origin_real_bottom_ << std::endl;
    std::cout << "Origin to Real (Top): \n" << tf_origin_real_top_ << std::endl;
    std::cout << "Origin to Real (Side): \n" << tf_origin_real_side_ << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr real_top_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr real_side_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr real_bottom_aligned(new pcl::PointCloud<pcl::PointXYZ>);

    // real data 가져오기
    pcl::PointCloud<pcl::PointXYZ>::Ptr real_top(new pcl::PointCloud<pcl::PointXYZ>);
    *real_top= icp_real_.returnRealPointTop();
    pcl::PointCloud<pcl::PointXYZ>::Ptr real_side(new pcl::PointCloud<pcl::PointXYZ>);
    *real_side = icp_real_.returnRealPointSide();
    pcl::PointCloud<pcl::PointXYZ>::Ptr real_bottom(new pcl::PointCloud<pcl::PointXYZ>);
     *real_bottom   = icp_real_.returnRealPointBottom();

    // 각각 origin 기준으로 변환
    pcl::transformPointCloud(*real_top, *real_top_aligned, tf_origin_real_top_);
    pcl::transformPointCloud(*real_side, *real_side_aligned, tf_origin_real_side_);
    pcl::transformPointCloud(*real_bottom, *real_bottom_aligned, tf_origin_real_bottom_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
    *merged += *real_top_aligned;
    *merged += *real_side_aligned;
    *merged += *real_bottom_aligned;
}