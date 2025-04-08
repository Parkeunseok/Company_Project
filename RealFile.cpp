#include "RealFile.h"

RealFile::RealFile()
{

}

RealFile::~RealFile()
{

}
void RealFile::vectorBottom2pointCloud(std::vector<cv::Point3f> vec_bot_) // �Է¹��� real bottom �����͸� point cloud�� ����
{
	for (int i = 0; i < vec_bot_.size(); ++i)
	{
		pcl::PointXYZ pt_;
		pt_.x = vec_bot_.at(i).x;
		pt_.y = vec_bot_.at(i).y;
		pt_.z = vec_bot_.at(i).z;

		realPointBottom_.push_back(pt_);
	}
}

void RealFile::vectorTop2pointCloud(std::vector<cv::Point3f> vec_top_) // �Է¹��� real top �����͸� point cloud�� ����
{
	for (int i = 0; i < vec_top_.size(); ++i)
	{
		pcl::PointXYZ pt_;
		pt_.x = vec_top_.at(i).x;
		pt_.y = vec_top_.at(i).y;
		pt_.z = vec_top_.at(i).z;

		realPointTop_.push_back(pt_);
	}
}
void RealFile::vectorSide2pointCloud(std::vector<cv::Point3f> vec_side_) // �Է¹��� real side �����͸� point cloud�� ����
{
	for (int i = 0; i < vec_side_.size(); ++i)
	{
		pcl::PointXYZ pt_;
		pt_.x = vec_side_.at(i).x;
		pt_.y = vec_side_.at(i).y;
		pt_.z = vec_side_.at(i).z;

		realPointSide_.push_back(pt_);
	}
}

void RealFile::loadFileReal() // real���� �ε� (�������ֽ� PDF���Ͽ� �����ִ� ������ �ε� ����� ����)
{
	const float NULLVALUE = -999, SCALEX = 0.006, SCALEY = 0.1;

	std::string filePath_bottom = "C://my_job//aiv_project//3D_ġ��_����_����//REAL_BOTTOM.tiff";
	realDepthMapBottom_ = cv::imread(filePath_bottom, cv::IMREAD_UNCHANGED);
	auto width_bottom = realDepthMapBottom_.cols, height_bottom = realDepthMapBottom_.rows;
	auto imgPtr_bottom_ = realDepthMapBottom_.ptr<float>(0);

	// vector
	std::vector<cv::Point3f> realBottomPC(width_bottom * height_bottom);
	auto curldx_bottom = 0;
	for (int j = 0; j < height_bottom; ++j)
	{
		auto step = width_bottom * j;
		for (int i = 0; i < width_bottom; ++i)
		{
			if (NULLVALUE != imgPtr_bottom_[step + i])
			{
				realBottomPC[curldx_bottom].x = SCALEX * i;
				realBottomPC[curldx_bottom].y = SCALEY * j;
				realBottomPC[curldx_bottom].z = -imgPtr_bottom_[step + i];
				curldx_bottom++;
			}
		}
	}

	vectorBottom2pointCloud(realBottomPC); // PDF���Ͽ��� ������ ������ �������� �����͸� �Է¹޾� point cloud�� ���� 
	//std::cout << realPointBottom_.size() << std::endl;

	// ----------------------------------------------------------------------------------------
	std::string filePath_top = "C://my_job//aiv_project//3D_ġ��_����_����//REAL_TOP.tiff";
	realDepthMapTop_ = cv::imread(filePath_top, cv::IMREAD_UNCHANGED);
	auto width_top = realDepthMapTop_.cols, height_top = realDepthMapTop_.rows;
	auto imgPtr_top_ = realDepthMapTop_.ptr<float>(0);

	//vector
	std::vector<cv::Point3f> realTopPC(width_top * height_top);
	auto curldx_top = 0;
	for (int j = 0; j < height_top ; ++j)
	{
		auto step = width_top * j;
		for (int i = 0; i < width_top; ++i)
		{
			if (NULLVALUE != imgPtr_top_[step + i])
			{
				realTopPC[curldx_top].x = SCALEX * i;
				realTopPC[curldx_top].y = SCALEY * j;
				realTopPC[curldx_top].z = -imgPtr_top_[step + i];
			}
		}
	}
	vectorTop2pointCloud(realTopPC); // PDF���Ͽ��� ������ ������ �������� �����͸� �Է¹޾� point cloud�� ���� 
	//std::cout << realPointTop_.size() << std::endl;
	// ----------------------------------------------------------------------------------------
	
	std::string filePath_side = "C://my_job//aiv_project//3D_ġ��_����_����//REAL_SIDE.tiff";
	realDepthMapSide_ = cv::imread(filePath_side, cv::IMREAD_UNCHANGED);
	auto width_side = realDepthMapSide_.cols, height_side = realDepthMapSide_.rows;
	auto imgPtr_side_ = realDepthMapSide_.ptr<float>(0);

	//vector
	std::vector<cv::Point3f> realSidePC(width_side * height_side);
	auto curldx_side = 0;
	for (int j = 0; j < height_side; ++j)
	{
		auto step = width_side * j;
		for (int i = 0; i < width_side; ++i)
		{
			if (NULLVALUE != imgPtr_side_[step + i])
			{
				realSidePC[curldx_side].x = SCALEX * i;
				realSidePC[curldx_side].y = SCALEY * j;
				realSidePC[curldx_side].z = -imgPtr_side_[step + i];
			}
		}
	}
	vectorSide2pointCloud(realSidePC); // PDF���Ͽ��� ������ ������ �������� �����͸� �Է¹޾� point cloud�� ���� 

	//std::cout << realPointSide_.size() << std::endl;
	std::cout << "Real Data file load complete" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ> RealFile::returnRealPointBottom()
{
	return realPointBottom_; // point cloud�� ������ real bottom�����͸� ����
}

pcl::PointCloud<pcl::PointXYZ> RealFile::returnRealPointTop()
{
	return realPointTop_; // point cloud�� ������ real top�����͸� ����
}

pcl::PointCloud<pcl::PointXYZ> RealFile::returnRealPointSide()
{
	return realPointSide_; // point cloud�� ������ real side�����͸� ����
}